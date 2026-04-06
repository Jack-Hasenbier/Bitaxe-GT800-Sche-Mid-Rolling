#include "esp_log.h"
#include "connect.h"
#include "system.h"
#include "global_state.h"
#include "lwip/dns.h"
#include <lwip/tcpip.h>
#include <lwip/netdb.h>
#include "nvs_config.h"
#include "stratum_task.h"
#include "work_queue.h"
#include "esp_wifi.h"
#include <esp_sntp.h>
#include <time.h>
#include <sys/time.h>
#include "esp_timer.h"
#include <stdbool.h>
#include "utils.h"
#include <pthread.h>
#include <fcntl.h>

// ---------- Externe Stratum-API-Variable ----------
extern StratumApiV1Message stratum_api_v1_message;

// ---------- Definition des fehlenden Typs ----------
typedef struct {
    struct sockaddr_storage dest_addr;
    socklen_t addrlen;
    int addr_family;
    int ip_protocol;
    char host_ip[INET6_ADDRSTRLEN + 16];
} stratum_connection_info_t;

// ---------- Fehlende globale Variablen ----------
static const char *primary_stratum_url = NULL;
static uint16_t primary_stratum_port = 0;

// ---------- Konstanten ----------
#define MAX_RETRY_ATTEMPTS              3
#define MAX_CRITICAL_RETRY_ATTEMPTS     5
#define MAX_EXTRANONCE_2_LEN            32
#define BUFFER_SIZE                     1024
#define DNS_CACHE_TTL_SEC               300
#define CONNECT_TIMEOUT_SEC             10
#define HEARTBEAT_INTERVAL_MS           10000
#define HEARTBEAT_RETRY_DELAY_MS        60000

#define COINBASE_VERSION_LEN            4
#define COINBASE_INPUT_COUNT_LEN        1
#define COINBASE_PREV_HASH_LEN          32
#define COINBASE_VOUT_LEN               4
#define COINBASE_SCRIPTSIG_LEN_OFFSET   41
#define COINBASE_BLOCK_HEIGHT_OFFSET    (COINBASE_SCRIPTSIG_LEN_OFFSET + 1)

static const char * TAG = "stratum_task";

// ---------- Statische Hilfsfunktionen ----------
static bool s_wifi_connected = false;

typedef struct {
    char hostname[256];
    uint16_t port;
    struct sockaddr_storage addr;
    socklen_t addrlen;
    int addr_family;
    int ip_protocol;
    char host_ip[INET6_ADDRSTRLEN + 16];
    time_t expires;
} dns_cache_entry_t;

static dns_cache_entry_t s_dns_cache = {0};
static pthread_mutex_t s_stratum_mutex = PTHREAD_MUTEX_INITIALIZER;

static bool is_wifi_connected_fast(void) {
    return s_wifi_connected;
}

void stratum_set_wifi_connected(bool connected) {
    s_wifi_connected = connected;
}

static esp_err_t resolve_stratum_address_cached(const char *hostname, uint16_t port,
                                                stratum_connection_info_t *conn_info) {
    time_t now = time(NULL);
    if (s_dns_cache.expires > now && strcmp(s_dns_cache.hostname, hostname) == 0 && s_dns_cache.port == port) {
        memcpy(&conn_info->dest_addr, &s_dns_cache.addr, s_dns_cache.addrlen);
        conn_info->addrlen = s_dns_cache.addrlen;
        conn_info->addr_family = s_dns_cache.addr_family;
        conn_info->ip_protocol = s_dns_cache.ip_protocol;
        strcpy(conn_info->host_ip, s_dns_cache.host_ip);
        ESP_LOGD(TAG, "Using cached DNS for %s", hostname);
        return ESP_OK;
    }

    struct addrinfo hints = {
        .ai_family = AF_UNSPEC,
        .ai_socktype = SOCK_STREAM,
        .ai_protocol = IPPROTO_TCP,
        .ai_flags = AI_NUMERICSERV
    };
    struct addrinfo *res;
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", port);

    int gai_err = getaddrinfo(hostname, port_str, &hints, &res);
    if (gai_err != 0) {
        ESP_LOGE(TAG, "getaddrinfo failed for %s: error code %d", hostname, gai_err);
        return ESP_FAIL;
    }

    memset(conn_info, 0, sizeof(stratum_connection_info_t));
    conn_info->addr_family = AF_UNSPEC;

    struct addrinfo *p;
    for (p = res; p != NULL; p = p->ai_next) {
        if (p->ai_family == AF_INET6) {
            memcpy(&conn_info->dest_addr, p->ai_addr, p->ai_addrlen);
            conn_info->addrlen = p->ai_addrlen;
            conn_info->addr_family = AF_INET6;
            conn_info->ip_protocol = IPPROTO_IPV6;
            break;
        }
    }
    if (conn_info->addr_family == AF_UNSPEC) {
        for (p = res; p != NULL; p = p->ai_next) {
            if (p->ai_family == AF_INET) {
                memcpy(&conn_info->dest_addr, p->ai_addr, p->ai_addrlen);
                conn_info->addrlen = p->ai_addrlen;
                conn_info->addr_family = AF_INET;
                conn_info->ip_protocol = IPPROTO_IP;
                break;
            }
        }
    }

    if (conn_info->addr_family == AF_UNSPEC) {
        freeaddrinfo(res);
        ESP_LOGE(TAG, "No suitable address found for %s", hostname);
        return ESP_FAIL;
    }

    if (conn_info->addr_family == AF_INET6) {
        struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)&conn_info->dest_addr;
        inet_ntop(AF_INET6, &addr6->sin6_addr, conn_info->host_ip, sizeof(conn_info->host_ip));
        if (IN6_IS_ADDR_LINKLOCAL(&addr6->sin6_addr) && addr6->sin6_scope_id != 0) {
            char zone_buf[16];
            snprintf(zone_buf, sizeof(zone_buf), "%%%lu", (unsigned long)addr6->sin6_scope_id);
            strncat(conn_info->host_ip, zone_buf, sizeof(conn_info->host_ip) - strlen(conn_info->host_ip) - 1);
        }
    } else {
        inet_ntop(AF_INET, &((struct sockaddr_in *)&conn_info->dest_addr)->sin_addr,
                  conn_info->host_ip, sizeof(conn_info->host_ip));
    }

    strncpy(s_dns_cache.hostname, hostname, sizeof(s_dns_cache.hostname) - 1);
    s_dns_cache.port = port;
    memcpy(&s_dns_cache.addr, &conn_info->dest_addr, conn_info->addrlen);
    s_dns_cache.addrlen = conn_info->addrlen;
    s_dns_cache.addr_family = conn_info->addr_family;
    s_dns_cache.ip_protocol = conn_info->ip_protocol;
    strcpy(s_dns_cache.host_ip, conn_info->host_ip);
    s_dns_cache.expires = now + DNS_CACHE_TTL_SEC;

    freeaddrinfo(res);
    return ESP_OK;
}

static int tcp_connect_timeout(int sock, const struct sockaddr *addr, socklen_t addrlen, int timeout_sec) {
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1) return -1;
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) return -1;

    int ret = connect(sock, addr, addrlen);
    if (ret == 0) {
        fcntl(sock, F_SETFL, flags);
        return 0;
    }
    if (errno != EINPROGRESS) {
        fcntl(sock, F_SETFL, flags);
        return -1;
    }

    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(sock, &fdset);
    struct timeval tv = { .tv_sec = timeout_sec, .tv_usec = 0 };
    ret = select(sock + 1, NULL, &fdset, NULL, &tv);
    if (ret <= 0) {
        fcntl(sock, F_SETFL, flags);
        return -1;
    }

    int so_error = 0;
    socklen_t len = sizeof(so_error);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0 || so_error != 0) {
        fcntl(sock, F_SETFL, flags);
        return -1;
    }

    fcntl(sock, F_SETFL, flags);
    return 0;
}

static void set_string_mutex(char **target, char *new_value) {
    pthread_mutex_lock(&s_stratum_mutex);
    char *old = *target;
    *target = new_value;
    pthread_mutex_unlock(&s_stratum_mutex);
    free(old);
}

static char* get_string_mutex(char **source) {
    pthread_mutex_lock(&s_stratum_mutex);
    char *copy = (*source) ? strdup(*source) : NULL;
    pthread_mutex_unlock(&s_stratum_mutex);
    return copy;
}

void cleanQueue(GlobalState * GLOBAL_STATE) {
    ESP_LOGI(TAG, "Clean Jobs: clearing queue");
    pthread_mutex_lock(&s_stratum_mutex);
    GLOBAL_STATE->abandon_work = 1;
    pthread_mutex_unlock(&s_stratum_mutex);

    while (GLOBAL_STATE->stratum_queue.count > 0) {
        mining_notify *notify = (mining_notify*) queue_dequeue(&GLOBAL_STATE->stratum_queue);
        if (notify) {
            STRATUM_V1_free_mining_notify(notify);
        }
    }
    queue_clear(&GLOBAL_STATE->stratum_queue);

    pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
    ASIC_jobs_queue_clear(&GLOBAL_STATE->ASIC_jobs_queue);
    for (int i = 0; i < 128; i += 4) {
        GLOBAL_STATE->valid_jobs[i] = 0;
    }
    pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);
}

void stratum_reset_uid(GlobalState * GLOBAL_STATE) {
    ESP_LOGI(TAG, "Resetting stratum uid");
    GLOBAL_STATE->send_uid = 1;
}

void stratum_close_connection(GlobalState * GLOBAL_STATE) {
    pthread_mutex_lock(&s_stratum_mutex);
    int sock = GLOBAL_STATE->sock;
    GLOBAL_STATE->sock = -1;
    pthread_mutex_unlock(&s_stratum_mutex);

    if (sock < 0) {
        ESP_LOGE(TAG, "Socket already shutdown, not shutting down again..");
        return;
    }

    ESP_LOGE(TAG, "Shutting down socket and restarting...");
    shutdown(sock, SHUT_RDWR);
    close(sock);
    cleanQueue(GLOBAL_STATE);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void stratum_primary_heartbeat(void * pvParameters) {
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    ESP_LOGI(TAG, "Starting heartbeat thread for primary pool: %s:%d",
             primary_stratum_url, primary_stratum_port);
    vTaskDelay(pdMS_TO_TICKS(10000));

    while (1) {
        if (!GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback) {
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
            continue;
        }

        if (!is_wifi_connected_fast()) {
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
            continue;
        }

        stratum_connection_info_t conn_info;
        if (resolve_stratum_address_cached(primary_stratum_url, primary_stratum_port, &conn_info) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_RETRY_DELAY_MS));
            continue;
        }

        int sock = socket(conn_info.addr_family, SOCK_STREAM, conn_info.ip_protocol);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_RETRY_DELAY_MS));
            continue;
        }

        int err = tcp_connect_timeout(sock, (struct sockaddr *)&conn_info.dest_addr,
                                      conn_info.addrlen, CONNECT_TIMEOUT_SEC);
        close(sock);

        if (err == 0 && !GLOBAL_STATE->SYSTEM_MODULE.use_fallback_stratum) {
            ESP_LOGI(TAG, "Heartbeat successful and in fallback mode. Switching back to primary.");
            GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback = false;
            stratum_close_connection(GLOBAL_STATE);
        }

        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_RETRY_DELAY_MS));
    }
}

static void decode_mining_notification(GlobalState * GLOBAL_STATE, const mining_notify *mining_notification) {
    double network_difficulty = networkDifficulty(mining_notification->target);
    GLOBAL_STATE->network_nonce_diff = (uint64_t) network_difficulty;
    suffixString(network_difficulty, GLOBAL_STATE->network_diff_string, DIFF_STRING_SIZE, 0);

    int coinbase_1_len = strlen(mining_notification->coinbase_1) / 2;
    int coinbase_2_len = strlen(mining_notification->coinbase_2) / 2;

    const int coinbase_scriptsig_offset = COINBASE_SCRIPTSIG_LEN_OFFSET;
    if (coinbase_1_len < coinbase_scriptsig_offset) return;

    uint8_t scriptsig_len;
    hex2bin(mining_notification->coinbase_1 + (coinbase_scriptsig_offset * 2), &scriptsig_len, 1);
    int pos = coinbase_scriptsig_offset + 1;

    if (coinbase_1_len < pos) return;

    uint8_t block_height_len;
    hex2bin(mining_notification->coinbase_1 + (pos * 2), &block_height_len, 1);
    pos++;

    if (coinbase_1_len < pos || block_height_len == 0 || block_height_len > 4) return;

    uint32_t block_height = 0;
    hex2bin(mining_notification->coinbase_1 + (pos * 2), (uint8_t *)&block_height, block_height_len);
    pos += block_height_len;

    if (block_height != GLOBAL_STATE->block_height) {
        ESP_LOGI(TAG, "Block height %d", block_height);
        GLOBAL_STATE->block_height = block_height;
    }

    size_t scriptsig_length = scriptsig_len - 1 - block_height_len;
    if (coinbase_1_len - pos < scriptsig_len - 1 - block_height_len) {
        scriptsig_length -= (strlen(GLOBAL_STATE->extranonce_str) / 2) + GLOBAL_STATE->extranonce_2_len;
    }
    if (scriptsig_length <= 0) return;

    char * scriptsig = malloc(scriptsig_length + 1);
    if (!scriptsig) return;

    int coinbase_1_tag_len = coinbase_1_len - pos;
    if (coinbase_1_tag_len > scriptsig_length) {
        coinbase_1_tag_len = scriptsig_length;
    }
    hex2bin(mining_notification->coinbase_1 + (pos * 2), (uint8_t *)scriptsig, coinbase_1_tag_len);

    int coinbase_2_tag_len = scriptsig_length - coinbase_1_tag_len;
    if (coinbase_2_len < coinbase_2_tag_len) {
        free(scriptsig);
        return;
    }
    if (coinbase_2_tag_len > 0) {
        hex2bin(mining_notification->coinbase_2, (uint8_t *)scriptsig + coinbase_1_tag_len, coinbase_2_tag_len);
    }

    for (int i = 0; i < scriptsig_length; i++) {
        if (!isprint((unsigned char)scriptsig[i])) {
            scriptsig[i] = '.';
        }
    }
    scriptsig[scriptsig_length] = '\0';

    char *old_scriptsig = get_string_mutex(&GLOBAL_STATE->scriptsig);
    if (old_scriptsig == NULL || strcmp(scriptsig, old_scriptsig) != 0) {
        ESP_LOGI(TAG, "Scriptsig: %s", scriptsig);
        set_string_mutex(&GLOBAL_STATE->scriptsig, scriptsig);
        free(old_scriptsig);
    } else {
        free(scriptsig);
        free(old_scriptsig);
    }
}

void stratum_task(void * pvParameters) {
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    primary_stratum_url = GLOBAL_STATE->SYSTEM_MODULE.pool_url;
    primary_stratum_port = GLOBAL_STATE->SYSTEM_MODULE.pool_port;

    char * stratum_url = GLOBAL_STATE->SYSTEM_MODULE.pool_url;
    uint16_t port = GLOBAL_STATE->SYSTEM_MODULE.pool_port;
    bool extranonce_subscribe = GLOBAL_STATE->SYSTEM_MODULE.pool_extranonce_subscribe;
    uint16_t difficulty = GLOBAL_STATE->SYSTEM_MODULE.pool_difficulty;

    STRATUM_V1_initialize_buffer();
    int retry_attempts = 0;
    int retry_critical_attempts = 0;

    xTaskCreateWithCaps(stratum_primary_heartbeat, "stratum primary heartbeat",
                        8192, pvParameters, 1, NULL, MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG, "Opening connection to pool: %s:%d", stratum_url, port);

    while (1) {
        if (!is_wifi_connected_fast()) {
            ESP_LOGI(TAG, "WiFi disconnected, attempting to reconnect...");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        if (retry_attempts >= MAX_RETRY_ATTEMPTS) {
            if (GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_url == NULL ||
                GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_url[0] == '\0') {
                ESP_LOGI(TAG, "Unable to switch to fallback. No url configured. (retries: %d)...", retry_attempts);
                GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback = false;
                retry_attempts = 0;
                continue;
            }

            GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback = !GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback;

            for (int i = 0; i < GLOBAL_STATE->SYSTEM_MODULE.rejected_reason_stats_count; i++) {
                GLOBAL_STATE->SYSTEM_MODULE.rejected_reason_stats[i].count = 0;
                GLOBAL_STATE->SYSTEM_MODULE.rejected_reason_stats[i].message[0] = '\0';
            }
            GLOBAL_STATE->SYSTEM_MODULE.rejected_reason_stats_count = 0;
            GLOBAL_STATE->SYSTEM_MODULE.shares_accepted = 0;
            GLOBAL_STATE->SYSTEM_MODULE.shares_rejected = 0;
            GLOBAL_STATE->SYSTEM_MODULE.work_received = 0;

            ESP_LOGI(TAG, "Switching target due to too many failures (retries: %d)...", retry_attempts);
            retry_attempts = 0;
        }

        stratum_url = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ?
                      GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_url :
                      GLOBAL_STATE->SYSTEM_MODULE.pool_url;
        port = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ?
               GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_port :
               GLOBAL_STATE->SYSTEM_MODULE.pool_port;
        extranonce_subscribe = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ?
                               GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_extranonce_subscribe :
                               GLOBAL_STATE->SYSTEM_MODULE.pool_extranonce_subscribe;
        difficulty = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ?
                     GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_difficulty :
                     GLOBAL_STATE->SYSTEM_MODULE.pool_difficulty;

        stratum_connection_info_t conn_info;
        if (resolve_stratum_address_cached(stratum_url, port, &conn_info) != ESP_OK) {
            ESP_LOGE(TAG, "Address resolution failed for %s", stratum_url);
            retry_attempts++;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Connecting to: stratum+tcp://%s:%d (%s)", stratum_url, port, conn_info.host_ip);

        int sock = socket(conn_info.addr_family, SOCK_STREAM, conn_info.ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            if (++retry_critical_attempts > MAX_CRITICAL_RETRY_ATTEMPTS) {
                ESP_LOGE(TAG, "Max retry attempts reached, restarting...");
                esp_restart();
            }
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        retry_critical_attempts = 0;

        if (tcp_connect_timeout(sock, (struct sockaddr *)&conn_info.dest_addr,
                                conn_info.addrlen, CONNECT_TIMEOUT_SEC) != 0) {
            retry_attempts++;
            ESP_LOGE(TAG, "Socket unable to connect to %s:%d (errno %d: %s)",
                     stratum_url, port, errno, strerror(errno));
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        struct timeval tcp_snd_timeout = { .tv_sec = 5, .tv_usec = 0 };
        struct timeval tcp_rcv_timeout = { .tv_sec = 180, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tcp_snd_timeout, sizeof(tcp_snd_timeout));
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tcp_rcv_timeout, sizeof(tcp_rcv_timeout));

        pthread_mutex_lock(&s_stratum_mutex);
        GLOBAL_STATE->sock = sock;
        pthread_mutex_unlock(&s_stratum_mutex);

        GLOBAL_STATE->SYSTEM_MODULE.pool_addr_family = conn_info.addr_family;

        stratum_reset_uid(GLOBAL_STATE);
        cleanQueue(GLOBAL_STATE);
        GLOBAL_STATE->abandon_work = 0;

        STRATUM_V1_configure_version_rolling(sock, GLOBAL_STATE->send_uid++, &GLOBAL_STATE->version_mask);
        STRATUM_V1_subscribe(sock, GLOBAL_STATE->send_uid++, GLOBAL_STATE->DEVICE_CONFIG.family.asic.name);

        char *username = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ?
                         GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_user :
                         GLOBAL_STATE->SYSTEM_MODULE.pool_user;
        char *password = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ?
                         GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_pass :
                         GLOBAL_STATE->SYSTEM_MODULE.pool_pass;

        int authorize_message_id = GLOBAL_STATE->send_uid++;
        STRATUM_V1_authorize(sock, authorize_message_id, username, password);
        STRATUM_V1_stamp_tx(authorize_message_id);

        while (1) {
            char *line = STRATUM_V1_receive_jsonrpc_line(sock);
            if (!line) {
                ESP_LOGE(TAG, "Failed to receive JSON-RPC line, reconnecting...");
                retry_attempts++;
                stratum_close_connection(GLOBAL_STATE);
                break;
            }

            double response_time_ms = STRATUM_V1_get_response_time_ms(stratum_api_v1_message.message_id);
            if (response_time_ms >= 0) {
                ESP_LOGI(TAG, "Stratum response time: %.2f ms", response_time_ms);
                GLOBAL_STATE->SYSTEM_MODULE.response_time = response_time_ms;
            }

            STRATUM_V1_parse(&stratum_api_v1_message, line);
            free(line);

            if (stratum_api_v1_message.method == MINING_NOTIFY) {
                GLOBAL_STATE->SYSTEM_MODULE.work_received++;
                SYSTEM_notify_new_ntime(GLOBAL_STATE, stratum_api_v1_message.mining_notification->ntime);

                if (stratum_api_v1_message.should_abandon_work) {
                    pthread_mutex_lock(&s_stratum_mutex);
                    bool abandon = (GLOBAL_STATE->stratum_queue.count > 0 || GLOBAL_STATE->ASIC_jobs_queue.count > 0);
                    pthread_mutex_unlock(&s_stratum_mutex);
                    if (abandon) {
                        cleanQueue(GLOBAL_STATE);
                    }
                }

                if (GLOBAL_STATE->stratum_queue.count == QUEUE_SIZE) {
                    mining_notify *old = (mining_notify*) queue_dequeue(&GLOBAL_STATE->stratum_queue);
                    STRATUM_V1_free_mining_notify(old);
                }
                queue_enqueue(&GLOBAL_STATE->stratum_queue, stratum_api_v1_message.mining_notification);
                decode_mining_notification(GLOBAL_STATE, stratum_api_v1_message.mining_notification);
            }
            else if (stratum_api_v1_message.method == MINING_SET_DIFFICULTY) {
                ESP_LOGI(TAG, "Set pool difficulty: %ld", stratum_api_v1_message.new_difficulty);
                GLOBAL_STATE->pool_difficulty = stratum_api_v1_message.new_difficulty;
                GLOBAL_STATE->new_set_mining_difficulty_msg = true;
            }
            else if (stratum_api_v1_message.method == MINING_SET_VERSION_MASK ||
                     stratum_api_v1_message.method == STRATUM_RESULT_VERSION_MASK) {
                ESP_LOGI(TAG, "Set version mask: %08lx", stratum_api_v1_message.version_mask);
                GLOBAL_STATE->version_mask = stratum_api_v1_message.version_mask;
                GLOBAL_STATE->new_stratum_version_rolling_msg = true;
            }
            else if (stratum_api_v1_message.method == MINING_SET_EXTRANONCE ||
                     stratum_api_v1_message.method == STRATUM_RESULT_SUBSCRIBE) {
                if (stratum_api_v1_message.extranonce_2_len > MAX_EXTRANONCE_2_LEN) {
                    ESP_LOGW(TAG, "Extranonce_2_len %d exceeds maximum %d, clamping",
                             stratum_api_v1_message.extranonce_2_len, MAX_EXTRANONCE_2_LEN);
                    stratum_api_v1_message.extranonce_2_len = MAX_EXTRANONCE_2_LEN;
                }
                ESP_LOGI(TAG, "Set extranonce: %s, extranonce_2_len: %d",
                         stratum_api_v1_message.extranonce_str, stratum_api_v1_message.extranonce_2_len);
                set_string_mutex(&GLOBAL_STATE->extranonce_str, stratum_api_v1_message.extranonce_str);
                GLOBAL_STATE->extranonce_2_len = stratum_api_v1_message.extranonce_2_len;
            }
            else if (stratum_api_v1_message.method == CLIENT_RECONNECT) {
                ESP_LOGE(TAG, "Pool requested client reconnect...");
                stratum_close_connection(GLOBAL_STATE);
                break;
            }
            else if (stratum_api_v1_message.method == STRATUM_RESULT) {
                if (stratum_api_v1_message.response_success) {
                    ESP_LOGI(TAG, "message result accepted");
                    SYSTEM_notify_accepted_share(GLOBAL_STATE);
                } else {
                    ESP_LOGW(TAG, "message result rejected: %s", stratum_api_v1_message.error_str);
                    SYSTEM_notify_rejected_share(GLOBAL_STATE, stratum_api_v1_message.error_str);
                }
            }
            else if (stratum_api_v1_message.method == STRATUM_RESULT_SETUP) {
                retry_attempts = 0;
                if (stratum_api_v1_message.response_success) {
                    ESP_LOGI(TAG, "setup message accepted");
                    if (stratum_api_v1_message.message_id == authorize_message_id && difficulty > 0) {
                        STRATUM_V1_suggest_difficulty(sock, GLOBAL_STATE->send_uid++, difficulty);
                    }
                    if (extranonce_subscribe) {
                        STRATUM_V1_extranonce_subscribe(sock, GLOBAL_STATE->send_uid++);
                    }
                } else {
                    ESP_LOGE(TAG, "setup message rejected: %s", stratum_api_v1_message.error_str);
                }
            }
        }
    }
    vTaskDelete(NULL);
}