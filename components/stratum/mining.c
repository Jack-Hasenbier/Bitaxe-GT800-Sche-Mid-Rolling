// ============================================================
//  mining.c  –  Bitaxe GT800 optimiert + dynamisches Version‑Rolling
// ============================================================

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "mining.h"
#include "utils.h"
#include "mbedtls/sha256.h"
#include "esp_log.h"
#include "../../main/version_rolling.h"

static const char *TAG = "mining";

void free_bm_job(bm_job *job)
{
    free(job->jobid);
    free(job->extranonce2);
    free(job);
}

char *construct_coinbase_tx(const char *coinbase_1, const char *coinbase_2,
                            const char *extranonce, const char *extranonce_2)
{
    size_t len = strlen(coinbase_1) + strlen(extranonce) +
                 strlen(extranonce_2) + strlen(coinbase_2) + 1;
    char *coinbase_tx = malloc(len);
    if (!coinbase_tx) {
        ESP_LOGE(TAG, "construct_coinbase_tx: malloc failed");
        return NULL;
    }
    snprintf(coinbase_tx, len, "%s%s%s%s", coinbase_1, extranonce, extranonce_2, coinbase_2);
    return coinbase_tx;
}

void calculate_merkle_root_hash(const char *coinbase_tx, const uint8_t merkle_branches[][32],
                                const int num_merkle_branches, char dest[65])
{
    size_t coinbase_tx_bin_len = strlen(coinbase_tx) / 2;
    uint8_t *coinbase_tx_bin = malloc(coinbase_tx_bin_len);
    if (!coinbase_tx_bin) {
        ESP_LOGE(TAG, "calculate_merkle_root_hash: malloc failed");
        return;
    }
    hex2bin(coinbase_tx, coinbase_tx_bin, coinbase_tx_bin_len);
    uint8_t both_merkles[64];
    double_sha256_bin(coinbase_tx_bin, coinbase_tx_bin_len, both_merkles);
    free(coinbase_tx_bin);
    for (int i = 0; i < num_merkle_branches; i++) {
        memcpy(both_merkles + 32, merkle_branches[i], 32);
        double_sha256_bin(both_merkles, 64, both_merkles);
    }
    bin2hex(both_merkles, 32, dest, 65);
}

// Hilfsfunktion: Wendet optimierte Reihenfolge auf einen bm_job an
void version_rolling_apply_to_job(bm_job *job, uint32_t version_mask, const uint8_t order[4]) {
    // Die vier Midstates sind bereits in job->midstate, midstate1, midstate2, midstate3 vorhanden.
    uint8_t *midstate_ptrs[4] = {
        job->midstate,
        job->midstate1,
        job->midstate2,
        job->midstate3
    };
    uint8_t temp[4][32];
    for (int i = 0; i < 4; i++) {
        memcpy(temp[i], midstate_ptrs[i], 32);
    }
    for (int i = 0; i < 4; i++) {
        memcpy(midstate_ptrs[i], temp[order[i]], 32);
    }
    job->version_mask = version_mask;
}

bm_job construct_bm_job(mining_notify *params, const char *merkle_root,
                        const uint32_t version_mask, const uint32_t difficulty)
{
    bm_job new_job;
    memset(&new_job, 0, sizeof(bm_job));

    new_job.version = params->version;
    new_job.target = params->target;
    new_job.ntime = params->ntime;
    new_job.starting_nonce = 0;
    new_job.pool_diff = difficulty;

    hex2bin(merkle_root, new_job.merkle_root, 32);
    swap_endian_words(merkle_root, new_job.merkle_root_be);
    reverse_bytes(new_job.merkle_root_be, 32);

    swap_endian_words(params->prev_block_hash, new_job.prev_block_hash);
    hex2bin(params->prev_block_hash, new_job.prev_block_hash_be, 32);
    reverse_bytes(new_job.prev_block_hash_be, 32);

    uint8_t midstate_data[64];
    memcpy(midstate_data,      &new_job.version,          4);
    memcpy(midstate_data + 4,  new_job.prev_block_hash,  32);
    memcpy(midstate_data + 36, new_job.merkle_root,      28);

    midstate_sha256_bin(midstate_data, 64, new_job.midstate);
    reverse_bytes(new_job.midstate, 32);

    if (version_mask != 0) {
        uint32_t rolled_version = increment_bitmask(new_job.version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate1);
        reverse_bytes(new_job.midstate1, 32);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate2);
        reverse_bytes(new_job.midstate2, 32);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate3);
        reverse_bytes(new_job.midstate3, 32);

        new_job.num_midstates = 4;
    } else {
        new_job.num_midstates = 1;
    }

    // Dynamische Optimierung der Midstate-Reihenfolge
    const uint8_t *order = version_rolling_get_order();
    version_rolling_apply_to_job(&new_job, version_rolling_get_mask(), order);

    return new_job;
}

void extranonce_2_generate(uint64_t extranonce_2, uint32_t length,
                           char dest[static length * 2 + 1])
{
    uint8_t extranonce_2_bytes[length];
    memset(extranonce_2_bytes, 0, length);
    size_t copy_len = (length < (uint32_t)sizeof(uint64_t)) ? (size_t)length : sizeof(uint64_t);
    memcpy(extranonce_2_bytes, &extranonce_2, copy_len);
    bin2hex(extranonce_2_bytes, length, dest, length * 2 + 1);
}

static const double truediffone = 26959535291011309493156476344723991336010898738574164086137773096960.0;

double test_nonce_value(const bm_job *job, const uint32_t nonce, const uint32_t rolled_version)
{
    double d64, s64, ds;
    unsigned char header[80];
    memcpy(header,      &rolled_version,     4);
    memcpy(header + 4,  job->prev_block_hash, 32);
    memcpy(header + 36, job->merkle_root,     32);
    memcpy(header + 68, &job->ntime,          4);
    memcpy(header + 72, &job->target,         4);
    memcpy(header + 76, &nonce,               4);

    unsigned char hash_buffer[32];
    unsigned char hash_result[32];

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, header, 80);
    mbedtls_sha256_finish(&ctx, hash_buffer);
    mbedtls_sha256_free(&ctx);

    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, hash_buffer, 32);
    mbedtls_sha256_finish(&ctx, hash_result);
    mbedtls_sha256_free(&ctx);

    d64 = truediffone;
    s64 = le256todouble(hash_result);
    ds  = d64 / s64;
    return ds;
}

uint32_t increment_bitmask(const uint32_t value, const uint32_t mask)
{
    if (mask == 0) return value;
    uint32_t carry = (value & mask) + (mask & -mask);
    uint32_t overflow = carry & ~mask;
    uint32_t new_value = (value & ~mask) | (carry & mask);
    if (overflow > 0) {
        uint32_t carry_mask = (overflow << 1);
        new_value = increment_bitmask(new_value, carry_mask);
    }
    return new_value;
}