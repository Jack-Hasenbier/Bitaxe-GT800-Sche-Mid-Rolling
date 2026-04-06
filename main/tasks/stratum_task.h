#ifndef STRATUM_TASK_H_
#define STRATUM_TASK_H_

/**
 * @brief Haupttask für Stratum-Verbindung und Nachrichtenverarbeitung
 * @param pvParameters Zeiger auf GlobalState
 */
void stratum_task(void *pvParameters);

/**
 * @brief Schliesst die aktive Stratum-Verbindung und bereinigt Warteschlangen
 * @param GLOBAL_STATE globaler Zustand
 */
void stratum_close_connection(GlobalState * GLOBAL_STATE);

#endif /* STRATUM_TASK_H_ */