#ifndef SDLOG_H
#define SDLOG_H

#include <stdint.h>
#include <stdbool.h>

/* Sdílené příznaky (definované v sdlog.c) */
extern volatile uint8_t flag_sd_toggle;
extern volatile uint8_t sd_logging;

/**
 * @brief Inicializace proměnných pro logování.
 */
void sd_log_init(void);

/**
 * @brief Spustí logování (Mount + Open).
 * @return 0 pokud OK.
 */
int sd_log_start(void);

/**
 * @brief Zastaví logování (Write finalize + Unmount).
 */
void sd_log_stop(void);

/**
 * @brief Zapíše řádek dat do otevřeného souboru.
 */
void sd_log_append_line(float T, float P, float H, uint16_t L);

#endif /* SDLOG_H */