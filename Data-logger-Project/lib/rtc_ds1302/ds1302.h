#ifndef DS1302_H
#define DS1302_H

#include <stdint.h>
#include <avr/io.h>

/* --- Nastavení Pinů (Upravte dle zapojení) --- */
/* Předpokládáme PORTB. Pokud máte jiný port, změňte PORT, DDR, PIN */
#define DS1302_PORT PORTB
#define DS1302_DDR  DDRB
#define DS1302_PIN  PINB

/* Čísla bitů na portu (0-7) */
#define DS1302_CE_PIN   3  /* Chip Enable (označováno jako RST) */
#define DS1302_IO_PIN   4  /* Data I/O (označováno jako DAT)    */
#define DS1302_SCLK_PIN 5  /* Serial Clock (označováno jako CLK)*/

/* Struktura pro uchování času */
typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t date;
    uint8_t month;
    uint8_t day;
    uint8_t year;
} ds1302_time_t;

/* Funkce */

/**
 * @brief Inicializace portů pro DS1302 a zapnutí hodin (pokud stojí).
 */
void ds1302_init(void);

/**
 * @brief Přečte kompletní čas z modulu (Burst read).
 * @param time Ukazatel na strukturu, kam se uloží výsledek.
 */
void ds1302_read_time(ds1302_time_t *time);

/**
 * @brief Nastaví čas v modulu.
 * @param time Ukazatel na strukturu s novým časem.
 */
void ds1302_set_time(ds1302_time_t *time);

#endif