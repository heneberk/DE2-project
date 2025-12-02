/*
 * sdlog.c
 * Implementace logování na SD kartu pomocí Petit FatFs.
 */

// POZOR: Includy musí být v tomto pořadí pro správnou kompilaci
#include "ds1302.h"        // Typy pro čas
#include "loggerControl.h" // Přístup k g_time

#include "sdlog.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // pro abs()
#include "pff.h"
#include "diskio.h"
#include "uart.h"

// --- Konfigurace ---
#define LOG_FILENAME "DATA.TXT"

// --- Globální proměnné ---
FATFS fs;                   
volatile uint8_t flag_sd_toggle = 0; 
volatile uint8_t sd_logging = 0;     

void sd_log_init(void)
{
    sd_logging = 0;
    flag_sd_toggle = 0;
}

int sd_log_start(void)
{
    if (sd_logging) return 0; 

    FRESULT res;

    uart_puts("SD: Mounting...\r\n");
    res = pf_mount(&fs);
    if (res != FR_OK) {
        uart_puts("SD: Mount Error!\r\n");
        return res;
    }

    uart_puts("SD: Opening file...\r\n");
    // Soubor DATA.TXT musí na kartě existovat a mít velikost!
    res = pf_open(LOG_FILENAME);
    if (res != FR_OK) {
        uart_puts("SD: Open Error! (Check DATA.TXT)\r\n");
        return res;
    }

    // Seek na začátek (přepisování souboru)
    res = pf_lseek(0);
    if (res != FR_OK) {
        uart_puts("SD: Seek Error!\r\n");
        return res;
    }

    sd_logging = 1;
    uart_puts("SD: Logging started.\r\n");
    return 0;
}

void sd_log_stop(void)
{
    if (!sd_logging) return;

    UINT bw;
    // Finalizace zápisu (flush neúplného sektoru)
    pf_write(0, 0, &bw);
    
    pf_mount(NULL); // Unmount
    
    sd_logging = 0;
    uart_puts("SD: Logging stopped.\r\n");
}

void sd_log_append_line(float T, float P, float H, uint16_t L)
{
    if (!sd_logging) return;

    char buffer[64];
    UINT bw;
    FRESULT res;

    // Převod float na int/dec pro spolehlivý tisk
    int t_int = (int)T;
    int t_dec = abs((int)((T - t_int) * 100));
    
    int p_int = (int)P;
    int p_dec = abs((int)((P - p_int) * 100));

    int h_int = (int)H;
    int h_dec = abs((int)((H - h_int) * 100));

    // Použití g_time z loggerControl.h
    sprintf(buffer, "%02d:%02d:%02d, %d.%02d, %d.%02d, %d.%02d, %u\r\n",
            g_time.hh, g_time.mm, g_time.ss,
            t_int, t_dec,
            p_int, p_dec,
            h_int, h_dec,
            L);

    // Zápis
    res = pf_write(buffer, strlen(buffer), &bw);

    if (res != FR_OK) {
        uart_puts("SD: Write Error!\r\n");
    } else if (bw < strlen(buffer)) {
        uart_puts("SD: Disk Full!\r\n");
        sd_log_stop();
    } else {
        uart_puts("LOG: ");
        uart_puts(buffer);
    }
}