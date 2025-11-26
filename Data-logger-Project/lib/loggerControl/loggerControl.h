#ifndef LOGGER_CONTROL_H
#define LOGGER_CONTROL_H

#include <stdint.h>

/*
 * Logger control public header
 *
 * - Exposes display/encoder helpers used by main.c
 * - Provides access to shared globals (temperature/pressure/humidity/light)
 * - Declares rtc_time_t and extern g_time (hours/minutes/seconds)
 *
 * NOTE: This header assumes you have a DS1302 driver available (ds1302.h)
 *       The loggerControl.c uses ds1302_read_time() to update g_time.
 */

#include "ds1302.h"   /* DS1302 time type (ds1302_time_t) and API */

/* Shared global sensor values (defined in main.c) */
extern volatile float g_T;
extern volatile float g_P;
extern volatile float g_H;
extern volatile uint16_t g_Light;

/* Unified small RTC type used by the application (hours/minutes/seconds) */
typedef struct {
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
} rtc_time_t;

/* Shared global RTC time (defined in main.c) */
extern volatile rtc_time_t g_time;

/* Control variables (defined in loggerControl.c) */
extern volatile uint8_t lcdValue;       /* which value to display (0..3) */
extern volatile uint8_t flag_update_lcd;/* set to request LCD redraw */

/* Public API */

/* Initialize LCD and show intro */
void logger_display_init(void);

/* Draw current screen (reads g_time and g_* globals) */
void logger_display_draw(void);

/* Initialize encoder pins and any local state */
void logger_encoder_init(void);

/* Poll encoder — call frequently from main loop */
void logger_encoder_poll(void);

/* Read time from DS1302 and store into g_time (hh/mm/ss) */
void logger_rtc_read_time(void);

#endif /* LOGGER_CONTROL_H */
