#ifndef LOGGER_CONTROL_H
#define LOGGER_CONTROL_H

#include <stdint.h>

/* Time structure */
typedef struct {
    uint8_t h;
    uint8_t m;
    uint8_t s;
} rtc_time_t;

/* Shared global variables (defined in main.c) */
/* Using 'extern' to avoid double allocation */
extern volatile float g_T;
extern volatile float g_P;
extern volatile float g_H;
extern volatile uint16_t g_Light; /* Added missing declaration for Light */
extern volatile rtc_time_t g_time;

/* Control variables (defined in loggerControl.c) */
extern volatile uint8_t lcdValue;
extern volatile uint8_t flag_update_lcd;

/* Public Functions */

// LCD initialization and intro print
void logger_display_init(void);

// Draw current state on LCD (call from main loop)
void logger_display_draw(void);

// Encoder pins initialization
void logger_encoder_init(void);

// Check encoder state (call frequently)
void logger_encoder_poll(void);

// Read time from RTC and store in g_time
void logger_rtc_read_time(void);

#endif