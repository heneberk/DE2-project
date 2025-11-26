/***********************************************************************
 * Main file of the Data Logger project
 * * Functions:
 * - Peripherals initialization (UART, I2C, SD card, Sensors, Timer)
 * - Main loop (Super-loop)
 * - Task scheduling using system time (millis)
 * * NON-BLOCKING DESIGN:
 * The main loop does not use _delay_ms(). Instead, elapsed time is
 * checked using Timer 0. This ensures immediate response to
 * encoder rotation.
 **********************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> // Used only for short delays during initialization
#include <stdio.h>
#include <stdlib.h>

/* Custom libraries */
#include "uart.h"       
#include "twi.h"        
#include "bme280.h"
#include "LightSensor.h"
#include "loggerControl.h"
#include "sdlog.h"
#include "lcd_i2c.h"    /* Using the uploaded I2C LCD library */
#include "ds1302.h"   /* add at top of main.c includes */
#include "timer.h"

/* Sampling period definition (e.g., 1000 ms = 1 second) */
#define SAMPLE_PERIOD_MS 1000UL

/* --- Global variables for data sharing --- */
/* These variables are externally visible in loggerControl.c */
volatile float g_T = 0.0f;
volatile float g_P = 0.0f;
volatile float g_H = 0.0f;
volatile uint16_t g_Light = 0; /* Light intensity */
volatile rtc_time_t g_time = {0, 0, 0};

/* SD card flag (controlled by encoder) */
extern volatile uint8_t flag_sd_toggle;

/* --- SYSTEM TIME (Millis) --- */
/* Millisecond counter since program start */
volatile uint32_t g_millis = 0;

/* Timer 0 interrupt every 1 ms */
ISR(TIMER0_OVF_vect) {
    g_millis++;
}

/* Function to get current time (atomically) */
static uint32_t millis(void) { 
    uint32_t t; 
    uint8_t sreg = SREG; 
    cli(); // Disable interrupts for atomic reading of 32-bit number
    t = g_millis; 
    SREG = sreg; // Restore interrupts
    return t; 
}

/* Timer 0 initialization for system time (1 ms) */
void timer0_init_system_tick(void) {
    // Timer 0 setup: Normal mode, Prescaler 64
    // At 16 MHz: 16 000 000 / 64 = 250 000 ticks/s
    // 250 ticks -> 1 ms (approximately)
    // Using timer.h library macros:
    tim0_ovf_1ms(); 
    tim0_ovf_enable();
}

/* Helper function for I2C scan (optional, for debug) */
void i2c_scan(void) {
    uart_puts("Scanning I2C bus...\r\n");
    for(uint8_t addr = 1; addr < 127; addr++) {
        if(twi_test_address(addr) == 0) {
            char buf[32];
            sprintf(buf, "Device found: 0x%02X\r\n", addr);
            uart_puts(buf);
        }
    }
    uart_puts("Scan done.\r\n");
}

/* === Main function === */
int main(void) {
    // -- 1. Hardware Initialization --
    
    // UART for debug (9600 baud)
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    
    // I2C bus
    twi_init();

    // Inicializace RTC
    ds1302_init();
    uart_puts("RTC (DS1302) init done.\r\n");

    /*
    // === RUN THIS ONCE TO SET TIME ===
    ds1302_time_t t_setup;
    t_setup.sec = 0;      // BIN values
    t_setup.min = 15;
    t_setup.hour = 14;
    t_setup.date = 1;     // dummy (unused by logger)
    t_setup.month = 1;    
    t_setup.day = 1;
    t_setup.year = 24;    // 2024
    ds1302_set_time(&t_setup);
    uart_puts("Time set!\r\n");
    // =======================================
    */

    // První přečtení času z RTC
    ds1302_time_t t_now;
    ds1302_read_time(&t_now);

    // BCD → BIN
    g_time.hh = ((t_now.hour >> 4) * 10) + (t_now.hour & 0x0F);
    g_time.mm = ((t_now.min  >> 4) * 10) + (t_now.min  & 0x0F);
    g_time.ss = ((t_now.sec  >> 4) * 10) + (t_now.sec  & 0x0F);

    /* ensure default display is Temperature at startup */
    lcdValue = 0;
    flag_update_lcd = 1;
    logger_display_draw();
    
    // SD Card
    sd_log_init();

    // Timer 0 for millis()
    timer0_init_system_tick();
    
    // Enable global interrupts
    sei();

    uart_puts("--- System Start ---\r\n");
    i2c_scan();

    // Sensor Initialization
    uart_puts("Init BME280...\r\n");
    bme280_init();
    
    uart_puts("Init Light Sensor...\r\n");
    lightSensor_init(3); // Analog pin A3
    lightSensor_setCalibration(10, 750);

    // Control Initialization (LCD + Encoder)
    logger_display_init();
    logger_encoder_init();

    // Timing variables
    uint32_t last_sample_time = 0;
    uint16_t calLight;
    char line_buffer[80];
    float temp, press, hum;

    // Initial time read (dummy read or RTC if present)
    ds1302_time_t t;
    ds1302_read_time(&t);

    // decode BCD → BIN
    g_time.hh = ((t.hour >> 4) * 10) + (t.hour & 0x0F);
    g_time.mm = ((t.min  >> 4) * 10) + (t.min  & 0x0F);
    g_time.ss = ((t.sec  >> 4) * 10) + (t.sec  & 0x0F);

    /* === Main Infinite Loop === */
    while(1) {
        uint32_t current_time = millis();

        // -- TASK 1: Encoder Handling (Must be called as often as possible) --
        logger_encoder_poll();

        // -- TASK 2: LCD Drawing --
        // If encoder changed state (flag_update_lcd), redraw display IMMEDIATELY.
        if (flag_update_lcd) {
            logger_display_draw(); 
        }

        // -- TASK 3: Periodic measurement (every 1000 ms) --
        if (current_time - last_sample_time >= SAMPLE_PERIOD_MS) {
            last_sample_time = current_time;

            /* A) Read sensors */
            bme280_read(&temp, &press, &hum);
            calLight = lightSensor_readCalibrated();
            
            /* B) Update global shared variables (atomically) */
            uint8_t sreg = SREG; cli();
            g_T = temp;
            g_P = press;
            g_H = hum;
            g_Light = calLight;
            SREG = sreg;

            /* C) Debug output to UART */
            char bufT[10], bufP[10], bufH[10];
            dtostrf(temp, 4, 1, bufT);
            dtostrf(press, 6, 1, bufP);
            dtostrf(hum, 4, 1, bufH);
            
            sprintf(line_buffer, "T: %s C, P: %s hPa, H: %s %%, L: %u %%\r\n", bufT, bufP, bufH, calLight);
            uart_puts(line_buffer);
            
            /* E) Read/Update time (DS1302) */
        {
            ds1302_time_t t;
            ds1302_read_time(&t);

            /* BCD -> BIN conversion */
            uint8_t hh = ((t.hour >> 4) * 10) + (t.hour & 0x0F);
            uint8_t mm = ((t.min  >> 4) * 10) + (t.min  & 0x0F);
            uint8_t ss = ((t.sec  >> 4) * 10) + (t.sec  & 0x0F);

            /* atomic update of global time */
            uint8_t sreg2 = SREG; cli();
            g_time.hh = hh;
            g_time.mm = mm;
            g_time.ss = ss;
            SREG = sreg2;
        }

            /* F) SD card logging (if active) */
            static uint8_t last_logged_sec = 255;
            if(sd_logging && g_time.ss != last_logged_sec) {
                sd_log_append_line(g_T, g_P, g_H, g_Light);
                last_logged_sec = g_time.ss;
            }

            /* G) Request LCD redraw with new data */
            flag_update_lcd = 1;
        }

        // -- TASK 4: SD Card Control (encoder button) --
        if(flag_sd_toggle) {
            flag_sd_toggle = 0; // Reset request
            if(!sd_logging) {
                uart_puts("SD: Start logging\r\n");
                sd_log_start();
            } else {
                uart_puts("SD: Stop logging\r\n");
                sd_log_stop();
            }
            // Update display (recording icon etc.)
            flag_update_lcd = 1;
        }
    }

    return 0; // Never reached
}