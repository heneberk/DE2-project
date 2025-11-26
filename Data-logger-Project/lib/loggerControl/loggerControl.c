#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#include "loggerControl.h"
#include "lcd_i2c.h" /* Updated to use your I2C LCD library */
#include "twi.h"
#include "ds1302.h"
#include "sdlog.h"   /* Access to sd_logging variable */

/* Encoder Pins */
#define ENC_SW   PC0
#define ENC_DT   PC1
#define ENC_CLK  PC2

#define ENC_PORT_REG  PORTB
#define ENC_DDR_REG   DDRB
#define ENC_PIN_REG   PINB

/* Local encoder state */
static uint8_t lastStateCLK = 0;

/* Global variables for display control */
/* 0=Temp, 1=Pressure, 2=Humidity, 3=Light */
/* Default is 0 (Temperature) */
volatile uint8_t lcdValue = 0;      
volatile uint8_t flag_update_lcd = 0; // 1 = Please redraw display

/* Helper function for BCD conversion from RTC */
static uint8_t bcd2dec(uint8_t v) { return ((v>>4)*10 + (v & 0x0F)); }

/* Display initialization and intro screen */
void logger_display_init(void)
{
    // Using lcd_i2c functions
    lcd_i2c_init(); 
    lcd_i2c_clrscr();
    
    lcd_i2c_gotoxy(0,0);
    lcd_i2c_puts("  DATA LOGGER  ");
    lcd_i2c_gotoxy(0,1);
    lcd_i2c_puts("   VUT FEKT    ");
    // Small delay to show intro
}

/* Encoder Initialization */
void logger_encoder_init(void)
{
    // Set pins as inputs (0)
    ENC_DDR_REG  &= ~((1 << ENC_CLK) | (1 << ENC_DT) | (1 << ENC_SW));
    // Enable internal pull-up resistors (1)
    ENC_PORT_REG |=  ((1 << ENC_CLK) | (1 << ENC_DT) | (1 << ENC_SW));

    // Read initial state to prevent false trigger on startup
    // We read it multiple times or just once to be sure
    lastStateCLK = (ENC_PIN_REG & (1 << ENC_CLK)) ? 1 : 0;
}

/* Encoder Polling - Call frequently in main loop */
void logger_encoder_poll(void)
{
    uint8_t currentStateCLK = (ENC_PIN_REG & (1 << ENC_CLK)) ? 1 : 0;

    // Detect CLK state change (rotation)
    // We only react to the Falling Edge (1 -> 0) or Rising Edge (0 -> 1)
    // Typically for KY-040, checking when CLK goes LOW is reliable.
    if (currentStateCLK != lastStateCLK && currentStateCLK == 0) {
        // If CLK went LOW, check DT pin
        uint8_t dt = (ENC_PIN_REG & (1 << ENC_DT)) ? 1 : 0;
        
        if (dt != currentStateCLK) {
            // Direction CW (Clockwise) -> 0, 1, 2, 3, 0...
            lcdValue++;
            if (lcdValue > 3) lcdValue = 0;
        }
        else {
            // Direction CCW (Counter-Clockwise) -> 0, 3, 2, 1, 0...
            if (lcdValue == 0) lcdValue = 3;
            else lcdValue--;
        }
        
        // Important: Tell main.c we want to redraw display
        flag_update_lcd = 1; 
    }
    lastStateCLK = currentStateCLK;

    /* Button detection (SW) - with simple debounce */
    if ((ENC_PIN_REG & (1 << ENC_SW)) == 0) {
        // Button pressed (Active Low)
        flag_sd_toggle = 1; 
    }
}

/* Read time from RTC DS3231 (Original I2C Implementation) */
#define RTC_ADR     0x68
#define RTC_SEC_MEM 0x00

void logger_rtc_read_time(void)
{
    uint8_t buf[3];
    // Read 3 bytes from address 0x00 (seconds, minutes, hours)
    twi_readfrom_mem_into(RTC_ADR, RTC_SEC_MEM, buf, 3);

    uint8_t sec  = bcd2dec(buf[0] & 0x7F);
    uint8_t min  = bcd2dec(buf[1]);
    uint8_t hour = bcd2dec(buf[2] & 0x3F); // Mask 12/24h bit

    // Atomic write to global structure
    uint8_t sreg = SREG; cli();
    g_time.hh = hour;
    g_time.mm = min;
    g_time.ss = sec;
    SREG = sreg;
}

/* Draw data on LCD */
void logger_display_draw(void)
{
    // 1. Clear flag (request handled)
    flag_update_lcd = 0;

    // --- Line 1: Header + Time ---
    lcd_i2c_gotoxy(0,0);
    
    // SD recording indicator (*)
    char sd_icon = sd_logging ? '*' : ' '; 
    
    // Time formatting
    char timeStr[9];
    snprintf(timeStr, 9, "%02d:%02d:%02d", g_time.hh, g_time.mm, g_time.ss);

    // Display based on selection
    switch (lcdValue) {
        case 0: lcd_i2c_puts("TEMP   "); break;
        case 1: lcd_i2c_puts("PRESS  "); break;
        case 2: lcd_i2c_puts("HUMID  "); break;
        case 3: lcd_i2c_puts("LIGHT  "); break;
    }
    
    // Print SD indicator and time on the rest of the line
    lcd_i2c_putc(sd_icon);
    lcd_i2c_puts(timeStr);

    // --- Line 2: Value ---
    lcd_i2c_gotoxy(0,1);
    
    // Clear line with spaces to remove artifacts from previous longer strings
    // lcd_i2c_puts("                ");
    // lcd_i2c_gotoxy(0,1);

    char valStr[16];
    
    switch (lcdValue)
    {
        case 0: // Temperature
            dtostrf(g_T, 6, 1, valStr); // float format
            lcd_i2c_puts(valStr); 
            lcd_i2c_puts(" \xDF""C   "); // Added spaces to clear line
            break;

        case 1: // Pressure
            dtostrf(g_P, 7, 1, valStr);
            lcd_i2c_puts(valStr); 
            lcd_i2c_puts(" hPa  ");
            break;

        case 2: // Humidity
            dtostrf(g_H, 6, 1, valStr);
            lcd_i2c_puts(valStr); 
            lcd_i2c_puts(" %    ");
            break;

        case 3: // Light
            sprintf(valStr, "%u", g_Light);
            lcd_i2c_puts(valStr);
            lcd_i2c_puts(" %      "); // More spaces for shorter number
            break;

        default:
            lcd_i2c_puts("Error           ");
            break;
    }
}