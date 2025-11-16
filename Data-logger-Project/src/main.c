#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <uart.h>      // Fleury UART
#include <twi.h>       // Fryza TWI (I2C)
#include <gpio.h>      // GPIO library for AVR-GCC
#include "timer.h"     // Fryza Timer utilities (1ms overflow)
#include "bme280.h"    // Our BME280 driver
#include <lcd.h>       // Peter Fleury's LCD library

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// ----------------------------------------------------
// Global software time (milliseconds)
// ----------------------------------------------------
volatile uint32_t g_millis = 0;

ISR(TIMER0_OVF_vect)
{
    g_millis++;    // called every ~1 ms
}

// Thread-safe access to g_millis
static uint32_t millis(void)
{
    uint32_t t;
    uint8_t sreg = SREG;
    cli();
    t = g_millis;
    SREG = sreg;
    return t;
}

// Simple println wrapper
static void uart_println(const char *s)
{
    uart_puts(s);
    uart_puts("\r\n");
}

/* -------------------- DISPLAY + ENCODER -------------------- */
/* Encoder KY-040 pin numbers */
#define ENC_SW   PC0   // push button (active low)
#define ENC_DT   PC1
#define ENC_CLK  PC2

/* Encoder port registers (PORTB) */
#define ENC_PORT_REG  PORTB
#define ENC_DDR_REG   DDRB
#define ENC_PIN_REG   PINB

/* Display selection: 0 = Temperature, 1 = Pressure, 2 = Humidity */
volatile uint8_t lcdValue = 0;          // which value to show
volatile uint8_t flag_update_lcd = 0;   // set by timer2 ISR or encoder to request redraw

/* Shared latest sensor values (updated after successful read) */
volatile float g_T = 0.0f, g_P = 0.0f, g_H = 0.0f;

/* Encoder state */
static uint8_t lastStateCLK = 0;
static uint32_t lastEncButtonPress = 0;

/* Forward declarations for display/encoder helpers */
static void display_init_minimal(void);
static void encoder_init(void);
static void display_draw_now(void);
static void timer2_init_for_display_1s(void);
static void encoder_check_poll(void);

/* --- Initialize LCD (simple, minimal) --- */
static void display_init_minimal(void)
{
    lcd_init(LCD_DISP_ON);

    /* initial message */
    lcd_clrscr();
    lcd_gotoxy(0,0);
    lcd_puts("Env. logger");
    lcd_gotoxy(0,1);
    lcd_puts("Starting...");
}

/* --- Initialize encoder pins as inputs with pull-ups --- */
static void encoder_init(void)
{
    /* Configure inputs (clear DDR bits) */
    ENC_DDR_REG &= ~((1 << ENC_CLK) | (1 << ENC_DT) | (1 << ENC_SW));
    /* Enable internal pull-ups */
    ENC_PORT_REG |= ((1 << ENC_CLK) | (1 << ENC_DT) | (1 << ENC_SW));

    /* Read initial CLK state */
    lastStateCLK = (ENC_PIN_REG & (1 << ENC_CLK)) ? 1 : 0;
}

/* --- Timer2 ISR will set flag_update_lcd every ~1 s (via multiple 16 ms overflows) --- */
ISR(TIMER2_OVF_vect)
{
    static uint8_t n_ovfs = 0;
    n_ovfs++;
    /* 62 * 16 ms = 992 ms ~ 1 s */
    if (n_ovfs >= 62)
    {
        n_ovfs = 0;
        flag_update_lcd = 1;
    }
}

/* --- Setup Timer2 for ~16 ms overflow and enable its interrupt --- */
static void timer2_init_for_display_1s(void)
{
    tim2_ovf_16ms();      /* from timer.h */
    tim2_ovf_enable();    /* enable TIMER2 overflow interrupt */
}

/* --- Poll encoder (call often from main loop) --- */
static void encoder_check_poll(void)
{
    uint8_t currentStateCLK = (ENC_PIN_REG & (1 << ENC_CLK)) ? 1 : 0;

    if (currentStateCLK != lastStateCLK)
    {
        /* Determine direction by reading DT */
        uint8_t dt = (ENC_PIN_REG & (1 << ENC_DT)) ? 1 : 0;
        if (dt != currentStateCLK)
        {
            /* counter-clockwise */
            if (lcdValue == 0) lcdValue = 2;
            else lcdValue--;
        }
        else
        {
            /* clockwise */
            lcdValue++;
            if (lcdValue > 2) lcdValue = 0;
        }
        /* request redraw */
        flag_update_lcd = 1;
    }
    lastStateCLK = currentStateCLK;

    /* Button (active LOW) - simple debounced check */
    if ( (ENC_PIN_REG & (1 << ENC_SW)) == 0 )
    {
        uint32_t now = millis();
        if (now - lastEncButtonPress > 50)
        {
            /* placeholder: button pressed */
            /* we don't implement action yet, but request redraw to be safe */
            flag_update_lcd = 1;
            lastEncButtonPress = now;
        }
    }
}

/* --- Render currently selected value to LCD (minimal, efficient) --- */
static void display_draw_now(void)
{
    char buf[16];
    uint8_t i;

    /* Write label on first row (overwrite previous content) */
    lcd_gotoxy(0,0);
    switch (lcdValue)
    {
        case 0: lcd_puts("Temperature:   "); break;  /* pad to clear rest */
        case 1: lcd_puts("Pressure:      "); break;
        case 2: lcd_puts("Humidity:      "); break;
        default: lcd_puts("Unknown        "); break;
    }

    /* Clear second row (16 chars) then write value */
    lcd_gotoxy(0,1);
    for (i = 0; i < 16; i++) lcd_putc(' ');
    lcd_gotoxy(0,1);

    switch (lcdValue)
    {
        case 0: /* Temperature: format " +xx.x C" or "-xx.x C" */
            dtostrf((double)g_T, 5, 1, buf);    /* width 5, 1 decimal */
            lcd_puts(buf);
            lcd_puts(" C");
            break;

        case 1: /* Pressure: format "xxxx.x hPa" */
            dtostrf((double)g_P, 6, 1, buf);    /* width 6, 1 decimal */
            lcd_puts(buf);
            lcd_puts(" hPa");
            break;

        case 2: /* Humidity: "xx.x %" */
            dtostrf((double)g_H, 5, 1, buf);
            lcd_puts(buf);
            lcd_puts(" %");
            break;

        default:
            lcd_puts("N/A");
            break;
    }
}
/* -------------------- END DISPLAY + ENCODER -------------------- */


/* ----------------------------------------------------
   Main application (BME280 logger)
   ---------------------------------------------------- */
int main(void)
{
    // UART 9600 baud (works reliably with ATmega328P/16 MHz)
    uart_init(UART_BAUD_SELECT(9600, F_CPU));

    // Initialize I2C/TWI
    twi_init();

    // Timer0 overflow every ~1ms
    tim0_ovf_1ms();
    tim0_ovf_enable();

    /* Initialize display and encoder and Timer2 for LCD updates.
       It's OK to init peripherals before enabling global interrupts. */
    display_init_minimal();
    encoder_init();
    timer2_init_for_display_1s();

    sei();  // enable interrupts

    uart_println("BME280 data logger starting...");

    // Check if sensor is connected
    if (twi_test_address(BME280_I2C_ADDR) != 0)
        uart_println("ERROR: BME280 not found!");

    bme280_init();
    uart_println("BME280 initialized.");

    float T, P, H;
    char bufT[16], bufP[16], bufH[16];
    char line[64];

    uint32_t last = millis();

    /* Force an initial display update with default values */
    flag_update_lcd = 1;

    while (1)
    {
        /* Poll encoder frequently so UI feels responsive */
        encoder_check_poll();

        /* Update every 1000 ms: read sensor, print to UART, update globals */
        if (millis() - last >= 1000)
        {
            last += 1000;

            // Read sensor
            bme280_read(&T, &P, &H);

            // Update shared values for display (atomic enough for floats here;
            // if you need strict atomicity, protect with cli()/SREG as needed)
            g_T = T;
            g_P = P;
            g_H = H;

            // Convert float -> string (AVR does not support float printf)
            dtostrf(T, 6, 2, bufT);
            dtostrf(P, 7, 2, bufP);
            dtostrf(H, 6, 2, bufH);

            // Format final output line
            snprintf(line, sizeof(line),
                     "T=%s C, P=%s hPa, H=%s %%", bufT, bufP, bufH);

            uart_println(line);

            /* Request display redraw immediately after new measurement */
            flag_update_lcd = 1;
        }

        /* If LCD needs update (timer or encoder), redraw now */
        if (flag_update_lcd)
        {
            display_draw_now();
            flag_update_lcd = 0;
        }

        /* Minimal idle - do not block (no delays) */
    }

    return 0;
}
