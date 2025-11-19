/**
 * @file main.c
 * @brief Measuring ambient light using a photoresistor and sending data via UART.
 */

#include <avr/io.h>
#include <util/delay.h>
#include "LightSensor.h"

// --------------------------------------
// UART (Serial Output)
// --------------------------------------

/**
 * @brief Initializes of UART communication.
 */
static void uart_init(void)
{
    uint16_t ubrr = 103; // 9600 baud @ 16 MHz

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    UCSR0B = (1 << TXEN0);                   // enable TX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data
}

/**
 * @brief Sends a single character via UART.
 * @param c Character to send.
 */
static void uart_send_char(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

/**
 * @brief Sends a null-terminated string via UART.
 * @param s Pointer to the string.
 */
static void uart_send_string(const char* s)
{
    while (*s)
        uart_send_char(*s++);
}

/**
 * @brief Sends an unsigned 16-bit integer as text.
 * @param val Value to transmit.
 */
static void uart_send_uint(uint16_t val)
{
    char buf[6];
    uint8_t i = 0;

    if (val == 0)
    {
        uart_send_char('0');
        return;
    }

    while (val > 0 && i < sizeof(buf))
    {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }

    while (i--)
        uart_send_char(buf[i]);
}

// --------------------------------------
// MAIN PROGRAM
// --------------------------------------

/**
 * @brief Main program loop.
 *
 * Initializes UART and the light sensor, applies calibration values,
 * and repeatedly measures light levels once per second.
 *
 * @return int Never returns (via infinite loop).
 */
int main(void)
{

  // CONFIGURATION PARAMETERS:

    uart_init();
    lightSensor_init(3);  ///< Initialize ADC on channel ADC3 (Arduino pin A3)

    /**
     * @brief Light sensor calibration.
     * @details First value = reading in darkness, second value = reading under maximum light.
     */
    lightSensor_setCalibration(10, 750);

  //________________________

    while (1)
    {
        uint16_t raw  = lightSensor_readRaw();          ///< Raw ADC reading
        uint16_t cal  = lightSensor_readCalibrated();   ///< Calibrated value in %

        uart_send_string("Raw: ");
        uart_send_uint(raw);

        uart_send_string(" | Brightness: ");
        uart_send_uint(cal);

        uart_send_string("%\r\n");

        _delay_ms(1000);  ///< Measurement interval = 1 second
    }
}
