

/**
 * @file LightSensor.c
 * @brief Implementation of light sensor.
 */

#include "LightSensor.h"
#include <avr/io.h>

// Selected ADC channel (0–7)
static uint8_t adc_pin = 0;

// Calibration boundaries
static uint16_t cal_min = 0;
static uint16_t cal_max = 1023;

/**
 * @brief Initializes the ADC for the selected analog input pin.
 *
 * Sets reference voltage to AVCC and configures ADC prescaler for stable readings.
 *
 * @param pin ADC channel number (0–5 on Arduino Uno).
 */
void lightSensor_init(uint8_t pin)
{
    adc_pin = pin & 0x07;

    // Reference voltage = AVCC, select ADC channel
    ADMUX = (1 << REFS0) | adc_pin;

    // Enable ADC, set prescaler to 128
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/**
 * @brief Reads the raw ADC value from the selected channel.
 *
 * @return 10-bit ADC value in the range 0–1023.
 */
uint16_t lightSensor_readRaw(void)
{
    // Select ADC channel
    ADMUX = (ADMUX & 0xF8) | adc_pin;

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to finish
    while (ADCSRA & (1 << ADSC));

    // Return raw ADC value
    return ADC;
}

/**
 * @brief Sets calibration values used for converting ADC readings to percentage.
 *
 * @param minValue ADC value measured in darkness.
 * @param maxValue ADC value measured at maximum brightness.
 */
void lightSensor_setCalibration(uint16_t minValue, uint16_t maxValue)
{
    cal_min = minValue;
    cal_max = maxValue;
}

/**
 * @brief Converts the raw ADC reading into a percentage based on calibration values.
 *
 * @return Light intensity from 0% (bright) to 100% (dark).
 */
uint16_t lightSensor_readCalibrated(void)
{
    uint16_t raw = lightSensor_readRaw();

    // Clamp to range
    if (raw <= cal_min) return 100;  // darkest = 0%
    if (raw >= cal_max) return 0;    // brightest = 100%

    // Linear mapping
    uint16_t pct = (raw - cal_min) * 100UL / (cal_max - cal_min);
    return 100 - pct;                // invert scale
}
