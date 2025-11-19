#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart.h"       // Fleury UART
#include "twi.h"        // Fryza TWI
#include "bme280.h"
#include "LightSensor.h"
#include <stdlib.h>

#define SAMPLE_PERIOD_MS 1000UL

volatile uint32_t g_millis = 0;
ISR(TIMER0_OVF_vect) { g_millis++; }
static uint32_t millis(void) { uint32_t t; uint8_t sreg=SREG; cli(); t=g_millis; SREG=sreg; return t; }

// Print line with Fleury UART
static void uart_println(const char *s) { uart_puts(s); uart_puts("\r\n"); }

void i2c_scan(void) {
    uart_println("Scanning I2C...");
    for(uint8_t addr=1; addr<127; addr++) {
        if(twi_test_address(addr)==0) {
            char buf[32];
            snprintf(buf,sizeof(buf),"Found: 0x%02X", addr);
            uart_println(buf);
        }
    }
    uart_println("Scan done!");
}

int main(void) {
    uart_init(UART_BAUD_SELECT(9600,F_CPU));
    twi_init();

    // Timer0: 1 ms overflow
    TIMSK0 = (1<<TOIE0);
    TCCR0B = (1<<CS01) | (1<<CS00);
    sei();

    i2c_scan();

    uart_println("Initializing BME280...");
    bme280_init();
    _delay_ms(10);
    uart_println("BME280 initialized.");

    // Light Sensor
    lightSensor_init(3);
    lightSensor_setCalibration(10,750);
    uart_println("Light sensor initialized.");

    float T,P,H;
    uint16_t rawLight, calLight;
    char bufT[8], bufP[10], bufH[8], line[128];

    uint32_t last_sample=millis();

    while(1) {
        uint32_t now = millis();
        if(now - last_sample >= SAMPLE_PERIOD_MS) {
            last_sample += SAMPLE_PERIOD_MS;

            // BME280
            bme280_read(&T,&P,&H);
            dtostrf(T,6,2,bufT);
            dtostrf(P,7,2,bufP);
            dtostrf(H,6,2,bufH);
            snprintf(line,sizeof(line),"BME280 -> T=%s C, P=%s hPa, H=%s %%",bufT,bufP,bufH);
            uart_println(line);

            // Light Sensor
            rawLight = lightSensor_readRaw();
            calLight = lightSensor_readCalibrated();
            snprintf(line,sizeof(line),"Light -> Raw=%u | SVETLo=%u%%",rawLight,calLight);
            uart_println(line);
        }
    }
}
