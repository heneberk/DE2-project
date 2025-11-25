#include "ds1302.h"
#include <util/delay.h>

/* Příkazy DS1302 */
#define CMD_READ_CLOCK_BURST 0xBF
#define CMD_WRITE_CLOCK_BURST 0xBE
#define CMD_WRITE_ENABLE 0x8E
#define CMD_TRICKLE_CHARGER 0x90

/* Makra pro ovládání pinů (přímý přístup do registrů = nejrychlejší) */
#define CE_SET()    (DS1302_PORT |=  (1 << DS1302_CE_PIN))
#define CE_CLR()    (DS1302_PORT &= ~(1 << DS1302_CE_PIN))

#define SCLK_SET()  (DS1302_PORT |=  (1 << DS1302_SCLK_PIN))
#define SCLK_CLR()  (DS1302_PORT &= ~(1 << DS1302_SCLK_PIN))

#define IO_SET()    (DS1302_PORT |=  (1 << DS1302_IO_PIN))
#define IO_CLR()    (DS1302_PORT &= ~(1 << DS1302_IO_PIN))

#define IO_INPUT()  (DS1302_DDR &= ~(1 << DS1302_IO_PIN))
#define IO_OUTPUT() (DS1302_DDR |=  (1 << DS1302_IO_PIN))
#define IO_READ()   ((DS1302_PIN & (1 << DS1302_IO_PIN)) ? 1 : 0)

/* BCD konverze */
static uint8_t bcd2dec(uint8_t val) { return ((val / 16 * 10) + (val % 16)); }
static uint8_t dec2bcd(uint8_t val) { return ((val / 10 * 16) + (val % 10)); }

/* Odeslání bajtu (LSB first) */
static void ds1302_write_byte(uint8_t val) {
    IO_OUTPUT();
    for (uint8_t i = 0; i < 8; i++) {
        if (val & 0x01) IO_SET();
        else IO_CLR();
        
        /* Data musí být platná před náběžnou hranou hodin */
        _delay_us(1); 
        SCLK_SET();
        _delay_us(1);
        SCLK_CLR();
        
        val >>= 1;
    }
}

/* Čtení bajtu (LSB first) */
static uint8_t ds1302_read_byte(void) {
    uint8_t val = 0;
    IO_INPUT();
    
    for (uint8_t i = 0; i < 8; i++) {
        val >>= 1;
        /* Data se čtou při sestupné hraně, nebo jsou platná po náběžné */
        if (IO_READ()) val |= 0x80;
        
        SCLK_SET();
        _delay_us(1);
        SCLK_CLR();
        _delay_us(1);
    }
    return val;
}

void ds1302_init(void) {
    // Nastavení směrů: CE a SCLK výstup
    DS1302_DDR |= (1 << DS1302_CE_PIN) | (1 << DS1302_SCLK_PIN);
    
    CE_CLR();   // Chip disable
    SCLK_CLR(); // Clock low
    
    // Odstranění ochrany proti zápisu (Write Protect)
    CE_SET();
    ds1302_write_byte(CMD_WRITE_ENABLE);
    ds1302_write_byte(0x00);
    CE_CLR();
}

void ds1302_read_time(ds1302_time_t *time) {
    CE_SET();
    ds1302_write_byte(CMD_READ_CLOCK_BURST); // Přečíst vše najednou
    
    /* Pořadí: Sec, Min, Hour, Date, Month, Day, Year, WP */
    time->sec   = bcd2dec(ds1302_read_byte() & 0x7F); // Mask CH bit
    time->min   = bcd2dec(ds1302_read_byte());
    time->hour  = bcd2dec(ds1302_read_byte() & 0x3F); // 12/24h mód
    time->date  = bcd2dec(ds1302_read_byte());
    time->month = bcd2dec(ds1302_read_byte());
    time->day   = bcd2dec(ds1302_read_byte());
    time->year  = bcd2dec(ds1302_read_byte());
    
    ds1302_read_byte(); // Ignorujeme WP byte
    
    CE_CLR();
}

void ds1302_set_time(ds1302_time_t *time) {
    CE_SET();
    ds1302_write_byte(CMD_WRITE_CLOCK_BURST);
    
    ds1302_write_byte(dec2bcd(time->sec));
    ds1302_write_byte(dec2bcd(time->min));
    ds1302_write_byte(dec2bcd(time->hour));
    ds1302_write_byte(dec2bcd(time->date));
    ds1302_write_byte(dec2bcd(time->month));
    ds1302_write_byte(dec2bcd(time->day));
    ds1302_write_byte(dec2bcd(time->year));
    ds1302_write_byte(0x00); // WP
    
    CE_CLR();
}