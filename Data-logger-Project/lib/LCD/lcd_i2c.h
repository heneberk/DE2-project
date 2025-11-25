#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <avr/io.h>
#include <util/delay.h>

/**
 * @brief I2C Address of the display.
 * Common values: 0x27 (generic) or 0x3F (some clones).
 * Check your specific hardware or run an I2C scanner if unsure.
 */
#define LCD_ADDR 0x27 

/* Display dimensions */
#define LCD_COLS 16
#define LCD_ROWS 2

/**
 * @brief Initialize LCD display via I2C
 */
void lcd_i2c_init(void);

/**
 * @brief Clear the display screen
 */
void lcd_i2c_clrscr(void);

/**
 * @brief Move cursor to specific position
 * @param col Column (0-15)
 * @param row Row (0-1)
 */
void lcd_i2c_gotoxy(uint8_t col, uint8_t row);

/**
 * @brief Print one character to the display
 * @param c Character to print
 */
void lcd_i2c_putc(char c);

/**
 * @brief Print string to the display
 * @param s Pointer to the string
 */
void lcd_i2c_puts(const char* s);

#endif