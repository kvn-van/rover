/*
 *  CAB202 Teensy Library (cab202_teensy)
 *	lcd.h
 *
 *	Michael, 32/13/2015 12:34:56 AM
 *  Modified: B.Talbot, April 2016
*  Last modified Luis Mejias, 21/04/2021 12:34:56 AM
 *  Queensland University of Technology
 */
#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>

// What pins did we connect D/C and RST to
#define DCPIN		5   // PORTD
#define RSTPIN		6   // PORTD

// What pins are the SPI lines on
#define DINPIN		4   // PORTD
#define SCKPIN		3   // PORTD
#define SCEPIN		7   // PORTD

// LCD Command and Data
#define LCD_C		0
#define LCD_D		1

// LCD Contrast levels, you may have to change these for your display
#define LCD_LOW_CONTRAST		0x2F
#define LCD_DEFAULT_CONTRAST	0x3F
#define LCD_HIGH_CONTRAST		0x4F

// Dimensions of the LCD Screen
#define LCD_X		84
#define LCD_Y		48


#ifdef __cplusplus
extern "C" {
#endif
// Functions for interfacing with the LCD hardware
void lcd_init(uint8_t contrast);
void lcd_write(uint8_t dc, uint8_t data);
void lcd_clear(void);
void lcd_position(uint8_t x, uint8_t y);

#ifdef __cplusplus
}
#endif

#endif /* LCD_H_ */
