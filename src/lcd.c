/*
 *  CAB202 Graphics Library (cab202_graphics)
 *	lcd.c
 *
 *	last modified Luis Mejias, 21/04/2021 12:34:56 AM
 *
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "lcd.h"
#include "ascii_font.h"
#include "macros.h"

/*
 * Function implementations
 */
void lcd_init(uint8_t contrast) {
	// Set up the pins connected to the LCD as outputs
	SET_OUTPUT(DDRD, SCEPIN);
	SET_OUTPUT(DDRD, RSTPIN);
	SET_OUTPUT(DDRD, DCPIN);
	SET_OUTPUT(DDRD, DINPIN);
	SET_OUTPUT(DDRD, SCKPIN);

	CLEAR_BIT(PORTD, RSTPIN);
	SET_BIT(PORTD, SCEPIN);
	SET_BIT(PORTD, RSTPIN);

	lcd_write(LCD_C, 0x21); // Enable LCD extended command set
	lcd_write(LCD_C, 0x80 | contrast ); // Set LCD Vop (Contrast)
	lcd_write(LCD_C, 0x04);
	lcd_write(LCD_C, 0x13); // LCD bias mode 1:48

	lcd_write(LCD_C, 0x0C); // LCD in normal mode.
  lcd_write(LCD_C, 0x20); // Enable LCD basic command set
	lcd_write(LCD_C, 0x0C);

	lcd_write(LCD_C, 0x40); // Reset row to 0
	lcd_write(LCD_C, 0x80); // Reset column to 0
}

void lcd_write(uint8_t dc, uint8_t data) {
	// Set the DC pin based on the parameter 'dc' (Hint: use the WRITE_BIT macro)
	WRITE_BIT(PORTD,DCPIN,dc);

	// Pull the SCE/SS pin low to signal the LCD we have data
	CLEAR_BIT(PORTD,SCEPIN);

	// Write the byte of data using "bit bashing"
	for(int i = 7; i >= 0; i--) {
		CLEAR_BIT(PORTD, SCKPIN) ;
		if((data>>i) & (1 == 1)) {
			SET_BIT(PORTD, DINPIN);
		} else {
			CLEAR_BIT(PORTD, DINPIN);
		}
		SET_BIT(PORTD, SCKPIN);
	}

	// Pull SCE/SS high to signal the LCD we are done
	SET_BIT(PORTD, SCEPIN);
}

void lcd_clear(void) {
	// For each of the bytes on the screen, write an empty byte
	// We don't need to start from the start: bonus question - why not?
	for (int i = 0; i < LCD_X * LCD_Y / 8; i++) {
		lcd_write(LCD_D, 0x00);
	}
}

void lcd_position(uint8_t x, uint8_t y) {
	lcd_write(LCD_C, (0x40 | y )); // Reset row to 0
	lcd_write(LCD_C, (0x80 | x )); // Reset column to 0
}
