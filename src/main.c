#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#include <graphics.h>
#include <macros.h>

#include "lcd_model.h"

#define MYF_CPU (16000000UL)
#define TOLERANCE 50

uint8_t pot0;
unsigned char buffer;


void uart_init(){
	UBRR0 = MYF_CPU / 16 / 9600 - 1;

	UCSR0A = 0;
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);	// Enable receive and transmit
	UCSR0C |= (3<<UCSZ00);				// Set the character size to 8 bits
}

unsigned char uart_receive(){
	while(!(UCSR0A & (1<< RXC0)));
	return UDR0;
}

void process(){
	buffer = uart_receive();
	if(buffer == 'a'){
		PORTD ^= (1<<2);
	}
}

void led_init(){
	DDRB |= (1<<0);	// set PB1 for output
	DDRD|= (1<<2);	// set PD2 for output
}

void adc_init(){
	ADMUX |= (1<<REFS0)|(1<<ADLAR); 			// AVcc reference, left adjusted
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);	// enable adc, clock prescalar of 128
	ADCSRB = 0;									// free running mode
}

void adc_read(){
	ADCSRA |= (1<<ADSC);		// start conversion
	while(ADCSRA&(1<<ADSC)){} 	// wait for conversion to complete
	pot0 = ADCH;				// store the result in pot0
}

void new_lcd_init(uint8_t contrast) {
    // Set up the pins connected to the LCD as outputs
    SET_OUTPUT(DDRD, SCEPIN); // Chip select -- when low, tells LCD we're sending data
    SET_OUTPUT(DDRD, RSTPIN); // Chip Reset
    SET_OUTPUT(DDRD, DCPIN);  // Data / Command selector
    SET_OUTPUT(DDRD, DINPIN); // Data input to LCD
    SET_OUTPUT(DDRD, SCKPIN); // Clock input to LCD

    CLEAR_BIT(PORTD, RSTPIN); // Reset LCD
    SET_BIT(PORTD, SCEPIN);   // Tell LCD we're not sending data.
    SET_BIT(PORTD, RSTPIN);   // Stop resetting LCD

    LCD_CMD(lcd_set_function, lcd_instr_extended);
    LCD_CMD(lcd_set_contrast, contrast);
    LCD_CMD(lcd_set_temp_coeff, 0);
    LCD_CMD(lcd_set_bias, 3);

    LCD_CMD(lcd_set_function, lcd_instr_basic);
    LCD_CMD(lcd_set_display_mode, lcd_display_normal);
    LCD_CMD(lcd_set_x_addr, 0);
    LCD_CMD(lcd_set_y_addr, 0);
}

void setup(void) {
    new_lcd_init(LCD_DEFAULT_CONTRAST);
    clear_screen();
	uart_init();
}


int main(void) {
    setup();
	adc_init();
	led_init();

	while(1){
		process();
		adc_read();
		clear_screen();

		if(pot0 < TOLERANCE){ 	// if the sensor sees light
			PORTB |= (1<<0);
			draw_string( 0, 0, "IN PURSUIT OF UV", FG_COLOUR );
			draw_string( 30, 30, "('_')", FG_COLOUR );
			
			// write a function for the robot to 
			// if robot has been searching for 20 seconds {turn off}
		}

		else{
			PORTB &= ~(1<<0);
			draw_string(25, 0, "basking~", FG_COLOUR);
			draw_string( 25, 20, "\\  |  /", FG_COLOUR );
			draw_string(10, 30, "-- \\(^O^)/ --", FG_COLOUR);
			draw_string( 25, 40, "/  |  \\", FG_COLOUR );
		}
		show_screen();
    }
} 
