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
#define RX_BUFFER_SIZE 10
#define HIGH 150
#define LOW 80


// uart definitions
unsigned char rx_buf;

static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_buffer_head;
static volatile uint8_t rx_buffer_tail;

// ADC outputs
uint8_t pot0;
uint8_t pot1;

unsigned char power = 0;

void pwm_init(){
	// TCCR0A -> Fast PWM Mode, CLEAR 0C0x on Compare Match when up-counting
	TCCR0A |= (1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);
	TCCR0A &= ~((1<<COM0A0)|(1<<COM0B0));

	// 256 clock prescalar
	TCCR0B |= (1<<CS02);
	TCCR0B &= ~((1<<WGM02)|(1<<CS01)|(1<<CS00));
	
	// motor A uses OC1A
	// motor B uses OC1B
	
	//Motor 2
//	DDRB |= (1<<0);
//	DDRB |= (1<<7);
	//Motor 1
//	DDRE |= (1<<6);
//	DDRD |= (1<<0);
	
//	PORTB |= (1<<0); 
//	PORTE |= (1<<6);
}

void setMotorSpeeds(double motorA, double motorB) {
	OCR0A = 255 * motorA/100; 
	OCR0B = 255 * motorB/100;
}

void uart_init(){
	cli();
	UBRR0 = MYF_CPU / 16 / 9600 - 1;

	UCSR0A = 0;
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);	// Enable receive and transmit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);				// Set the character size to 8 bits

	sei();
}

// Receive a byte
uint8_t uart_getchar(void) {
	uint8_t c, i;

	while ( rx_buffer_head == rx_buffer_tail ); // wait for character
	i = rx_buffer_tail + 1;
	if ( i >= RX_BUFFER_SIZE ) i = 0;
	c = rx_buffer[i];
	rx_buffer_tail = i;
	return c;
}

// check to see if uart is available
uint8_t uart_available(void) {
	uint8_t head, tail;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if ( head >= tail ) return head - tail;
	return RX_BUFFER_SIZE + head - tail;
}

// Receive interrupt
ISR(USART_RX_vect) { 
	uint8_t c, i;
  
	c = UDR0;
	i = rx_buffer_head + 1;
	if ( i >= RX_BUFFER_SIZE ) i = 0;
	if ( i != rx_buffer_tail ) {
		rx_buffer[i] = c;
		rx_buffer_head = i;
	}
}

void process(){
	if(uart_available()){
		rx_buf = uart_getchar();
	}

	if(rx_buf == 'a' && power == 1){	// if 'a' is received toggle LED backlight
		PORTD ^= (1<<2);
	}
	if(rx_buf == 'b'){					// if 'b' is received, toggle power
		power ^= 1;
	}
	rx_buf = 0;							// clear buffer
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

void sensor0(){
	ADMUX = 0b01100000;			// select ADC0
	ADCSRA |= (1<<ADSC);		// start conversion
	while(ADCSRA&(1<<ADSC)){} 	// wait for conversion to complete
	pot0 = ADCH;				// store the result in pot0
}
void sensor1(){
	ADMUX = 0b01100001;			// select ADC1
	ADCSRA |= (1<<ADSC);		// start conversion
	while(ADCSRA&(1<<ADSC)){} 	// wait for conversion to complete
	pot1 = ADCH;				// store the result in pot0
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
	adc_init();
	led_init();
}

void control(){
	if(pot0 > HIGH && pot1 > HIGH){ // if both sensors get lots of light, turn off motors, execute basking sequence
		setMotorSpeeds(0, 0);
		PORTB |= (1<<0);
		draw_string(25, 0, "basking~", FG_COLOUR);
		draw_string( 25, 20, "\\  |  /", FG_COLOUR );
		draw_string(10, 30, "-- \\(^O^)/ --", FG_COLOUR);
		draw_string( 25, 40, "/  |  \\", FG_COLOUR );
	}
	else{
		PORTB &= ~(1<<0);
		if(pot0 > LOW && pot1 > LOW){ // if sensor 1 and 0 sees low light go straight
			setMotorSpeeds(40, 40);
			draw_string( 2, 0, "SEEKING SUNLIGHT", FG_COLOUR );
			draw_string( 20, 10, "STRAIGHT", FG_COLOUR );
			draw_string( 30, 30, "('_')", FG_COLOUR );
		}
		if(pot0 > LOW && pot1 < LOW){ // if sensor 0 sees light and 1 does not, turn right
			setMotorSpeeds(40, 0);
			draw_string( 2, 0, "SEEKING SUNLIGHT", FG_COLOUR );
			draw_string( 30, 10, "RIGHT", FG_COLOUR );
			draw_string( 30, 30, "('_')", FG_COLOUR );
		}
		if(pot0 < LOW && pot1 > LOW){ // if sensor 1 sees light and 0 does not, turn left
			setMotorSpeeds(0, 40);
			draw_string( 2, 0, "SEEKING SUNLIGHT", FG_COLOUR );
			draw_string( 30, 10, "LEFT", FG_COLOUR );
			draw_string( 30, 30, "('_')", FG_COLOUR );
		}
		else{
			setMotorSpeeds(20, 20);
			draw_string( 2, 0, "SEEKING SUNLIGHT", FG_COLOUR );
			draw_string( 30, 30, "('_')", FG_COLOUR );
		}
	}
}


int main(void) {
    setup();

	while(1){
		process();
		_delay_ms(10);
		sensor0();
		sensor1();
		clear_screen();
		if(power == 1){	// if power is on
			SMCR = 0; 	// turn off power saving mode
			control();
		}
		if(power == 0){
			setMotorSpeeds(0, 0);
			clear_screen();
			PORTB &= ~(1<<0);
			PORTD &= ~(1<<2);
			SMCR |= (1<<SM1)|(1<<SE); // enable sleep mode, power down mode
		}
	show_screen();
    }
} 
