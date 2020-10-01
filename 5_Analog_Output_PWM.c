/*
TITLE	: Analog Output using PWM
DATE	: 2019/11/12
AUTHOR	: e-Yantra Team

AIM: To decrease the brightness of Red Led and simultaneously increase the brightness of Green Led,
	 then to decrease the brightness of Green Led and simultaneously increase the brightness of Blue Led,
	 then to decrease the brightness of Blue Led and simultaneously increase the brightness of Red Led
	 and repeat this cyclically.
*/

#define F_CPU 16000000UL		// Define Crystal Frequency of eYFi-Mega Board

#include <avr/io.h>				// Standard AVR IO Library
#include <util/delay.h>			// Standard AVR Delay Library
#include <avr/interrupt.h>		// Standard AVR Interrupt Library

#define PIN_LED_RED		PH3
#define PIN_LED_GREEN	PH5
#define PIN_LED_BLUE	PH4

void led_init(void){
	DDRH    |= (1 << PIN_LED_RED) | (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE);    
	PORTH   |= (1 << PIN_LED_RED) | (1 << PIN_LED_GREEN) | (1 << PIN_LED_BLUE);    
}

void led_redOn(void){
	PORTH &= ~(1 << PIN_LED_RED);
}

void led_redOff(void){
	PORTH |= (1 << PIN_LED_RED);
}

void led_greenOn(void){
	PORTH &= ~(1 << PIN_LED_GREEN);
}

void led_greenOff(void){
	PORTH |= (1 << PIN_LED_GREEN);
}

void led_blueOn(void){
	PORTH &= ~(1 << PIN_LED_BLUE);
}

void led_blueOff(void){
	PORTH |= (1 << PIN_LED_BLUE);
}

// Timer 4 initialized in PWM mode for brightness control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer4_init()
{
	cli(); //disable all interrupts
	
	TCCR4B = 0x00;	//Stop
	
	TCNT4H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT4L = 0x00;	//Counter lower 8-bit value to which OCR5xH value is compared with
	
	OCR4AH = 0x00;	//Output compare register high value for Red Led
	OCR4AL = 0xFF;	//Output compare register low value for Red Led
	
	OCR4BH = 0x00;	//Output compare register high value for Blue Led
	OCR4BL = 0xFF;	//Output compare register low value for Blue Led

	OCR4CH = 0x00;	//Output compare register high value for Green Led
	OCR4CL = 0xFF;	//Output compare register low value for Green Led
	
	//  Clear OC4A, OC4B & OC4C on compare match (set output to low level)
	TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);
	TCCR4A &= ~((1 << COM4A0) | (1 << COM4B0) | (1 << COM4C0));

	// FAST PWM 8-bit Mode
	TCCR4A |= (1 << WGM40);
	TCCR4A &= ~(1 << WGM41);
	TCCR4B |= (1 << WGM42);
	
	// Set Prescalar to 64
	TCCR4B |= (1 << CS41) | (1 << CS40);
	TCCR4B &= ~(1 << CS42);
	
	sei(); //re-enable interrupts
}

// Function for brightness control of all 3 LEDs
void brightness (unsigned char red_led, unsigned char green_led, unsigned char blue_led){
	OCR4AL = 255 - (unsigned char)red_led; 	// active low thats why subtracting by 255
	OCR4CL = 255 - (unsigned char)green_led;
	OCR4BL = 255 - (unsigned char)blue_led;
}

//use this function to initialize all devices
void init_devices (void) {
	led_init();
	timer4_init();
}

//Main Function
int main(){
	init_devices();
	int step = 0;
	
	while(1){

		// decrease Red and increase Green, Blue off
		for (step = 0; step < 256; step++){
			
			brightness(255 - step, 0 + step, 0);
			_delay_ms(10);
			
		}

		// Red off, decrease Green and increase Blue
		for (step = 0; step < 256; step++){
			
			brightness(0, 255 - step, 0 + step);
			_delay_ms(10);
			
		}

		// increase Red, Green off and decrease Blue
		for (step = 0; step < 256; step++){
			
			brightness(0 + step, 0, 255 - step);
			_delay_ms(10);
			
		}
	}
}
