//*****************************************************************************
//
// File Name	: 'main.c'
// Title		: LCD demo
// Author		: Scienceprog.com - Copyright (C) 2007
// Created		: 2007-03-29
// Revised		: 2007-08-28
// Version		: 1.0
// Target MCU	: Atmel AVR series 
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "lcd_lib.h"

//#define F_CPU 8000000ul

//Strings stored in AVR Flash memory
const uint8_t LCDwelcomeln1[] PROGMEM="AVR LCD DEMO\0";
const uint8_t LCDprogress[] PROGMEM="Loading...\0";
const uint8_t LCDanimation[] PROGMEM=" LCD animation \0";
#define CALLING_LED_PIN	0x10
#define SPARE_LED_PIN	0x20
#define RS485RW_PIN		0x04
// additional custom LCD characters
const uint8_t backslash[8] PROGMEM= 
{
0b00000000,//back slash
0b00010000,
0b00001000,
0b00000100,
0b00000010,
0b00000001,
0b00000000,
0b00000000
};

void IOConfig(void)
{
	DDRC |= (SPARE_LED_PIN | CALLING_LED_PIN);
	PORTC &= ~(SPARE_LED_PIN | CALLING_LED_PIN);
	DDRD |= (RS485RW_PIN);
	PORTD |= (RS485RW_PIN);
}
//delay 1s
void delay1s(void)
{
	uint8_t i;
	for(i=0;i<100;i++)
	{
		_delay_ms(10);
	}
}

//demonstration of progress bar
void progress(void)
{
	LCDclr();
	CopyStringtoLCD(LCDwelcomeln1, 3, 0);	
	delay1s();
	LCDclr();
	CopyStringtoLCD(LCDprogress, 3, 0);
	for(uint8_t i=0;i<255;i++)
		{
			_delay_ms(10);
			LCDGotoXY(0, 1);
			LCDprogressBar(i, 255, 16);
		}
}
//demonstration of animation
void demoanimation(void)
{
	LCDclr();
	LCDdefinechar(backslash,0);
	CopyStringtoLCD(LCDanimation, 0, 0);
	for(uint8_t i=0;i<3;i++)
		{
			LCDGotoXY(8, 1);
			LCDsendChar(0);
			delay1s();
			LCDGotoXY(8, 1);
			LCDsendChar('-');
			delay1s();
			LCDGotoXY(8, 1);
			LCDsendChar('/');
			delay1s();
			LCDGotoXY(8, 1);
			LCDsendChar('|');
			delay1s();
			LCDGotoXY(8, 1);
			LCDsendChar(8);//backslash
			delay1s();
			LCDGotoXY(8, 1);
			LCDsendChar('-');
			delay1s();
			LCDGotoXY(8, 1);
			LCDsendChar('/');
			delay1s();
			LCDGotoXY(8, 1);
			LCDsendChar('|');
			delay1s();
		}	
}
void uartInit(void)
{

}
void timerInit(void)
{

	// enable timer overflow interrupt for both Timer0 and Timer1
	TIMSK=(1<<TOIE0) | (1<<TOIE1);
	
	// set timer0 counter initial value to 0
	TCNT0=0x00;

	// start timer0 with /1024 prescaler
	TCCR0 = (1<<CS02) | (1<<CS00);

	// lets turn on 16 bit timer1 also with /1024
	TCCR1B |= (1 << CS10) | (1 << CS12);
	
	// enable interrupts
	sei(); 

}
/*

*/

/*

*/
int main(void)
{
	//IOConfig();
	LCDinit();//init LCD bit, dual line, cursor right
	IOConfig();
	timerInit();
	LCDclr();//clears LCD
	//modbusInit();

	// LCDGotoXY(0, 0);
	// LCDstring("TDNC", 4);
	LCDGotoXY(0, 1);
	LCDstring("TDNC", 4);
	while(1)//loop demos
	{
		//modbusTickTimer();
		//PORTC ^= (SPARE_LED_PIN | CALLING_LED_PIN);
		delay1s();

		//PORTD = (RS485RW_PIN);
		//modbusExchangeRegisters(user, 0x01, 5);
		//progress();
		//delay1s();
		//PORTC = 0x00;
		//PORTD &=~ (RS485RW_PIN);
		//progress();
		// PORTC &= ~(SPARE_LED_PIN | CALLING_LED_PIN);
		// uart_puts("ABCDEFGG");
		// delay1s();

		// demoanimation();
	}
	return 0;
}

// timer1 overflow
ISR(TIMER0_OVF_vect) {
    // process the timer1 overflow here
    PORTC ^= (CALLING_LED_PIN);

}

// timer1 overflow
ISR(TIMER1_OVF_vect) {
    // process the timer1 overflow here
    PORTC ^= (SPARE_LED_PIN);

}