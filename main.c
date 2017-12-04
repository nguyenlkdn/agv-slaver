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
#include "modbus.h"
#include "config.h"

uint16_t SLAVER_REG_READ[50];
uint16_t SLAVER_REG_WRITE[50];
uint8_t rxbuffer[MaxFrameIndex+1];
uint32_t data=0;
int32_t rc;

uint16_t CONFIRMED = 0;
uint16_t calling_debound = 0;
uint16_t stop_debound = 0;
uint16_t up_debound = 0;
uint16_t down_debound = 0;
uint16_t haskey = 0;

uint8_t request = 0;
uint8_t updown = 0;
uint8_t isprint = 0;
//Strings stored in AVR Flash memory
const uint8_t LCDwelcomeln1[] PROGMEM="AGV Version\0";
const uint8_t LCDprogress[] PROGMEM="Loading...\0";
const uint8_t LCDanimation[] PROGMEM=" LCD animation \0";

// Define baud rate
#define USART_BAUDRATE 38400   
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

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
	PORTD &= ~(RS485RW_PIN);
	DDRB &= ~(CALLING_BUTTON_PIN | STOP_BUTTON_PIN|UP_BUTTON_PIN|DOWN_BUTTON_PIN);
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
	for(uint8_t i=0;i<50;i++)
		{
			_delay_ms(1);
			LCDGotoXY(0, 1);
			LCDprogressBar(i, 50, 16);
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
	UBRRL = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
	UBRRH = (BAUD_PRESCALE >> 8); 
	UCSRB = ((1<<TXEN)|(1<<RXEN) | (1<<RXCIE));
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
	uartInit();
	LCDclr();//clears LCD
	sei(); 
	progress();
	LCDPrintf(0, 1, "                ");
	LCDPrintf(0, 0, "AGV ");
	LCDsendNum(SLAVER_ADDR);
	LCDsendChar(':');
	LCDsendChar(' ');

	while(1)
	{
		if(request == 0)
		{
			LCDPrintf(7, 0, "NoReq    ");
		}
		else
		{
			LCDPrintf(7, 0, "Request  ");
		}
		if(DataPos >= 8)
		{
			rc = modbusarrayProcessing(rxbuffer, DataPos, SLAVER_ADDR);
			if(rc == 0)
			{
				DataPos = 0;
			}
		}

		LCDGotoXY(0, 1);
		LCDsendNum(SLAVER_REG_WRITE[0]);
		LCDsendChar(' ');
		LCDsendNum(SLAVER_REG_WRITE[1]);
		LCDsendChar(' ');
		LCDsendNum(SLAVER_REG_WRITE[2]);
		LCDsendChar(' ');
		LCDsendNum(SLAVER_REG_WRITE[3]);
		LCDsendChar(' ');
		LCDsendNum(SLAVER_REG_WRITE[4]);
		LCDsendChar(' ');
		_delay_ms(100);
	}
	return 0;
}

// timer1 overflow
ISR(TIMER0_OVF_vect) {
    // process the timer1 overflow here
    if(SLAVER_REG_WRITE[0] == SLAVER_ADDR)
    {
    	PORTC ^= (CALLING_LED_PIN);
    }
    else
    {
    	PORTC &= ~(CALLING_LED_PIN);
    }

    if((SLAVER_REG_WRITE[1] == 1) && (SLAVER_REG_READ[0] == 0))
    {
    	request = 1;
    	SLAVER_REG_READ[0] = 1;
    }
    haskey = 0;
    if(bit_is_clear(PINB, 0))
    {
    	haskey = 1;
    	if(++down_debound == 5)
    	{
    		PORTC |= (SPARE_LED_PIN);
    		updown = 1;
    		SLAVER_REG_READ[2] = 0;
    		SLAVER_REG_READ[3] = 1;
    		// LCDPrintf(0, 0, "DOWN");
    	}
    }
    else
    {
    	down_debound = 0;
    }

    if(bit_is_clear(PINB, 1))
    {
    	haskey = 2;
    	if(++up_debound == 5)
    	{
    		// LCDPrintf(0, 0, "UP");
    		PORTC |= (SPARE_LED_PIN);
    		updown = 0;
    		SLAVER_REG_READ[2] = 1;
    		SLAVER_REG_READ[3] = 0;
    	}
    }
    else
    {
    	up_debound = 0;
    }

    if(bit_is_clear(PINB, 2))
    {
    	haskey = 3;
    	if(++calling_debound == 5)
    	{
    		// LCDPrintf(0, 0, "CALL");
    		PORTC |= (SPARE_LED_PIN);
    		SLAVER_REG_READ[0] = 1;
    		request = 1;
    	}
    }
    else
    {
    	calling_debound = 0;
    }

    if(bit_is_clear(PINB, 3))
    {
    	haskey = 4;
    	if(++stop_debound == 5)
    	{
    		// LCDPrintf(0, 0, "STOP");
    		PORTC |= (SPARE_LED_PIN);
    		SLAVER_REG_READ[0] = 0;
    		request = 0;
    	}
    }
    else
    {
    	stop_debound = 0;
    }
	if((haskey == 0))
	{
		PORTC &=~ (SPARE_LED_PIN);
	}
    // if(SLAVER_REG_WRITE[0] == haskey || (haskey != 0))
    // {

    // }
    // else
    // {
    // 	PORTC &=~ (SPARE_LED_PIN);
    // 	// if((haskey == 0))
    // 	// {
    // 	// 	PORTC &=~ (SPARE_LED_PIN);
    // 	// }
    // }

}

// timer1 overflow
ISR(TIMER1_OVF_vect) {
    // process the timer1 overflow here
    //PORTC ^= (SPARE_LED_PIN);

}

ISR(USART_RXC_vect){
	//volatile uint8_t value = UDR;             //read UART register into value
	rxbuffer[DataPos++] = UDR;
	if(DataPos == 255)
	{
		DataPos = 0;
	}
}