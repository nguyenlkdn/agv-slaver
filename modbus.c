#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "modbus.h"
#include "lcd_lib.h"
#include "config.h"

volatile unsigned char BusState = 0;
volatile uint16_t modbusTimer = 0;
volatile uint16_t DataPos = 0;
volatile unsigned char PacketTopIndex = 7;
volatile unsigned char modBusStaMaStates = 0;
/*

*/

/*

*/
uint16_t SLAVER_REG_READ[50];
uint16_t SLAVER_REG_WRITE[50];
/*

*/

/*

*/
uint8_t modbusGetBusState(void)
{
	return BusState;
}

#if ADDRESS_MODE == SINGLE_ADR
volatile unsigned char Address = 0x01;
uint8_t modbusGetAddress(void)
{
	return Address;
}

void modbusSetAddress(unsigned char newadr)
{
	Address = newadr;
}
#endif

#if PHYSICAL_TYPE == 485
void transceiver_txen(void)
{
	TRANSCEIVER_ENABLE_PORT|=(1<<TRANSCEIVER_ENABLE_PIN);
}

 void transceiver_rxen(void)
{
	TRANSCEIVER_ENABLE_PORT&=~(1<<TRANSCEIVER_ENABLE_PIN);
}
#endif

/* @brief: A fairly simple Modbus compliant 16 Bit CRC algorithm.
*
*  	Returns 1 if the crc check is positive, returns 0 and saves the calculated CRC bytes
*	at the end of the data array if it fails.
*  	
*/
/* CRC algorithm */
uint16_t CRC16(uint16_t crc, uint8_t data)
{
    const uint16_t Poly16=0xA001;
    uint16_t LSB, i;
    crc = ((crc^data) | 0xFF00) & (crc | 0x00FF);
    for (i=0; i<8; i++) {
        LSB=(crc & 0x0001);
        crc=crc/2;
        if (LSB)
        {
            crc=crc^Poly16;
        }
    }
    return(crc);
}

uint16_t swap16bits(uint16_t input)
{
    uint16_t tmp=input&0xFF00;
    input = ((input<<8) | (tmp>>8));
    return input;
}

uint16_t crc16(uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    int i;
    for (i = 0; i < length; i++) {
        //UARTprintf("Checksum %d: %x\n", i, data[i]);
        crc = CRC16 (crc, data[i]);
    }
    return swap16bits(crc);
}


/* @brief: Back to receiving state.
*
*/
void modbusReset(void)
{
	BusState=(1<<TimerActive); //stop receiving (error)
	modbusTimer=0;
}
uint8_t getLOWbyte(uint16_t input)
{
    return input & 0xFF;
}

/* Get HIGH byte of int */
uint8_t getHIGHbyte(uint16_t input)
{
    return (input >> 8) & 0xFF;
}

/* Combine HIGH and LOW bytes */
uint16_t combineBytes(uint8_t high, uint8_t low)
{
    return (uint16_t) (high << 8) + low;
}
void USART_SendByte(uint8_t u8Data){

  // Wait until last byte has been transmitted
  while((UCSRA &(1<<UDRE)) == 0);

  // Transmit data
  UDR = u8Data;
}

void USART_SendBytes(uint8_t *data, uint16_t number)
{
	int i;
	transceiver_txen();
	for(i=0;i<number;i++)
	{
		USART_SendByte(data[i]);
	}
	_delay_ms(2);
	transceiver_rxen();
}

uint8_t modbusarrayProcessing(uint8_t *buffer, uint16_t lenght, uint8_t addr)
{
    int index, i;
    uint16_t crcgen, crcrec;
    uint8_t f03processbuffer[50];
    uint8_t f16processbuffer[50];
    uint16_t startaddr = 0;
    uint16_t numofbytes = 0;
    for(index=0;index<lenght;index++)
    {
        if(buffer[index] == addr)
        {
            switch(buffer[index+1])
            {
            case 0x03:
                for(i=0;i<8;i++)
                {
                    f03processbuffer[i] = buffer[index+i];
                }
                crcgen = crc16(f03processbuffer, 6);
                crcrec = combineBytes(f03processbuffer[6], f03processbuffer[7]);
                if(crcgen == crcrec)
                {
                	startaddr = combineBytes(f03processbuffer[2], f03processbuffer[3]);
                	numofbytes = combineBytes(f03processbuffer[4], f03processbuffer[5])*2;
                	if(numofbytes <= MODBUS_MAXREG)
                	{
                		f03processbuffer[2] = numofbytes;
                		for(i=0;i<numofbytes;i++)
                		{
                			f03processbuffer[3 + (i*2)] = getHIGHbyte(SLAVER_REG_READ[i+startaddr]);
                			f03processbuffer[3 + (i*2) + 1] = getLOWbyte(SLAVER_REG_READ[i+startaddr]);
                		}
                		crcgen = crc16(f03processbuffer, 3+f03processbuffer[2]);
                		f03processbuffer[numofbytes + 3] = getHIGHbyte(crcgen);
                		f03processbuffer[numofbytes + 3 + 1] = getLOWbyte(crcgen);
                		USART_SendBytes(f03processbuffer, numofbytes + 3 + 2);
                		return 0;
                	}
                	else
                	{
                		return 2;
                	}
                	// LCDGotoXY(0, 0);
                	// LCDsendNum(f03processbuffer[numofbytes + 3]);
                	// LCDGotoXY(0, 1);
                	// LCDsendNum(f03processbuffer[numofbytes + 3 + 1]);
                	//LCDsendChar(' ');
                	// LCDsendNum(crcrec);
                	// LCDGotoXY(0, 0);
                	// for(i=0;i<DataPos;i++)
                	// {
                	// 	if(i == 6)
                	// 	{
                	// 		break;
                	// 	}
                	// 	LCDsendNum(buffer[i]);
                	// 	LCDsendChar(' ');
                	// }
                	// LCDGotoXY(0, 1);
                	// for(;i<DataPos;i++)
                	// {
                	// 	LCDsendNum(buffer[i]);
                	// 	LCDsendChar(' ');
                	// }
                }
                else
                {
                	// LCDGotoXY(0, 1);
                 //    LCDsendNum(crcgen);
                	// LCDsendChar(' ');
                	// LCDsendNum(crcrec);
                    continue;
                }
            case 0x10:
	            for(i=0;i<lenght;i++)
	            {
	                f16processbuffer[i] = buffer[index+i];
	            }
	            startaddr = combineBytes(f16processbuffer[2], f16processbuffer[3]);
	            numofbytes = combineBytes(f16processbuffer[4], f16processbuffer[5])*2;              
	            if((lenght - index) < numofbytes+9)
                {
                	// LCDGotoXY(0, 0);
                	// LCDPrintf(0, 0, "Mismatch length");
                    return 4;
                }
                else
                {
                	crcgen = crc16(f16processbuffer, numofbytes+7);
                	crcrec = combineBytes(f16processbuffer[numofbytes+7], f16processbuffer[numofbytes+8]);
                	if((numofbytes+startaddr) > MODBUS_MAXREG)
                	{
                		//LCDPrintf(0, 0, "T3");
                		return 2;
                	}
                	else
                	{
                		if(crcrec == crcgen)
                		{
                			//LCDPrintf(0, 0, "T1");
                			LCDGotoXY(0, 0);
                			for(i=0;i<numofbytes/2;i++)
                			{
                				SLAVER_REG_WRITE[startaddr+i] = combineBytes(f16processbuffer[(i*2)+7], f16processbuffer[(i*2)+8]);
                				// LCDsendNum(SLAVER_REG_WRITE[i]);
                				// LCDsendChar(' ');
                			}
                			crcgen = crc16(f16processbuffer, 6);
                			f16processbuffer[6] = getHIGHbyte(crcgen);
                			f16processbuffer[7] = getLOWbyte(crcgen);
                			USART_SendBytes(f16processbuffer, 8);

                			// LCDGotoXY(0, 1);
                			// LCDsendNum(f16processbuffer[6]);
                			// LCDsendChar(' ');
                			// LCDsendNum(f16processbuffer[7]);
                			// LCDsendChar(' ');
                			// LCDsendNum(crcgen);
                			// LCDsendChar(' ');
                			// LCDsendNum(f16processbuffer[3]);
                			// LCDsendChar(' ');
                			// LCDsendNum(f16processbuffer[4]);
                			// LCDsendChar(' ');
                			// LCDsendNum(f16processbuffer[5]);
                			// LCDsendChar(' ');
                			return 0;
                		}
                		else
                		{
                			//LCDPrintf(0, 1, "T2");
                			return 1;
                		}
                	}


                	// LCDGotoXY(0, 0);
                	// LCDsendNum(startaddr);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+1]);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+2]);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+3]);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+4]);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+5]);
                	// LCDGotoXY(0, 1);
                	// LCDsendNum(numofbytes);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+7]);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+8]);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+9]);
                	// LCDsendChar(' ');
                	// LCDsendNum(f16processbuffer[numofbytes+10]);
                	// LCDsendChar(' ');
                }
                // else
                // {
                //     if(crcgen == crcrec)
                //     {
                //         uint16_t startaddr = combineBytes(rxbuffer[index+2], rxbuffer[index+3]);
                //         uint16_t numofreg = combineBytes(rxbuffer[index+4], rxbuffer[index+5]);
                //         if(startaddr + numofreg > ROBOT_RXBUFFER_SIZE)
                //         {
                //         }
                //         else
                //         {
                //             int i;
                //             for(i=startaddr;i<startaddr+numofreg;i++)
                //             {
                //                 ROBOTRX_Buffer[i] = combineBytes(rxbuffer[index+7+i*2], rxbuffer[index+8+i*2]);
                //             }
                //             uint8_t respond[8];
                //             for(i=0;i<6;i++)
                //             {
                //                 respond[i] = rxbuffer[index+i];
                //             }
                //             crcgen = crc16Gen(respond, 6);
                //             respond[6] = getHIGHbyte(crcgen);
                //             respond[7] = getLOWbyte(crcgen);
                //             return 0;
                //         }
                //     }
                //     else
                //     {
                //         continue;
                //     }
                // }

                break;
            default:
                continue;
            }
        }
    }
    return 0;
}
