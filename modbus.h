//*****************************************************************************
//
// File Name	: 'lcd_lib.h'
// Title		: 8 and 4 bit LCd interface
// Author		: Scienceprog.com - Copyright (C) 2007
// Created		: 2007-03-29
// Revised		: 2007-08-08
// Version		: 1.0 
// Target MCU	: Atmel AVR series
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************
#ifndef MOSBUS
#define MOSBUS

#include <inttypes.h>
#include <avr/io.h>

/*
 * Available address modes.
*/
#define MULTIPLE_ADR 2
#define SINGLE_ADR 1

/*
* Use SINGLE_ADR or MULTIPLE_ADR, default: SINGLE_ADR
* This is useful for building gateways, routers or clients that for whatever reason need multiple addresses.
*/
#define ADDRESS_MODE SINGLE_ADR

/*
* Use 485 or 232, default: 485
* Use 232 for testing purposes or very simple applications that do not require RS485 and bus topology.
*/
#define PHYSICAL_TYPE 485 //possible values: 485, 232

/*
#define modbusBaudrate 38400
#define modbusBlocksize 10
#define modbusBlockTime ((float)modbusBlocksize*1000000)/((float) modbusBaudrate) //is 260 für 38400
#define timerISROccurenceTime 102

#define TimeoutStartOfMessage  (uint16_t)(modbusBlockTime*3.5/(float)timerISROccurenceTime)
#define TimeoutEndOfMessage (uint16_t)(modbusBlockTime*4/(float)timerISROccurenceTime)
#define ReceiveMaxGap  (uint16_t)(modbusBlockTime*1.5/(float)timerISROccurenceTime)
*/
#define modbusInterFrameDelayReceiveStart 16
#define modbusInterFrameDelayReceiveEnd 18
#define modbusInterCharTimeout 7

/**
 * @brief    Defines the maximum Modbus frame size accepted by the device. 255 is the default
 *           and also the maximum value. However, it might be useful to set this to lower
 *           values, with 8 being the lowest possible value, in order to save on ram space.
 */
#define MaxFrameIndex 255

/**
 * @brief    Modbus Function Codes
 *           Refer to modbus.org for further information.
 *           It's good practice to return exception code 01 in case you receive a function code
 *           that you haven't implemented in your application.
 */
#define fcReadCoilStatus 1 //read single/multiple coils
#define fcReadInputStatus 2 //read single/multiple inputs
#define fcReadHoldingRegisters 3 //read analog output registers
#define fcReadInputRegisters 4 //read analog input registers (2 Bytes per register)
#define fcForceSingleCoil 5 //write single bit
#define fcPresetSingleRegister 6 //write analog output register (2 Bytes)
#define fcForceMultipleCoils 15 //write multiple bits
#define fcPresetMultipleRegisters 16 //write multiple analog output registers (2 Bytes each)
#define fcReportSlaveID 17 //read device description, run status and other device specific information

#define TRANSCEIVER_ENABLE_PORT PORTD
#define TRANSCEIVER_ENABLE_PIN 2
#define TRANSCEIVER_ENABLE_PORT_DDR DDRD

/**
 * @brief    Internal bit definitions
 */
#define BusTimedOut 0
#define Receiving 1
#define Transmitting 2
#define ReceiveCompleted 3
#define TransmitRequested 4
#define TimerActive 5
#define GapDetected 6

#define MODBUS_MAXREG 50

extern uint16_t SLAVER_REG_READ[50];
extern uint16_t SLAVER_REG_WRITE[50];

/**
* @brief    Configures the UART. Call this function only once.
*/
extern void modbusInit(void);

/**
* @brief    receive/transmit data array
*/
extern uint8_t rxbuffer[MaxFrameIndex+1];

/**
* @brief    Current receive/transmit position
*/
volatile uint16_t DataPos;

/**
 * This only applies to single address mode.
 */
#if ADDRESS_MODE == SINGLE_ADR
	/**
	* @brief: Read the device address   
	*/
	extern uint8_t modbusGetAddress(void);

	/**
	* @brief: Set the device address
	*         Arguments: - newadr: the new device address
	*/
	extern void modbusSetAddress(unsigned char newadr);
#endif

/* @brief: Sends a response.
*
*         Arguments: - packtop, index of the last byte in rxbuffer
*                      that contains payload. Maximum value is
*				       MaxFrameIndex-2.
*/
extern void modbusSendMessage(unsigned char packtop);

/* @brief: Sends a Modbus exception.
*
*         Arguments: - exceptionCode
*/
extern void modbusSendException(unsigned char exceptionCode);

/* @brief: Discards the current transaction. For MULTIPLE_ADR-mode and general
*		   testing purposes. Call this function if you don't want to reply at all.
*/
void modbusReset(void);

/**
 * @brief    Call this function whenever possible and check if its return value has the ReceiveCompleted Bit set.
 *           Preferably do this in the main while. I do not recommend calling this function within ISRs.
 * @example  if (modbusGetBusState() & (1<<ReceiveCompleted)) {
 *           modbusSendExcepton(ecIllegalFunction);
 *           }
 */
extern uint8_t modbusGetBusState(void);

/**
 * @brief    Call every 100µs using a timer ISR.
 */
extern void modbusTickTimer(void);

/**
 * @brief    Returns amount of bits/registers requested.
 */
extern uint16_t modbusRequestedAmount(void);

/**
 * @brief    Returns the address of the first requested bit/register.
 */
extern uint16_t modbusRequestedAddress(void);

/* A fairly simple and hopefully Modbus compliant 16 Bit CRC algorithm.
*  Returns 1 if the crc check is positive, returns 0 if it fails.
*  Appends two crc bytes to the array.
*/
uint16_t crc16(uint8_t *data, uint8_t length);

/* @brief: Handles single/multiple input/coil reading and single/multiple coil writing.
*
*         Arguments: - ptrToInArray: pointer to the user's data array containing bits
*                    - startAddress: address of the first bit in the supplied array
*                    - size: input array size in the requested format (bits)
*
*/
extern uint8_t modbusExchangeBits(volatile uint8_t *ptrToInArray, uint16_t startAddress, uint16_t size);

/* @brief: Handles single/multiple register reading and single/multiple register writing.
*
*         Arguments: - ptrToInArray: pointer to the user's data array containing registers
*                    - startAddress: address of the first register in the supplied array
*                    - size: input array size in the requested format (16bit-registers)
*
*/
extern uint8_t modbusExchangeRegisters(volatile uint16_t *ptrToInArray, uint16_t startAddress, uint16_t size);
extern uint8_t modbusarrayProcessing(uint8_t *buffer, uint16_t lenght, uint8_t addr);
#endif
