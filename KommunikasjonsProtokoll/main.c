/*! 
 *\file main.c	
 */

/*!
 *\author Christoffer Boothby and James Alexander Cowie
 *\date 2018
 *\copyright GNU Public License.
 */

#define version 0x0063

/* Comments:
 * This C code is made for G-Chaser project on the "EL-BOKS" card, made by Erlend Restad.
 * It is possible to implement this software on a different microcontroller or same type
 * but it is highly recommended to change pins.
 * Commenting in this software is primarily made for the editors, possible future editors
 * and for easy understanding on the thesis. Requires prior knowledge or understanding of 
 * the datasheet for the ICs explained on the two lines below, and datasheet for Atmega328PB.
 * SPI Communication is made for the IC LTC1859(ADC) and DAC8420(DAC).
 * I2C Communication is made for the IC LTC4151.
 *
 */


// UART settings
#include "comm.h"
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/wdt.h>

// A more robust way for setting baudrates-- F_CPU is set inside comm.h
#define BAUD 230400
#include <util/setbaud.h>


////////////////////////////////////////////////////////
//////////////////// Variables /////////////////////////
////////////////////////////////////////////////////////

uint8_t volatile mainComm_Counter; // Choose which packet to be sent out.
uint8_t volatile subComm_Counter; // Choose which of the subcomms to be used and sent out.
uint8_t volatile packetID;

uint8_t volatile mainRefresher_Counter;
uint8_t volatile subRefresher_Counter;

uint16_t volatile crc16;
uint8_t volatile channelData[17];
uint8_t volatile channelData_2[17];
uint8_t volatile channelData_i2c[13];
uint8_t volatile lastChannelAccessed;
uint8_t volatile lastChannelAccessed_2;
uint8_t volatile tempData[3];

uint8_t mcusr;


void Port_Init()
{
	// Datadirections only, to set, use PORTxn, 0 = Input, 1 = Output
	DDRB = (0<<ADC_2_BUSY)|(1<<CS_DAC_1)|(1<<LD_DAC_1);
	DDRC = (1<<CS_DAC_2)|(1<<LD_DAC_2)|(0<<ADC_1_BUSY);
	DDRD = (1<<ADC_READ_2)|(1<<ADV_CONVERSION_START_2);
	DDRE = (1<<ADC_READ_1)|(1<<ADV_CONVERSION_START_1);

	// Start configurations
	
	// ADC STARTUP PIN CONFIGURATIONS
	PORTD = (1<<ADC_READ_2)|(0<<ADV_CONVERSION_START_2);
	PORTE = (1<<ADC_READ_1)|(0<<ADV_CONVERSION_START_1);
	// DAC STARTUP PIN CONFIGURATION
	PORTC = (1<<LD_DAC_2)|(1<<CS_DAC_2);
	PORTB = (1<<LD_DAC_1)|(1<<CS_DAC_1);
	// wdt_reset();
}

/*! \fn void Port_Init()
 * \brief Port initalization function.
 * \param None.
 * \return None.
*/

void USART_Init()
{
	/* Set baud rate */
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	
	/* Enable transmitter */
	UCSR0B = (1<<TXEN0)|(1<<UDRIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
	// wdt_reset();
}


// Interrupt handler
ISR(USART0_UDRE_vect)	
{
	switch(mainComm_Counter)
	{
		case 0: // SYNC 1
			UDR0 = 0x6B;
			mainComm_Counter++;
			break;
		case 1: // SYNC 2
			UDR0 = 0x90;
			mainComm_Counter++;
			break;
		case 2: // SUBCOMM
			crc16 = _crc_xmodem_update(crc16, packetID);
			UDR0 = packetID;
			mainComm_Counter++;
			break;
		case 3: // GT1-1 
			tempData[0] = channelData[4];
			tempData[1] = channelData[5];
			crc16 = _crc_xmodem_update(crc16, tempData[0]);
			UDR0 = tempData[0];
			mainComm_Counter++;
			break;
		case 4: // GT1-2
			crc16 = _crc_xmodem_update(crc16, tempData[1]);
			UDR0 = tempData[1];
			mainComm_Counter++;
			break;
		case 5: // GT2-1
			tempData[0] = channelData_2[4];
			tempData[1] = channelData_2[5];
			crc16 = _crc_xmodem_update(crc16, tempData[0]);
			UDR0 = tempData[0];
			mainComm_Counter++;
			break;
		case 6: // GT2-2
			crc16 = _crc_xmodem_update(crc16, tempData[1]);
			UDR0 = tempData[1];
			mainComm_Counter++;
			break;
		case 7: // MP-1
			tempData[0] = channelData[8];
			tempData[1] = channelData[9];
			crc16 = _crc_xmodem_update(crc16, tempData[0]);
			UDR0 = tempData[0];
			mainComm_Counter++;
			break;
		case 8: // MP-2
			crc16 = _crc_xmodem_update(crc16, tempData[1]);
			UDR0 = tempData[1];
			mainComm_Counter++;
			break;
		case 9: // GB2-1
			tempData[0] = channelData_2[12];
			tempData[1] = channelData_2[13];
			crc16 = _crc_xmodem_update(crc16, tempData[0]);
			UDR0 = tempData[0];
			mainComm_Counter++;
			break;
		case 10: // GB2-2
			crc16 = _crc_xmodem_update(crc16, tempData[1]);
			UDR0 = tempData[1];
			mainComm_Counter++;
			break;
		case 11: // GB1-1
			tempData[0] = channelData[12];
			tempData[1] = channelData[13];
			crc16 = _crc_xmodem_update(crc16, tempData[0]);
			UDR0 = tempData[0];
			mainComm_Counter++;
			break;
		case 12: // GB1-2
			crc16 = _crc_xmodem_update(crc16, tempData[1]);
			UDR0 = tempData[1];
			mainComm_Counter++;
			break;
		case 13:
			switch(subComm_Counter)
			{
				case 0: // Bias GT1
					tempData[0] = channelData[6];
					tempData[1] = channelData[7];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 1:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 2: // Bias GT2
					tempData[0] = channelData_2[6];
					tempData[1] = channelData_2[7];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 3:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 4: // Bias MP
					tempData[0] = channelData[10];
					tempData[1] = channelData[11];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 5:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 6: // Bias GB2
					tempData[0] = channelData_2[14];
					tempData[1] = channelData_2[15];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 7:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 8: // Bias GB1
					tempData[0] = channelData[14];
					tempData[1] = channelData[15];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 9:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 10: // PCB1 INT
					tempData[0] = channelData[0];
					tempData[1] = channelData[1];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 11:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 12: // PCB1 EXT
					tempData[0] = channelData[2];
					tempData[1] = channelData[3];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 13:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 14: // PCB2 INT
					tempData[0] = channelData_2[0];
					tempData[1] = channelData_2[1];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 15:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 16: // PCB2 EXT
					tempData[0] = channelData_2[2];
					tempData[1] = channelData_2[3];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 17:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter = 30;
					mainComm_Counter++;
					break;
				case 30: // SP
					tempData[0] = channelData_2[8];
					tempData[1] = channelData_2[9];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 31: 
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 32: // BIAS SP
					tempData[0] = channelData_2[10];
					tempData[1] = channelData_2[11];
					crc16 = _crc_xmodem_update(crc16, tempData[0]);
					UDR0 = tempData[0];
					subComm_Counter++;
					packetID++;
					break;
				case 33:
					crc16 = _crc_xmodem_update(crc16, tempData[1]);
					UDR0 = tempData[1];
					subComm_Counter++;
					mainComm_Counter++;
					break;
				case 34: // Version #
					crc16 = _crc_xmodem_update(crc16, (version>>8));
					UDR0 = (version>>8);
					subComm_Counter++;
					packetID = 0;
					break;
				case 35:
					crc16 = _crc_xmodem_update(crc16, (uint8_t) version);
					UDR0 = (uint8_t)version;
					subComm_Counter = 0;
					mainComm_Counter++;
					break;
			}
			break;
		case 14: // CRC-1
			UDR0 = (crc16>>8);
			mainComm_Counter++;
			break;
		case 15: //CRC-2
			UDR0 = (uint8_t)crc16;
			mainComm_Counter = 0;
			crc16 = 0xFFFF;
			break;
	}
}

// Last channel accessed, store the data received into channel data
// CH0: 0-1: CH1: 2-3 CH2: 4-5 CH3: 6-7 CH4: 8-9 CH5: 10-11CH6 12-13 CH7 14-15
void channelUpdater_1(uint8_t * datain) // if channel and store value
{
	switch (lastChannelAccessed)
	{
		case LTC1859_CH0: // CH0
			channelData[0] = datain[0];
			channelData[1] = datain[1];		
			break;
		case LTC1859_CH1: // CH1
			channelData[2] = datain[0];
			channelData[3] = datain[1];
			break;
		case LTC1859_CH2: // CH2
			channelData[4] = datain[0];
			channelData[5] = datain[1];
			break;
		case LTC1859_CH3: // CH3
			channelData[6] = datain[0];
			channelData[7] = datain[1];
			break;
		case LTC1859_CH4: // CH4
			channelData[8] = datain[0];
			channelData[9] = datain[1];
			break;
		case LTC1859_CH5: // CH5
			channelData[10] = datain[0];
			channelData[11] = datain[1];
			break;
		case LTC1859_CH6: // CH6
			channelData[12] = datain[0];
			channelData[13] = datain[1];
			break;
		case LTC1859_CH7: // CH7
			channelData[14] = datain[0];
			channelData[15] = datain[1];
			break;
	}
	// wdt_reset();
}

void channelUpdater_2(uint8_t * datain) // if channel and store value
{
	switch (lastChannelAccessed_2)
	{
		case LTC1859_CH0: // CH0
			channelData_2[0] = datain[0];
			channelData_2[1] = datain[1];
			break;
		case LTC1859_CH1: // CH1
			channelData_2[2] = datain[0];
			channelData_2[3] = datain[1];
			break;
		case LTC1859_CH2: // CH2
			channelData_2[4] = datain[0];
			channelData_2[5] = datain[1];
			break;
		case LTC1859_CH3: // CH3
			channelData_2[6] = datain[0];
			channelData_2[7] = datain[1];
			break;
		case LTC1859_CH4: // CH4
			channelData_2[8] = datain[0];
			channelData_2[9] = datain[1];
			break;
		case LTC1859_CH5: // CH5
			channelData_2[10] = datain[0];
			channelData_2[11] = datain[1];
			break;
		case LTC1859_CH6: // CH6
			channelData_2[12] = datain[0];
			channelData_2[13] = datain[1];
			break;
		case LTC1859_CH7: // CH7
			channelData_2[14] = datain[0];
			channelData_2[15] = datain[1];
			break;
	}
	// wdt_reset();
}

uint8_t volatile x = 0;

void subRefresher() 
{
	x = subRefresher_Counter;
	uint8_t tempVal[2];
	switch(x){
		case 0:
			spiTransmitADC_1(tempVal, LTC1859_CH3);
			channelUpdater_1(tempVal);
			lastChannelAccessed = LTC1859_CH3;
			x++;
			break;
		case 1:
			spiTransmitADC_2(tempVal,LTC1859_CH3);
			channelUpdater_2(tempVal);
			lastChannelAccessed_2 = LTC1859_CH3;
			x++;
			break;
		case 2:
			spiTransmitADC_1(tempVal, LTC1859_CH5);
			channelUpdater_1(tempVal);
			lastChannelAccessed = LTC1859_CH5;
			x++;
			break;
		case 3:
			spiTransmitADC_2(tempVal,LTC1859_CH7);
			channelUpdater_2(tempVal);
			lastChannelAccessed_2 = LTC1859_CH7;
			x++;
			break;
		case 4:
			spiTransmitADC_1(tempVal, LTC1859_CH7);
			channelUpdater_1(tempVal);
			lastChannelAccessed = LTC1859_CH7;
			x++;
			break;
		case 5:
			spiTransmitADC_1(tempVal, LTC1859_CH0);
			channelUpdater_1(tempVal);
			lastChannelAccessed = LTC1859_CH0;
			x++;
			break;
		case 6:
			spiTransmitADC_1(tempVal, LTC1859_CH1);
			channelUpdater_1(tempVal);
			lastChannelAccessed = LTC1859_CH1;
			x++;
			break;
		case 7:
			spiTransmitADC_2(tempVal,LTC1859_CH0);
			channelUpdater_2(tempVal);
			lastChannelAccessed_2 = LTC1859_CH0;
			x++;
			break;
		case 8:
			spiTransmitADC_2(tempVal,LTC1859_CH1);
			channelUpdater_2(tempVal);
			lastChannelAccessed_2 = LTC1859_CH1;
			x = 15;		// change to x++ if I2C enabled, else x=15 to skip i2c package.
			break;
		case 15:
			spiTransmitADC_2(tempVal,LTC1859_CH4);
			channelUpdater_2(tempVal);
			lastChannelAccessed_2 = LTC1859_CH4;
			x++;
			break;
		case 16:
			spiTransmitADC_2(tempVal,LTC1859_CH5);
			channelUpdater_2(tempVal);
			lastChannelAccessed_2 = LTC1859_CH5;
			x = 0;
			break;
	}
	subRefresher_Counter = x;
	// wdt_reset();
}

uint8_t volatile i = 0;

void refresher() 
{
    i = mainRefresher_Counter;
	uint8_t  tempAdc[2];
	switch(i){
		case 2:
			spiTransmitADC_1(tempAdc, LTC1859_CH2);
			channelUpdater_1(tempAdc);
			lastChannelAccessed = LTC1859_CH2;
			i++;
			break;
		case 3:
			spiTransmitADC_2(tempAdc,LTC1859_CH2);
			channelUpdater_2(tempAdc);
			lastChannelAccessed_2 = LTC1859_CH2;
			i++;
			break;
		case 4:
			spiTransmitADC_1(tempAdc, LTC1859_CH4);
			channelUpdater_1(tempAdc);
			lastChannelAccessed = LTC1859_CH4;
			i++;
			break;
		case 5:
			spiTransmitADC_2(tempAdc,LTC1859_CH6);
			channelUpdater_2(tempAdc);
			lastChannelAccessed_2 = LTC1859_CH6;
			i++;
			break;
		case 6:
			spiTransmitADC_1(tempAdc, LTC1859_CH6);
			channelUpdater_1(tempAdc);
			lastChannelAccessed = LTC1859_CH6;
			i++;
			break;
		case 7: 
			 subRefresher();
			i = 2;
			break;
	}
	
	mainRefresher_Counter = i;
	// wdt_reset();
}



int main(void)
{
	wdt_disable();
	// watchdog_enable();
	// Variable startups for ID and counters.
	 subComm_Counter = 0;
	 packetID = 0;
	 mainComm_Counter = 0;
	 mainRefresher_Counter = 2;
	 subRefresher_Counter = 0;
	 crc16 = 0xFFFF;
	 lastChannelAccessed = 0;
	 lastChannelAccessed_2 = 0;
	 tempData[0] = 0;
	 tempData[1] = 0;
	 //default value
	 // wdt_reset();
	for (volatile uint8_t i = 0; i<16; i++)
	{
		channelData[i] = 0xBB;
		channelData_2[i] = 0xBB;
		// wdt_reset();
	}
	for (volatile uint8_t i = 0; i<12; i++)
	{
		channelData_i2c[i] = 0xBB;
		// wdt_reset();
	}
	 
	USART_Init();
	Port_Init();
	spi_init_dac();
	// Grid voltages
	//// PCB1					   // (Grid#_bias_pcb#)
	spiTransmitDAC_1((DAC_B<<4 | GT1>>8), (uint8_t)GT1);
	spiTransmitDAC_1((DAC_D<<4 | MP>>8), (uint8_t)MP);
	spiTransmitDAC_1((DAC_C<<4 | GB1>>8), (uint8_t)GB1);
	 ////PCB2
	spiTransmitDAC_2((DAC_B<<4 | GT2>>8), (uint8_t)GT2);
	spiTransmitDAC_2((DAC_D<<4 | SP>>8), (uint8_t)SP);
	spiTransmitDAC_2((DAC_C<<4 | GB2>>8), (uint8_t)GB2);
	
	spi_init_adc();

	sei(); // enable global interrupt - run after INITS
	while(1)
	{
		refresher();
	}
}
