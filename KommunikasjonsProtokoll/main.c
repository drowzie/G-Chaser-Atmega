/*! 
 *\file main.c	
 *\
 *\
 */

/*! \mainpage My Personal Index Page
 *
 *\section intro_sec Introduction
 *\author Christoffer Boothby
 *\version 0.0.1.2
 *\date 2018
 *\copyright GNU Public License.
 *Main program for the G-Chaser Project using Atmega328PB
 *
 * \section install_sec Installation
 *
 *
 * \subsection step1 Step 1: Opening the bxo
 *
 * etc...
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
#include <util/twi.h>
#include <util/atomic.h>

// A more robust way for setting baudrates-- F_CPU is set inside comm.h
#define BAUD 1200
#include <util/setbaud.h>

// FOR EEEPROM STORAGE //
// #include <avr/eeprom.h>

// Storing to EEPROM functions if necessary
//#define read_eeprom_word(address) eeprom_read_word ((const uint16_t*)address)
//#define write_eeprom_word(address,value) eeprom_write_word ((uint16_t*)address,(uint16_t)value)
//#define update_eeprom_word(address,value) eeprom_update_word ((uint16_t*)address,(uint16_t)value)

/* Size of Buffer*/
#define UART_BUFFER_SIZE 128
#define UART_TX0_MAXBUFFER (UART_BUFFER_SIZE-1)


////////////////////////////////////////////////////////
//////////////////// STRUCTS ///////////////////////////
////////////////////////////////////////////////////////
typedef struct {
	uint8_t * buffer;
	uint8_t  volatile head;
	uint8_t  volatile tail;
	uint8_t  size;
} circular_buf_t;

typedef struct {
	uint8_t volatile mainComm_Counter; // Choose which packet to be sent out.
	uint8_t volatile subComm_Counter; // Choose which of the subcomms to be used and sent out.
	uint8_t volatile maxMainComms; // For CRC updater to skip when sending in CRC.
	uint16_t crc16;
} packet_data;

circular_buf_t cbuf;
packet_data  pData;
uint8_t array[UART_BUFFER_SIZE];

////////////////////////////////////////////////////////
///////////////////////FUNCTIONS////////////////////////
////////////////////////////////////////////////////////


void circular_buf_put(circular_buf_t * cbuf,packet_data * pData, uint8_t  data)
{
	uint16_t tmphead;
	
	tmphead = (cbuf->head + 1) & UART_TX0_MAXBUFFER; 
	
	while (tmphead == cbuf->tail); /* wait for free space in buffer */
	cbuf->buffer[tmphead] = data;
	cbuf->head = tmphead;
	
	UCSR0B |= (1<<UDRIE0); // enable interrupt when buffer is increasing again.
}

/*! \fn void circular_buf_put(circular_buf_t * cbuf,packet_data * pData, uint8_t  data)
 * \brief Putting 1 byte into the buffer
 * \param[in] data The data that goes into the buffer.
 * \param[in] cbuf refrence to the circular buffer.
 * \param[in] pdata refrence for pdata.
 * \return None.
*/


void Port_Init()
{
	// Datadirections only, to set, use PORTxn, 0 = Input, 1 = Output
	DDRB = (0<<ADC_2_BUSY)|(1<<CS_DAC_2)|(1<<LD_DAC_2);
	DDRC = (1<<CS_DAC_1)|(1<<LD_DAC_1)|(0<<ADC_1_BUSY);
	DDRD = (1<<ADC_READ_2)|(1<<ADV_CONVERSION_START_2);
	DDRE = (1<<ADC_READ_1)|(1<<ADV_CONVERSION_START_1);

	// Start configurations
	
	// ADC STARTUP PIN CONFIGURATIONS
	PORTD = (1<<ADC_READ_2)|(0<<ADV_CONVERSION_START_2);
	PORTE = (1<<ADC_READ_1)|(0<<ADV_CONVERSION_START_1);
	// DAC STARTUP PIN CONFIGURATION
	PORTC |= (1<<LD_DAC_1)|(1<<CS_DAC_1);
	PORTB = (1<<LD_DAC_2)|(1<<CS_DAC_2);
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
	UCSR0B = (1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}

/*! \fn void USART_INIT()
 * \brief USART init of BAUD
 * \param None.
 * \return None.
*/


ISR(USART0_UDRE_vect)	
{
	uint16_t tmptail;
	
	if(cbuf.head != cbuf.tail)
	{
		tmptail = (cbuf.tail + 1) & UART_TX0_MAXBUFFER;
		cbuf.tail = tmptail;
		UDR0 = cbuf.buffer[tmptail];
	}	
	else {
		// When empty, disable the intterupt
		UCSR0B &= ~(1<<UDRIE0);
	}
	
}

void subCommFormat(circular_buf_t * cbuf, packet_data * pData) 
{
	uint8_t x = pData->subComm_Counter;
	uint8_t tempVal[2];
	switch(x){
		case 0:
			spiTransmitADC_1(tempVal,LTC1859_CH3);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 1:
			spiTransmitADC_2(tempVal,LTC1859_CH3);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 2:
			spiTransmitADC_1(tempVal,LTC1859_CH5);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 3:
			spiTransmitADC_2(tempVal,LTC1859_CH7);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 4:
			spiTransmitADC_1(tempVal,LTC1859_CH7);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 5:
			spiTransmitADC_1(tempVal,LTC1859_CH0);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 6:
			spiTransmitADC_1(tempVal,LTC1859_CH1);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 7:
			spiTransmitADC_2(tempVal,LTC1859_CH0);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x++;
			break;
		case 8:
			spiTransmitADC_2(tempVal,LTC1859_CH1);
			circular_buf_put(cbuf,pData,tempVal[0]);
			circular_buf_put(cbuf,pData,tempVal[1]);
			x = 0;
			break;		
	}
	pData->subComm_Counter = x;
}

void packetFormat(circular_buf_t * cbuf,packet_data * pData) 
{
	uint8_t i = pData->mainComm_Counter;
	uint8_t tempAdc[2];
	switch(i){
		case 0: // SYNC
			circular_buf_put(cbuf,pData,0x6B);
			circular_buf_put(cbuf,pData,0x90);
			i++;
			break;
		case 1:
			circular_buf_put(cbuf,pData,pData->subComm_Counter);
			i++;
			break;
		case 2:
			spiTransmitADC_1(tempAdc,LTC1859_CH2);
			circular_buf_put(cbuf,pData,tempAdc[0]);
			circular_buf_put(cbuf,pData,tempAdc[1]);
			i++;
			break;
		case 3:
			spiTransmitADC_2(tempAdc,LTC1859_CH2);
			circular_buf_put(cbuf,pData,tempAdc[0]);
			circular_buf_put(cbuf,pData,tempAdc[1]);
			i++;
			break;
		case 4:
			spiTransmitADC_1(tempAdc,LTC1859_CH4);
			circular_buf_put(cbuf,pData,tempAdc[0]);
			circular_buf_put(cbuf,pData,tempAdc[1]);
			i++;
			break;
		case 5:
			spiTransmitADC_2(tempAdc,LTC1859_CH6);
			circular_buf_put(cbuf,pData,tempAdc[0]);
			circular_buf_put(cbuf,pData,tempAdc[1]);
			i++;
			break;
		case 6:
			spiTransmitADC_1(tempAdc,LTC1859_CH6);
			circular_buf_put(cbuf,pData,tempAdc[0]);
			circular_buf_put(cbuf,pData,tempAdc[1]);
			i++;
			break;
		case 7: // SUBCOMM
			subCommFormat(cbuf,pData);
			i++;
			break;
		case 8: // CRC
			spiTransmitADC_1(tempAdc,LTC1859_CH6);
			circular_buf_put(cbuf,pData,tempAdc[0]);
			circular_buf_put(cbuf,pData,tempAdc[1]);
			i = 0;
			break;
	}
	pData->mainComm_Counter = i;
}

uint8_t testvalue;

int main(void)
{
	// Struct defines
	cbuf.buffer = array;
	cbuf.size = UART_BUFFER_SIZE;
	cbuf.tail = 0;
	cbuf.head = 0;
	pData.mainComm_Counter = 0;
	pData.crc16 = 0xFFFF;


	USART_Init();
	Port_Init();
	
#pragma region i2cTEST
	//i2c_init();
	//uint8_t volatile status;
	//while(1)
	//{
		//status = i2c_start(0xCE|0x00);
		//status = i2c_write(0x01);
		//status = i2c_start(0xCE|0x01);
		//status = i2c_read_ack();
		//i2c_stop();
		//UDR0 = status;
		//while ( !( UCSR0A & (1<<UDRE0)) );
		//_delay_ms(100);
		//
	//}
#pragma endregion

#pragma region SetGridVoltages

	spi_init_dac();
	// PCB1
	spiTransmitDAC_1((DAC_B<<4 | G1_BIAS_1>>8), (uint8_t)G1_BIAS_1);
	spiTransmitDAC_1((DAC_C<<4 | G2_BIAS_1>>8), (uint8_t)G2_BIAS_1);
	spiTransmitDAC_1((DAC_D<<4 | G3_BIAS_1>>8), (uint8_t)G3_BIAS_1);
	// PCB2
	spiTransmitDAC_2((DAC_B<<4 | G1_BIAS_2>>8), (uint8_t)G1_BIAS_2);
	spiTransmitDAC_2((DAC_C<<4 | G2_BIAS_2>>8), (uint8_t)G2_BIAS_2);
	spiTransmitDAC_2((DAC_D<<4 | G3_BIAS_2>>8), (uint8_t)G3_BIAS_2);
	
#pragma endregion
	spi_init_adc();
	uint8_t testData[2];
	while(1) {
		
		spiTransmitADC_1(testData,LTC1859_CH0);
		UDR0 = testData[0];
		while (!( UCSR0A & (1<<UDRE0)));
		UDR0 = testData[1];
		while (!( UCSR0A & (1<<UDRE0)));
		_delay_ms(1);
	}
	
	
	//sei(); // enable global interrupt - run after INITS
	//while(1)
	//{
		//packetFormat(&cbuf,&pData);
	//}
}
