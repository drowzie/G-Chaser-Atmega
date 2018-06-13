/*! 
 *\file main.c	
 */

/*!
 *\author Christoffer Boothby and James Alexander Cowie
 *\version 0.4.3
 *\date 2018
 *\copyright GNU Public License.
 */

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
#include <util/twi.h>
#include <util/atomic.h>
#include <avr/wdt.h>

// A more robust way for setting baudrates-- F_CPU is set inside comm.h
#define BAUD 230400
#include <util/setbaud.h>


/* Size of Buffer*/
#define UART_BUFFER_SIZE 1024
#define UART_TX0_MAXBUFFER (UART_BUFFER_SIZE-1)


////////////////////////////////////////////////////////
//////////////////// STRUCTS ///////////////////////////
////////////////////////////////////////////////////////
typedef struct {
	uint8_t * buffer;
	uint16_t  volatile head;
	uint16_t  volatile tail;
	uint16_t  size;
} circular_buf_t;

typedef struct {
	uint8_t volatile mainComm_Counter; // Choose which packet to be sent out.
	uint8_t volatile subComm_Counter; // Choose which of the subcomms to be used and sent out.
	uint8_t volatile maxMainComms; // For CRC updater to skip when sending in CRC.
	uint16_t volatile crc16;
	uint8_t * channelData;
	uint8_t * channelData_2;
	uint8_t lastChannelAccessed;
	uint8_t lastChannelAccessed_2;
} packet_data;

	
circular_buf_t cbuf;
packet_data  pData;
uint8_t array[UART_BUFFER_SIZE];
uint8_t channels[16];
uint8_t channels_2[16];
////////////////////////////////////////////////////////
///////////////////////FUNCTIONS////////////////////////
////////////////////////////////////////////////////////

void circular_buf_put(circular_buf_t * cbuf,packet_data * pData, uint8_t  data)
{
	uint16_t tmphead;
	uint16_t txtail_tmp;
	
	
	tmphead = (cbuf->head + 1) & UART_TX0_MAXBUFFER; 
	
	do {
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			txtail_tmp = cbuf->tail;
		}
		
	} while (tmphead == txtail_tmp); /* wait for free space in buffer */
	//wdt_reset();
	cbuf->buffer[tmphead] = data;
	cbuf->head = tmphead;
	
	UCSR0B |= (1<<UDRIE0); // enable interrupt when buffer is increasing again.
	
	// Update CRC only when above ID(#2 and below CRC)
	if(pData->mainComm_Counter > 0 && pData->mainComm_Counter < pData->maxMainComms)
	{
		pData->crc16 = _crc_xmodem_update(pData->crc16, data);
	}
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
	//wdt_reset();
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
//	wdt_reset();
}

/*! \fn void USART_INIT()
 * \brief USART init of BAUD
 * \param None.
 * \return None.
*/


// Interrupt handler
ISR(USART0_UDRE_vect)	
{
	uint16_t tmptail;
	
	if(cbuf.head != cbuf.tail)
	{
		tmptail = (cbuf.tail + 1) & UART_TX0_MAXBUFFER; // Reset when reaching maximum buffer size
		cbuf.tail = tmptail;
		UDR0 = cbuf.buffer[tmptail];
		//_delay_ms(1);
	}	
	else {
		// When empty, disable the intterupt
		UCSR0B &= ~(1<<UDRIE0);
	}
	//wdt_reset();
}

// Last channel accessed, store the data received into channel data
// CH0: 0-1: CH1: 2-3 CH2: 4-5 CH3: 6-7 CH4: 8-9 CH5: 10-11CH6 12-13 CH7 14-15
void channelUpdater_1(uint8_t * datain, packet_data * pData) // if channel and store value
{
	switch (pData->lastChannelAccessed)
	{
		case LTC1859_CH0: // CH0
		pData->channelData[0] = datain[0];
		pData->channelData[1] = datain[1];		
		break;
		case LTC1859_CH1: // CH1
		pData->channelData[2] = datain[0];
		pData->channelData[3] = datain[1];
		break;
		case LTC1859_CH2: // CH2
		pData->channelData[4] = datain[0];
		pData->channelData[5] = datain[1];
		break;
		case LTC1859_CH3: // CH3
		pData->channelData[6] = datain[0];
		pData->channelData[7] = datain[1];
		break;
		case LTC1859_CH4: // CH4
		pData->channelData[8] = datain[0];
		pData->channelData[9] = datain[1];
		break;
		case LTC1859_CH5: // CH5
		pData->channelData[10] = datain[0];
		pData->channelData[11] = datain[1];
		break;
		case LTC1859_CH6: // CH6
		pData->channelData[12] = datain[0];
		pData->channelData[13] = datain[1];
		break;
		case LTC1859_CH7: // CH7
		pData->channelData[14] = datain[0];
		pData->channelData[15] = datain[1];
		break;
	}
}

void channelUpdater_2(uint8_t * datain, packet_data * pData) // if channel and store value
{
	switch (pData->lastChannelAccessed_2)
	{
		case LTC1859_CH0: // CH0
		pData->channelData_2[0] = datain[0];
		pData->channelData_2[1] = datain[1];
		break;
		case LTC1859_CH1: // CH1
		pData->channelData_2[2] = datain[0];
		pData->channelData_2[3] = datain[1];
		break;
		case LTC1859_CH2: // CH2
		pData->channelData_2[4] = datain[0];
		pData->channelData_2[5] = datain[1];
		break;
		case LTC1859_CH3: // CH3
		pData->channelData_2[6] = datain[0];
		pData->channelData_2[7] = datain[1];
		break;
		case LTC1859_CH4: // CH4
		pData->channelData_2[8] = datain[0];
		pData->channelData_2[9] = datain[1];
		break;
		case LTC1859_CH5: // CH5
		pData->channelData_2[10] = datain[0];
		pData->channelData_2[11] = datain[1];
		break;
		case LTC1859_CH6: // CH6
		pData->channelData_2[12] = datain[0];
		pData->channelData_2[13] = datain[1];
		break;
		case LTC1859_CH7: // CH7
		pData->channelData_2[14] = datain[0];
		pData->channelData_2[15] = datain[1];
		break;
	}
}


void subCommFormat(circular_buf_t * cbuf, packet_data * pData) 
{
	uint8_t x = pData->subComm_Counter;
	uint8_t tempVal[2];
	switch(x){
		case 0:
			spiTransmitADC_1(tempVal, LTC1859_CH3);
			channelUpdater_1(tempVal, pData);
			pData->lastChannelAccessed = LTC1859_CH3;
			circular_buf_put(cbuf,pData,pData->channelData[6]);
			circular_buf_put(cbuf,pData,pData->channelData[7]);
			x++;
			break;
		case 1:
			spiTransmitADC_2(tempVal,LTC1859_CH3);
			channelUpdater_2(tempVal,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH3;
			circular_buf_put(cbuf,pData,pData->channelData_2[6]);
			circular_buf_put(cbuf,pData,pData->channelData_2[7]);
			x++;
			break;
		case 2:
			spiTransmitADC_1(tempVal, LTC1859_CH5);
			channelUpdater_1(tempVal, pData);
			pData->lastChannelAccessed = LTC1859_CH5;
			circular_buf_put(cbuf,pData,pData->channelData[10]);
			circular_buf_put(cbuf,pData,pData->channelData[11]);
			x++;
			break;
		case 3:
			spiTransmitADC_2(tempVal,LTC1859_CH7);
			channelUpdater_2(tempVal,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH7;
			circular_buf_put(cbuf,pData,pData->channelData_2[14]);
			circular_buf_put(cbuf,pData,pData->channelData_2[15]);
			x++;
			break;
		case 4:
			spiTransmitADC_1(tempVal, LTC1859_CH7);
			channelUpdater_1(tempVal, pData);
			pData->lastChannelAccessed = LTC1859_CH7;
			circular_buf_put(cbuf,pData,pData->channelData[14]);
			circular_buf_put(cbuf,pData,pData->channelData[15]);
			x++;
			break;
		case 5:
			spiTransmitADC_1(tempVal, LTC1859_CH0);
			channelUpdater_1(tempVal, pData);
			pData->lastChannelAccessed = LTC1859_CH0;
			circular_buf_put(cbuf,pData,pData->channelData[0]);
			circular_buf_put(cbuf,pData,pData->channelData[1]);
			x++;
			break;
		case 6:
			spiTransmitADC_1(tempVal, LTC1859_CH1);
			channelUpdater_1(tempVal, pData);
			pData->lastChannelAccessed = LTC1859_CH1;
			circular_buf_put(cbuf,pData,pData->channelData[2]);
			circular_buf_put(cbuf,pData,pData->channelData[3]);
			x++;
			break;
		case 7:
			spiTransmitADC_2(tempVal,LTC1859_CH0);
			channelUpdater_2(tempVal,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH0;
			circular_buf_put(cbuf,pData,pData->channelData_2[0]);
			circular_buf_put(cbuf,pData,pData->channelData_2[1]);
			x++;
			break;
		case 8:
			spiTransmitADC_2(tempVal,LTC1859_CH1);
			channelUpdater_2(tempVal,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH1;
			circular_buf_put(cbuf,pData,pData->channelData_2[2]);
			circular_buf_put(cbuf,pData,pData->channelData_2[3]);
			x = 15;		// change to x++ if I2C enabled, else x=15 to skip i2c package.
			break;
			// SKIP I2C ^
		//case 9:
			//twiDataHandler(VDIG,TWIVOLT, tempVal);
			//circular_buf_put(cbuf,pData,tempVal[0]);
			//circular_buf_put(cbuf,pData,tempVal[1]);
			//x++;
			//break;
		//case 10:
			//twiDataHandler(VDIG,TWICURRENT, tempVal);
			//circular_buf_put(cbuf,pData,tempVal[0]);
			//circular_buf_put(cbuf,pData,tempVal[1]);
			//x = 15;	
			//break;
		//case 11:
			//twiDataHandler(VAPLUS,TWIVOLT, tempVal);
			//circular_buf_put(cbuf,pData,tempVal[0]);
			//circular_buf_put(cbuf,pData,tempVal[1]);
			//x++;
			//break;
		//case 12:
			//twiDataHandler(VAPLUS,TWICURRENT, tempVal);
			//circular_buf_put(cbuf,pData,tempVal[0]);
			//circular_buf_put(cbuf,pData,tempVal[1]);
			//x++;
			//break;
		//case 13:
			//twiDataHandler(VAMINUS,TWIVOLT, tempVal);
			//circular_buf_put(cbuf,pData,tempVal[0]);
			//circular_buf_put(cbuf,pData,tempVal[1]);
			//x++;
			//break;
		//case 14:
			//twiDataHandler(VAMINUS,TWICURRENT, tempVal);
			//circular_buf_put(cbuf,pData,tempVal[0]);
			//circular_buf_put(cbuf,pData,tempVal[1]);
			//x++;
			//break;
		case 15:
			spiTransmitADC_2(tempVal,LTC1859_CH4);
			channelUpdater_2(tempVal,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH4;
			circular_buf_put(cbuf,pData,pData->channelData_2[8]);
			circular_buf_put(cbuf,pData,pData->channelData_2[9]);
			x++;
			break;
		case 16:
			spiTransmitADC_2(tempVal,LTC1859_CH5);
			channelUpdater_2(tempVal,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH5;
			circular_buf_put(cbuf,pData,pData->channelData_2[10]);
			circular_buf_put(cbuf,pData,pData->channelData_2[11]);
			x = 0;
			break;
	}
	pData->subComm_Counter = x;

}
/*! \fn void subCommFormat() 
 * \brief subcomm packet, will repeat itself
 * \param cbuf Use the buffer.
 * \param pData For using subcomm ID
 * \return None.
*/


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
			spiTransmitADC_1(tempAdc, LTC1859_CH2);
			channelUpdater_1(tempAdc, pData);
			pData->lastChannelAccessed = LTC1859_CH2;
			circular_buf_put(cbuf,pData,pData->channelData[4]);
			circular_buf_put(cbuf,pData,pData->channelData[5]);
			i++;
			break;
		case 3:
			spiTransmitADC_2(tempAdc,LTC1859_CH2);
			channelUpdater_2(tempAdc,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH2;
			circular_buf_put(cbuf,pData,pData->channelData_2[4]);
			circular_buf_put(cbuf,pData,pData->channelData_2[5]);
			i++;
			break;
		case 4:
			spiTransmitADC_1(tempAdc, LTC1859_CH4);
			channelUpdater_1(tempAdc, pData);
			pData->lastChannelAccessed = LTC1859_CH4;
			circular_buf_put(cbuf,pData,pData->channelData[8]);
			circular_buf_put(cbuf,pData,pData->channelData[9]);
			i++;
			break;
		case 5:
			spiTransmitADC_2(tempAdc,LTC1859_CH6);
			channelUpdater_2(tempAdc,pData);
			pData->lastChannelAccessed_2 = LTC1859_CH6;
			circular_buf_put(cbuf,pData,pData->channelData_2[12]);
			circular_buf_put(cbuf,pData,pData->channelData_2[13]);
			i++;
			break;
		case 6:
			spiTransmitADC_1(tempAdc, LTC1859_CH6);
			channelUpdater_1(tempAdc, pData);
			pData->lastChannelAccessed = LTC1859_CH6;
			circular_buf_put(cbuf,pData,pData->channelData[12]);
			circular_buf_put(cbuf,pData,pData->channelData[13]);
			i++;
			break;
		case 7: // SUBCOMM
			 subCommFormat(cbuf,pData);
			//circular_buf_put(cbuf,pData,0xEE);
			//circular_buf_put(cbuf,pData,0x99);
			i++;
			break;
		case 8: // CRC
			circular_buf_put(cbuf,pData,(pData->crc16>>8));
			circular_buf_put(cbuf,pData,(uint8_t)pData->crc16);
			pData->crc16 = 0xFFFF; // reset when done
			i = 0;
			break;
	}
	pData->mainComm_Counter = i;
}

/*! \fn void packetFormat() 
 * \brief Main packet format, will repeat itself
 * \param cbuf Use the buffer.
 * \param pData For using subcomm ID
 * \return None.
*/

int main(void)
{
	 //watchdog_enable(); // enable watchdog timer System reset
	// Struct defines
	 cbuf.buffer = array;
	 cbuf.size = UART_BUFFER_SIZE;
	 cbuf.tail = 0;
	 cbuf.head = 0;
	 pData.mainComm_Counter = 0;
	 pData.maxMainComms = 8;
	 pData.crc16 = 0xFFFF;
	 pData.channelData = channels;
	 pData.channelData_2 = channels_2;
	 pData.lastChannelAccessed = 0;
	 pData.lastChannelAccessed_2 = 0;
	USART_Init();
	Port_Init();
	// For testing one I2C channel
#pragma region i2cTEST
	 	//i2c_init();
	 	//uint8_t twiTemp[2];
	 	//while(1)
	 	//{
			//twiDataHandler(0xD2, TWICURRENT, twiTemp);
		 	//UDR0 = twiTemp[0];
		 	//while ( !( UCSR0A & (1<<UDRE0)) );
		 	//UDR0 = twiTemp[1];
		 	//while ( !( UCSR0A & (1<<UDRE0)) );
		 	//_delay_ms(10);
	 	//}
		 
#pragma endregion
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
//	i2c_init();
	//// For testing one ADC channel
#pragma region TestADC
	////uint8_t testData[2];
	//while(1) 
	//{
		//spiTransmitADC_2(testData,LTC1859_CH1);
		//UDR0 = 0x9C;
		//while ( !( UCSR0A & (1<<UDRE0)) ){}
		//UDR0 = testData[0];
		//while ( !( UCSR0A & (1<<UDRE0)) ){}
		//UDR0 = testData[1];
		//while ( !( UCSR0A & (1<<UDRE0)) ){}
	//}
#pragma endregion	
	sei(); // enable global interrupt - run after INITS
	while(1)
	{
		packetFormat(&cbuf,&pData);
		wdt_reset();
	}
}
