/*
Author: Christoffer Boothby
Version: 0.0.0.1
Comments:
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
// A more robust way for setting baudrates-- F_CPU is set inside comm.h
#define BAUD 1200
#include <util/setbaud.h>


// FOR EEEPROM STORAGE //
// #include <avr/eeprom.h>

// Storing to EEPROM functions if necessary
//#define read_eeprom_word(address) eeprom_read_word ((const uint16_t*)address)
//#define write_eeprom_word(address,value) eeprom_write_word ((uint16_t*)address,(uint16_t)value)
//#define update_eeprom_word(address,value) eeprom_update_word ((uint16_t*)address,(uint16_t)value)


typedef struct {
	uint8_t volatile * buffer;
	size_t  head;
	size_t  tail;
	size_t  size;
} circular_buf_t;

typedef struct {
	uint8_t volatile mainComm_Counter; // Choose which packet to be sent out.
	uint8_t volatile subComm_Counter; // Choose which of the subcomms to be used and sent out.
	uint8_t volatile maxMainComms; // For CRC updater to skip when sending in CRC.
	uint16_t crc16;
} packet_data;

circular_buf_t cbuf;
packet_data  pData;
uint8_t tempADC[2];
uint8_t array[51];

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// FUNCTIONS BELOW HERE //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Truth statements for the circular buffer
bool circular_buf_empty(circular_buf_t cbuf)
{
	// We define empty as head == tail
	return (cbuf.head == cbuf.tail);
}

bool circular_buf_full(circular_buf_t cbuf)
{
	return ((cbuf.head + 1) % cbuf.size) == cbuf.tail;
}

// Put 8 bit data into cbuf buffer
void circular_buf_put(circular_buf_t * cbuf,packet_data * pData, uint8_t  data)
{
	cbuf->buffer[cbuf->head] = data; // Data added
	cbuf->head = (cbuf->head + 1) % cbuf->size; // Increase head and resets on 24%24 => 0
	
	// CRC UPDATER
	//if(!(pData->mainComm_Counter == pData->maxMainComms-1)) // Do not enter when CRC(Max main comm has been reached)
	//{
		//pData->crc16 = _crc_ccitt_update(pData->crc16, data); // Update the crc word.
	//}
}

int circular_buf_get(circular_buf_t * cbuf)
{
	uint8_t data;
	data = cbuf->buffer[cbuf->tail];
	cbuf->tail = (cbuf->tail + 1) % cbuf->size; // Tail resets when it reaches buffer size.
	return data;
}


#define SCL         PORTC5
#define SDA         PORTC4

void Port_Init()
{
	// Datadirections only, to set, use PORTxn, 0 = Input, 1 = Output
	// DDRB = (0<<ADC_2_BUSY)|(1<<CS_DAC_2)|(1<<LD_DAC_2);
	DDRC = (1<<CS_DAC_1)|(1<<LD_DAC_1)|(0<<ADC_1_BUSY);
	DDRD = (1<<ADC_READ_2)|(1<<ADV_CONVERSION_START_2);
	DDRE = (1<<ADC_READ_1)|(1<<ADV_CONVERSION_START_1);

	// Start configurations
	PORTE |= (1<<ADC_READ_1); // set default high
	PORTE &= ~(1<<ADV_CONVERSION_START_1); // Default low
	
	PORTC |= (1<<LD_DAC_1)|(1<<CS_DAC_1);
	
	PORTB |= (1<<LD_DAC_2);
}
void USART_Init()
{
	/* Set baud rate */
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	
	/* Enable transmitter */
	UCSR0B = (1<<TXEN0)|(1<<UDRIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}

/*
subCommPacket is the main method to circle through different HOUSEKEEPING channels.
Should be over 20 different channels.
*/

#pragma region subcomm
void subCommPacket(circular_buf_t * cbuf, packet_data * pData)
{
	if(pData->subComm_Counter == 0) {
		circular_buf_put(cbuf,pData, 0xB0);
		pData->subComm_Counter++;	
	} else if(pData->subComm_Counter == 1){
		circular_buf_put(cbuf,pData, 0xB1);
		pData->subComm_Counter++;
	} else if(pData->subComm_Counter == 2){
		circular_buf_put(cbuf,pData, 0xB2);
		pData->subComm_Counter++;
	} else if(pData->subComm_Counter == 3) {
		circular_buf_put(cbuf,pData, 0xB3);
		pData->subComm_Counter++;
	} else if(pData->subComm_Counter == 4) {
		circular_buf_put(cbuf,pData, 0xB4);
		pData->subComm_Counter++;
	} else if(pData->subComm_Counter == 5) {
		circular_buf_put(cbuf,pData, 0xB5);
		pData->subComm_Counter++;
	} else if(pData->subComm_Counter == 6) {
		circular_buf_put(cbuf,pData, 0xB6);
		pData->subComm_Counter++;
	} else if(pData->subComm_Counter == 7) {
		circular_buf_put(cbuf,pData, 0xB7);
		pData->subComm_Counter++;
	} else if(pData->subComm_Counter == 8) {
		circular_buf_put(cbuf,pData, 0xB8);
		pData->subComm_Counter = 0;
	}
}

#pragma endregion

ISR(USART0_UDRE_vect)
{
	UDR0 = circular_buf_get(&cbuf);
}

int main(void)
{
	// Struct defines
	cbuf.buffer = array;
	cbuf.size = 51;
	pData.mainComm_Counter = 0;
	pData.subComm_Counter = 0;
	pData.crc16 = 0xFFFF; // INITIAL CRC word	
	 //When UART interrupt is turned on, it will instantly interrupt.
	 //To prevent it from being stuck inside the loop. Fill the buffer with startup info.
	for(int i = 0; i < 4; i++)
	{
	circular_buf_put(&cbuf,&pData,0xEE);
	}
	
	USART_Init();
	Port_Init();
#pragma region ADCTEST
	//spi_init_adc();
	//uint8_t dataOut[2];
	//uint8_t dataIn;
	//dataIn = LTC1859_CH2;
	//while(1)
	//{
		//spiTransmitADC_1(dataOut,dataIn);
		//UDR0 = 0xdF;
		//while ( !( UCSR0A & (1<<UDRE0)) );
		//UDR0 = dataOut[0];
		//while ( !( UCSR0A & (1<<UDRE0)) );
		//UDR0 = dataOut[1];
		//while ( !( UCSR0A & (1<<UDRE0)) );
	//}
#pragma endregion 

#pragma region I2CTest Commented out
//i2c_init();
//
//uint8_t recievedData[2];
//uint8_t status;
//while(1){
	//
	//status = i2c_start(0xD2|0x00);
	//status = i2c_write(0x01);
	//status = i2c_start(0x01|0x01);
	//status = i2c_read_ack();
	//i2c_stop();
	//
	//UDR0 = status;
	//while ( !( UCSR0A & (1<<UDRE0)) );

//}
#pragma endregion

#pragma region Set Grid Voltages

	spi_init_dac();
	// PCB1
	spiTransmitDAC_1((DAC_B<<4 | 0xF), 0xFF);
	spiTransmitDAC_1((DAC_C<<4 | 0xF), 0xFF);
	spiTransmitDAC_1((DAC_D<<4 | 0xF), 0xFF);
	// PCB2
	
#pragma endregion

	///* Replace with your application code */
	spi_init_adc(); 
 	sei();
	while(1)
	{
		if (circular_buf_full(cbuf)){} 
		else
		{
			if(pData.mainComm_Counter == 0)
			pData.mainComm_Counter) {
				case 0: // SYNC
					pData.crc16 = 0xFFFF;
					circular_buf_put(&cbuf, &pData, 0x6B);
					circular_buf_put(&cbuf, &pData, 0x90);
					pData.mainComm_Counter++;
					break;
				case 1:
					circular_buf_put(&cbuf, &pData, pData.mainComm_Counter);
					pData.mainComm_Counter++;
					break;
				case 2:
					spiTransmitADC_1(tempADC,LTC1859_CH2);
					circular_buf_put(&cbuf, &pData, tempADC[0]);
					circular_buf_put(&cbuf, &pData, tempADC[1]);
					pData.mainComm_Counter = 2;
					break;
			}
		}
	}
}
