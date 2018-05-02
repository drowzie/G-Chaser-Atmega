/*
 * KommunikasjonsProtokoll.c
 * 
 * Author : chris
 */ 

// UART settings
#include "comm.h"
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/crc16.h>
#include <avr/wdt.h> 

// A more robust way for setting baudrates-- F_CPU is set inside comm.h
#define BAUD 9600
#include <util/setbaud.h>

#define SYNC 0x6B90

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

// Increment struct aswell as CRC16 updates.
typedef struct {
	uint8_t volatile mainComm_Counter; // Choose which packet to be sent out.
	uint8_t volatile subComm_Counter; // Choose which of the subcomms to be used and sent out.
	uint8_t volatile maxMainComms; // For CRC updater to skip when sending in CRC. 
	uint16_t crc16;
} packet_data;

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
void circular_buf_put(circular_buf_t * cbuf,packet_data * pData, uint8_t data)
{
	cbuf->buffer[cbuf->head] = data; // Data added
	cbuf->head = (cbuf->head + 1) % cbuf->size; // Increase head and resets on 24%24 => 0
	
	// CRC UPDATER
	if(!(pData->mainComm_Counter == pData->maxMainComms-1))
	{
		pData->crc16 = _crc_ccitt_update(pData->crc16, data);
	}
}

int circular_buf_get(circular_buf_t * cbuf)
{
	uint8_t data;
	data = cbuf->buffer[cbuf->tail];
	cbuf->tail = (cbuf->tail + 1) % cbuf->size; // Tail resets when it reaches buffer size.
	return data;
}

// Set direction of ports

void Port_Init()
{
	// Datadirections only, to set, use PORTxn, 0 = Input, 1 = Output
	DDRB = (0<<ADC_2_BUSY)|(1<<CS_DAC_2)|(1<<LD_DAC_2);
	DDRC = (0<<ADC_1_BUSY)|(1<<CS_DAC_2)|(1<<LD_DAC_1)|(1<<ADV_CONVERSION_START_1);
	DDRD = (1<<ADC_READ_2)|(1<<ADV_CONVERSION_START_2);
	DDRE = (1<<ADC_READ_1);
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

circular_buf_t cbuf;
uint8_t array[51];

// Interrupt function for UART when ready
ISR(USART0_UDRE_vect)
{
	UDR0 = 	circular_buf_get(&cbuf);;
}

void subCommPacket(circular_buf_t * cbuf, packet_data * pData)
{
	if(pData->subComm_Counter == 0)
	{
		circular_buf_put(cbuf,pData, 0x33);
		pData->subComm_Counter++;
		
	} else if(pData->subComm_Counter == 1)
	{
		circular_buf_put(cbuf,pData, 0x34);
		pData->subComm_Counter = 0;
	}
}

int main(void)
{
	// Struct defines
	cbuf.buffer = array;
	cbuf.size = 51;
	packet_data pData;
	pData.mainComm_Counter = 0;
	pData.subComm_Counter = 0;
	pData.maxMainComms = 5;
	pData.crc16 = 0xFFFF; // INITIAL CRC word
	
	// STARTUP operations
	circular_buf_put(&cbuf,&pData,0xAA); // Initial value so bool full doesnt fuck up.
	USART_Init();
	
	/* Replace with your application code */
	sei();
	while(1)
	{
		
		if(circular_buf_full(cbuf))
		{
			// Do nothing when full
		}
		// PACKET FORMAT
		else if(pData.mainComm_Counter == 0) // SYNC FIRST TWO
		{
			pData.crc16 = 0xFFFF;
			circular_buf_put(&cbuf, &pData, (SYNC>>8));
			circular_buf_put(&cbuf, &pData, SYNC);
			pData.mainComm_Counter++;
			
			} else if (pData.mainComm_Counter == 1) { // PACKETID
				
			circular_buf_put(&cbuf, &pData, pData.subComm_Counter);
			
			pData.mainComm_Counter++;
			} else if (pData.mainComm_Counter == 2) { // CH0
				
			circular_buf_put(&cbuf, &pData, 0xCD);
			pData.mainComm_Counter++;
			
			} else if (pData.mainComm_Counter == 3) { // SUBCOMM
				
			subCommPacket(&cbuf, &pData);
			pData.mainComm_Counter++;
			
			} else if (pData.mainComm_Counter == 4) { // CRC packet
				
			circular_buf_put(&cbuf, &pData, (pData.crc16>>8)); // upper 8 bits
			circular_buf_put(&cbuf, &pData, (pData.crc16)); // lower 8 bits
			pData.mainComm_Counter = 0;
			
		}
	}
}

