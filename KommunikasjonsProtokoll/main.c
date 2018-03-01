/*
 * KommunikasjonsProtokoll.c
 * 
 * Author : chris
 */ 

// UART settings
#define FOSC 16000000
#define F_CPU 16000000
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


#include <stdint.h>
#include <avr/io.h>
#include "comm.h"
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/interrupt.h>

// Ports
// SPI PORTS ADC:

#define ADV_CONVST_2			DDD6	// CONVST 1 and READ 1 might have issues considering they are on pin ADC6 and 7...
								

#define ADC_READ_2				DDD7

#define ADC_1_BUSY				DDC0
#define ADC_2_BUSY				DDB0
	
// DAC PORTS:
#define CS_DAC_1				DDC1
#define LD_DAC_1				DDC2

#define CS_DAC_2				DDB1
#define LD_DAC_2				DDB2 // SPI slave select, when SPI is set as master: DDB2 controls the direction.


// SPI Defines for LTC1859
// Single-Ended Channel Address
#define LTC1859_CH0             0x80
#define LTC1859_CH1             0xC0
#define LTC1859_CH2             0x90
#define LTC1859_CH3             0xD0
#define LTC1859_CH4             0xA0
#define LTC1859_CH5             0xE0
#define LTC1859_CH6             0xB0
#define LTC1859_CH7             0xF0

// UNDER M� kombineres med LTC1859 defines.... ADDR + INP + POWDWN -> 8bits data ord. (0000) + (00) + (00)...etc
#define inputRange				0x3 // feil verdier.. m� endres
#define PowerDownSel			0x1 // 

// SPI defines for DAC8420
// M� endres for riktig data st�rrelse 
#define DAC_A					0x0
#define DAC_B					0x4
#define DAC_C					0x8
#define DAC_D					0xC

// I2C Devices
#define U7_ADDR					0xD2		
#define U8_ADDR					0xCE
#define U9_ADDR					0xDE

// I2C Register adresses
#define sense_A					0x00
#define sense_B					0x01
#define vIn_MSB					0x02
#define vIn_LSB					0x03
#define adIn_MSB				0x04
#define adIn_LSB				0x05
#define control					0x06

// Storing to EEPROM functions if necesarry
#define read_eeprom_word(address) eeprom_read_word ((const uint16_t*)address)
#define write_eeprom_word(address,value) eeprom_write_word ((uint16_t*)address,(uint16_t)value)
#define update_eeprom_word(address,value) eeprom_update_word ((uint16_t*)address,(uint16_t)value)
	 
// Circular Buffer
typedef struct {
	uint8_t * buffer;
	size_t head;
	size_t tail;
	size_t size; 
} circular_buf_t;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// FUNCTIONS BELOW HERE //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Reset the circular buffer
int circular_buf_reset(circular_buf_t * cbuf)
{
	int r = -1;

	if(cbuf)
	{
		cbuf->head = 0;
		cbuf->tail = 0;
		r = 0;
	}

	return r;
}

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
// Puts 8 bit data into cbuf buffer 
int circular_buf_put(circular_buf_t * cbuf, uint8_t data)
{
	int r = -1;

	if(cbuf)
	{
		cbuf->buffer[cbuf->head] = data;
		cbuf->head = (cbuf->head + 1) % cbuf->size;

		if(cbuf->head == cbuf->tail)
		{
			cbuf->tail = (cbuf->tail + 1) % cbuf->size;
		}

		r = 0;
	}

	return r;
}
int circular_buf_get(circular_buf_t * cbuf, uint8_t * data)
{
	int r = -1;

	if(cbuf && data && !circular_buf_empty(*cbuf))
	{
		*data = cbuf->buffer[cbuf->tail];
		cbuf->tail = (cbuf->tail + 1) % cbuf->size;

		r = 0;
	}

	return r;
}

// Set direction of ports

void Port_Init()
{
	// Datadirections only, to set, use PORTxn
	DDRB = (0<<ADC_2_BUSY)|(1<<CS_DAC_2)|(1<<LD_DAC_2);
	DDRC = (0<<ADC_1_BUSY)|(1<<CS_DAC_2)|(1<<LD_DAC_1);
	DDRD = (1<<ADV_CONVST_2)|(1<<ADC_READ_2);
}

// UART MODULE
void USART_Init( unsigned int ubrr)
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	/* Enable receiver and transmitter */
	UCSR0B = (1<<TXEN0)|(1<<UDRIE0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

// Variables 
	uint16_t crc16;
	circular_buf_t cbuf;

	
// INTERRUPT FUNCTION
ISR(USART0_UDRE_vect)
{
	// UDR0 = 'A'; data to send
}
	
// ############################ MAIN #######################//	
int main(void) {
	
	int * array[255];
	/////////INITS///////
	// Declare the circular buffer struct with size 5.
	cbuf.size = 255;
	cbuf.buffer = &array;   //malloc(cbuf.size); // Malloc returns a pointer to allocated memory. or NULL if it fails. Takes memory from heap in runtime.
	crc16 = 0xFFFF; // Start value of CRC16
	
	spi_init();		// SPI before Port init so that the SS is properly configured.
	Port_Init();
	
	
	
	
	
	sei();							// Interrupt
	USART_Init(MYUBRR);
	
	while(1)
	{
		
	}


	 //premade in atmel studio, ccitt update will update its values everytime the data is added.
	 //use http://www.sunshine2k.de/coding/javascript/crc/crc_js.html to test crc
	
	
return 0;
}
