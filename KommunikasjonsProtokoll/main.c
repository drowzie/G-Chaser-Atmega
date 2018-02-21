/*
 * KommunikasjonsProtokoll.c
 *
 * Author : chris
 */ 
#include <stdint.h>
#include <avr/io.h>
#include "comm.h"
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <stdbool.h>
#include <stdlib.h>

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

// UNDER MÅ kombineres med LTC1859 defines.... ADDR + INP + POWDWN -> 8bits data ord. (0000) + (00) + (00)...etc

#define inputRange				0x03 // feil verdier.. må endres
#define PowerDownSel			0x01 // 

// SPI defines for DAC8420
// Må endres for riktig data størrelse 
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

// Some other defines

// variables
// 	uint8_t EEMEM k[304]; // 512 bytes

 	uint16_t crc16;
	 
	 
// Circular Buffer
typedef struct {
	uint8_t * buffer;
	size_t head;
	size_t tail;
	size_t size; 
} circular_buf_t;

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

// Init function
	uint8_t UpperSync;
	uint8_t lowerSync;
int main(void) {
	circular_buf_t cbuf;
	cbuf.size = 5;
	cbuf.buffer = malloc(cbuf.size); // Malloc returns a pointer to allocated memory. or NULL if it fails.
	crc16 = 0xFFFF; // Start value of CRC16
	
	uint16_t Synkeord = 0xABCD;

	UpperSync = Synkeord & 0x00FF;
	
	circular_buf_put(&cbuf, UpperSync);

	
// 	circular_buf_put(&cbuf, upper);
// 	circular_buf_put(&cbuf, lower);

// 	crc16 = _crc_ccitt_update(crc16, 'a');
// 	circular_buf_put(&cbuf, 'a');
// 
// 	uint8_t data;
// 	
// 	circular_buf_get(&cbuf, &data);


	 //premade in atmel studio, ccitt update will update its values everytime the data is added.
	 //use http://www.sunshine2k.de/coding/javascript/crc/crc_js.html to test crc
	
	
return 0;
}
