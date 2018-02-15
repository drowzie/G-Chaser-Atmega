/*
 * KommunikasjonsProtokoll.c
 *
 * Created: 05.02.2018 12.07.27
 * Author : chris
 */ 
#include <stdint.h>
#include <avr/io.h>
#include "comm.h"
#include <util/crc16.h>
#include <avr/eeprom.h>

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

// UNDER MÅ kombineres med LTC1859 defines.... ADDR + INP + POWDWN -> 8bits data ord.
#define inputRange				0x03 // b11 - 0V-10V or b01 +- 10v
#define PowerDownSel			0x01 // Nap

// SPI defines for DAC8420
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

// shows an example of using EEPROM rather than memory
#define read_eeprom_word(address) eeprom_read_word ((const uint16_t*)address)
#define write_eeprom_word(address,value) eeprom_write_word ((uint16_t*)address,(uint16_t)value)
#define update_eeprom_word(address,value) eeprom_update_word ((uint16_t*)address,(uint16_t)value)

	uint8_t EEMEM k[200]; // 512 bytes
	uint16_t EEMEM crc16;
int main(void) {
	spi_init();
	i2c_start(U7_ADDR);
	
	// Premade in atmel studio, ccitt update will update its values everytime the data is added.
	// use http://www.sunshine2k.de/coding/javascript/crc/crc_js.html to test CRC
	write_eeprom_word(crc16,0xFFFF);
	
	int i;
	// Fill up the array with 512 times 0xFF
	for (i=0;i<512;i++) write_eeprom_word(&k[i], 0xF0);
	// Calculate and update the CRC
	for (i=0;i<512;i++) write_eeprom_word(crc16, _crc_ccitt_update(read_eeprom_word(crc16), read_eeprom_word(&k[i])));
	
	
return 0;
}
