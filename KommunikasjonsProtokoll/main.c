/*
 * KommunikasjonsProtokoll.c
 *
 * Created: 05.02.2018 12.07.27
 * Author : chris
 */ 
#include <stdint.h>
#include <avr/io.h>
#include "comm.h"

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


int main(void)
{
    /* Replace with your application code */
	spi_init();	
	i2c_start(U7_ADDR);
	
return 0;
}
