/*
 * comm.h
 *	
 * 
 */ 

#pragma once

#ifndef COMM_H_
#define COMM_H_
#include <stdint.h>

#define F_CPU (14745600UL) // Need to be changed when deployed on our own

// Ports
// SPI PORTS ADC:
#define ADV_CONVERSION_START_1			DDRE2	// Should be correct
#define ADV_CONVERSION_START_2			DDRB6	// CONVST 1 and READ 1 might have issues considering they are on pin ADC6 and 7...

#define ADC_READ_1				DDRE3	//  Using port E for ADC_READ_1
#define ADC_READ_2				DDD7	

#define ADC_1_BUSY				DDRC0
#define ADC_2_BUSY				DDRB0

// DAC PORTS:
#define CS_DAC_1				DDRC1				
#define LD_DAC_1				DDRC2

#define CS_DAC_2				DDRB1
#define LD_DAC_2				DDRB2 // SPI slave select, when SPI is set as master: DDB2 controls the direction.


// SPI Defines for LTC1859
// Single-Ended Channel Address
#define LTC1859_CH0             0b10000100
#define LTC1859_CH1             0b11010100
#define LTC1859_CH2             0x10010100
#define LTC1859_CH3             0x11010100
#define LTC1859_CH4             0x10100100
#define LTC1859_CH5             0x11100100
#define LTC1859_CH6             0x10110100
#define LTC1859_CH7             0x11110100

// UNDER M� kombineres med LTC1859 defines.... ADDR + INP + POWDWN -> 8bits data ord. (0000) + (00) + (00)...etc
#define inputRange				0x3 // feil verdier.. m� endres
#define PowerDownSel			0x1 //

// DAC8420 Adress in binary
#define DAC_B					0x4
#define DAC_C					0x8
#define DAC_D					0xC

// I2C Devices
#define U7_ADDR					0xD2
#define U8_ADDR					0xCE
#define U9_ADDR					0xDE

// I2C Register addresses
#define sense_A					0x00
#define sense_B					0x01
#define vIn_MSB					0x02
#define vIn_LSB					0x03
#define adIn_MSB				0x04
#define adIn_LSB				0x05
#define control					0x06

//SPI functions
void spi_init_dac();

//ADC functions - LTC1859
void spiTransmitADC_1(uint8_t * dataout, uint8_t datain);
void spiTransmitADC_2(uint8_t * dataout, uint8_t datain);

//DAC functions
void spiTransmitDAC_1(uint8_t dacAdress, uint8_t dacData);
void spiTransmitDAC_2(uint8_t * dataout, uint8_t len);

//TWI/I2C functions.
uint8_t i2c_start(uint8_t address);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length);
uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length);
uint8_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
uint8_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
void i2c_stop(void);

#endif /* COMM_H_ */