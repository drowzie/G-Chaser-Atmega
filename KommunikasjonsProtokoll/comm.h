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

// Defines for ports on the SPI settings
	//PCB 1
#define ADV_CONVERSION_START_1			DDRE2
#define ADC_READ_1						DDRE3 
#define ADC_1_BUSY						DDRC0
	// PCB 2
#define ADV_CONVERSION_START_2			DDRB6
#define ADC_READ_2						DDD7
#define ADC_2_BUSY						DDRB0

// DAC PORTS:
	//PCB 1
#define CS_DAC_1						DDRC1				
#define LD_DAC_1						DDRC2
	//PCB 2
#define CS_DAC_2						DDRB1
#define LD_DAC_2						DDRB2 


// SPI Defines for LTC1859
// Single-Ended Channel Address 
#define LTC1859_CH0						0b10000100
#define LTC1859_CH1						0b11010100
#define LTC1859_CH2						0b10010100
#define LTC1859_CH3						0b11010100
#define LTC1859_CH4						0b10100100
#define LTC1859_CH5						0b11100100
#define LTC1859_CH6						0b10110100
#define LTC1859_CH7						0b11110100

// Voltage settings for DAC8420
#define G1_BIAS_1						(uint16_t)0xFFF
#define G2_BIAS_1						(uint16_t)0xFFF
#define G3_BIAS_1						(uint16_t)0xFFF

#define G1_BIAS_2						(uint16_t)0xFFF
#define G2_BIAS_2						(uint16_t)0xFFF
#define G3_BIAS_2						(uint16_t)0xFFF

// DAC8420 ADRESSES for channels
#define DAC_B							0x4
#define DAC_C							0x8
#define DAC_D							0xC

// I2C Devices
#define U7_ADDR							0xD2
#define U8_ADDR							0xCE
#define U9_ADDR							0xDE

// I2C Register addresses
#define I2C_READ						0x01
#define I2C_WRITE						0x00

//SPI functions
void spi_init_dac();
void spi_init_adc();

//ADC functions - LTC1859
void spiTransmitADC_1(uint8_t * dataout, uint8_t datain);
void spiTransmitADC_2(uint8_t * dataout, uint8_t datain);

//DAC functions
void spiTransmitDAC_1(uint8_t dacAdress, uint8_t dacData);
void spiTransmitDAC_2(uint8_t dacAdress, uint8_t dacData);

//TWI/I2C functions.
void i2c_init(void);
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