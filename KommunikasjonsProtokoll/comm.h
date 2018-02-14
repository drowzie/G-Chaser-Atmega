/*
 * comm.h
 *
 * Created: 14.02.2018 12.06.18
 *  Author: chris
 */ 


#ifndef COMM_H_
#define COMM_H_
#include <stdint.h>


//SPI functions
void spi_init();
void spiSync(uint8_t * dataout, uint8_t * datain, uint8_t len);

//i2c functions
uint8_t i2c_start(uint8_t address);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length);
uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length);
uint8_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
uint8_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);

// Might remove 
void i2c_stop(void);

#endif /* COMM_H_ */