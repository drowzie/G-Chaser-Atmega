/*
 * comm.c
 *  Author: chris
 */ 
#include <stdint.h>
#include "comm.h"
#include <avr/io.h>
#include <util/twi.h>



// SPI defines - Not all ports are correctly set...
#define PORT_SPI	PORTB // PORTB
#define DDR_SPI		DDRB  // Velger hele DDRB
#define DD_MISO		DDB4  // Master input
#define DD_MOSI		DDB3  // Master output
#define DD_SS		DDB2  // Slave select
#define DD_SCK		DDB5  // Clock

// I2C Defines
#define I2C_READ 0x01
#define I2C_WRITE 0x00


void spi_init()
{
	// Reset pins
	DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_MISO)|(1<<DD_SS)|(1<<DD_SCK));
	// Output
	DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SS)|(1<<DD_SCK));
	
	SPCR = ((1<<SPE)|	// ENABLE
			(0<<SPIE)|	// no interrupt
			(0<<DORD)|	//Data order MSB first
			(1<<MSTR)|	//Master/slave sel
			(0<<SPR1)|(1<<SPR0)| // Spi clock rate
			(0<<CPOL)|	// Clock polarity
			(0<<CPHA)); // Clock phase
			
	SPSR = (1<<SPI2X);  // Double Clock Rate
}

 // Shift full array(8 bits data) through target device
void spiSync(uint8_t * dataout, uint8_t * datain, uint8_t len) 
{
       uint8_t i;
       for (i = 0; i < len; i++) {
             SPDR = dataout[i];
             while((SPSR & (1<<SPIF))==0);
             datain[i] = SPDR;
       }
}


// Så langt en kopi av databladet til Atmega 2560


uint8_t i2c_start(uint8_t address)
{
	// I2C example taken  from ATMEGA 2560
	// 1<<TWINT clears the interrupt
	// reset TWI control register
	TWCR = 0;
	// transmit START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for TWINT Flag set.
	while( !(TWCR & (1<<TWINT)) );
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){ return 1; }
		
	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	// check if the device has acknowledged the READ / WRITE mode
	uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	
	return 0;
}

uint8_t i2c_write(uint8_t data)
{
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	
	return 0;
}

uint8_t i2c_read_ack(void)
{
	
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}


uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address | I2C_WRITE)) return 1; // if I2c fails, break transmit...
	
	for (uint16_t i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}
	
	i2c_stop();
	
	return 0;
}

uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address | I2C_READ)) return 1; // if I2C fails, break
	
	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	
	data[(length-1)] = i2c_read_nack();
	
	i2c_stop();
	
	return 0;
}

uint8_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_start(devaddr | 0x00)) return 1;

	i2c_write(regaddr);

	for (uint16_t i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}

	i2c_stop();

	return 0;
}

uint8_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	if (i2c_start(devaddr)) return 1;

	i2c_write(regaddr);

	if (i2c_start(devaddr | 0x01)) return 1;

	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();

	i2c_stop();

	return 0;
}

void i2c_stop(void)
{
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}