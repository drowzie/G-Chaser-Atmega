/*
 * comm.c
 *  Author: chris
 */ 

#include <stdint.h>
#include "comm.h" 
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// SPI defines - Not all ports are correctly set...
#define PORT_SPI			PORTB // PORTB
#define DDR_SPI				DDRB  // Velger hele DDRB
#define DD_MISO				DDB4  // Master input
#define DD_MOSI				DDB3  // Master output
#define DD_SCK				DDB5  // Clock

// I2C Defines
#define I2C_READ 0x01
#define I2C_WRITE 0x00
#define F_SCL 100000U
#define Prescaler 1
#define TWBR_VALUE ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)


// Set up for DAC usage.
void spi_init_dac()
{
	// Reset pins
	DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_MISO)|(1<<DD_SCK));
	// Output
	DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SCK));
	
	SPCR0 = ((1<<SPE)|	// ENABLE
			(0<<SPIE)|	// no interrupt
			(0<<DORD)|	//Data order MSB first
			(1<<MSTR)|	//Master/slave sel
			(0<<SPR1)|(1<<SPR0)| // Spi clock rate -- SPI2X:0 = fosc/4, Spi2x:1 = fosc/2
			(0<<CPOL)|	// Clock polarity
			(0<<CPHA)); // Clock phase
			
	SPSR0 = (1<<SPI2X);  // Double Clock Rate
}

void spiTransmitADC_1(uint8_t * dataout, uint8_t datain)
{
	uint8_t i;
	SPDR0 = datain; // Transmit data
	while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
	PORTC &= ~(1 << ADV_CONVERSION_START_1); // set to 1
	_delay_us(0.0042); // Delay for 42ns++
	PORTC |= (0 << ADV_CONVERSION_START_1); // set to 0
	while((PORTC & (1<<ADC_1_BUSY))==0); // Wait for BUSY in ADC1859 to be set high.
	cli(); // stop interrupt, data recieved now is time important.
	PORTE &= ~(1 << ADC_READ_1); // Activate Read
	for (i = 0; i < 2; i++)
	{
		while(!(SPSR0 & (1<<SPIF)));	// Wait for reception complete.
		dataout[i] = SPDR0;	
	}
	sei();
}
void spiTransmitADC_2(uint8_t * dataout, uint8_t datain)
{
	uint8_t i;
	SPDR0 = datain; // Transmit data
	while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
	PORTD &= ~(1 << ADV_CONVERSION_START_2); // set to 1
	_delay_us(0.0042); // Delay for 42ns++
	PORTD |= (0 << ADV_CONVERSION_START_2); // set to 0
	while((PORTB & (1<<ADC_2_BUSY))==0); // Wait for BUSY in ADC1859 to be set high.
	cli(); // stop intterupt, data recieved now is time important.
	PORTD &= ~(1 << ADC_READ_2); // Activate Read
	for (i = 0; i < 2; i++)
	{
		while(!(SPSR0 & (1<<SPIF)));	// Wait for reception complete.
		dataout[i] = SPDR0;
	}
	sei();
}



// Methods may fail during UART interrupt, uncertain if the extra delay will cause any issues.
// Best solution might just be using it before activating UART interrupt.

void spiTransmitDAC_1(uint8_t * dataout, uint8_t len) 
{
		uint8_t i;
		
		PORTC = (0<<CS_DAC_1)|(1<<LD_DAC_1); // Enable register input.
		// Need delay???
		for(i = 0; i < len; i++) 
		{
			SPDR0 = dataout[i];
			while((SPSR0 & (1<<SPIF))==0);
		}
		PORTC = (1<<CS_DAC_1)|(0<<LD_DAC_1); // Stop data in.
}

void spiTransmitDAC_2(uint8_t * dataout, uint8_t len) 
{
		uint8_t i;
		
		PORTB = (0<<CS_DAC_2)|(1<<LD_DAC_2); // Enable register input.		
		
		for(i = 0; i < len; i++) 
		{
			SPDR0 = dataout[i];
			while((SPSR0 & (1<<SPIF))==0);
		}
		
		PORTB = (1<<CS_DAC_2)|(0<<LD_DAC_2); // Stop register.	
}

// Så langt en kopi av databladet til Atmega 2560


void i2c_init()
{
	TWBR0 = (uint8_t)TWBR_VALUE;
}

/*
*I2C Start is called upon the start when "connecting" with a device.
* Possible error handling in return statements 
*
*/
uint8_t i2c_start(uint8_t address)
{
	// I2C example taken  from ATMEGA 2560 datasheet
	// 1<<TWINT clears the interrupt
	// reset TWI control register
	TWCR0 = 0;
	// transmit START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR0 & (1<<TWINT)) );
	
	// check if the start condition was successfully transmitted
	if((TWSR0 & 0xF8) != TW_START){ return 1; }
	
	// load slave address into data register
	TWDR0 = address;
	// start transmission of address
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR0 & (1<<TWINT)) );
	
	// check if the device has acknowledged the READ / WRITE mode
	uint8_t TWST = TWSR0 & 0xF8;
	
   if ((TWST != TW_MT_SLA_ACK) && (TWST != TW_MR_SLA_ACK)) return 1;
	
	return 0;
}


uint8_t i2c_write(uint8_t data)
{
	// load data into data register
	TWDR0 = data;
	// start transmission of data
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR0 & (1<<TWINT)) );
	
	if( (TWSR0 & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	
	return 0;
}

uint8_t i2c_read_ack(void)
{
	
	// start TWI module and acknowledge data after reception
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// wait for end of transmission
	while( !(TWCR0 & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR0;
}

uint8_t i2c_read_nack(void)
{
	
	// start receiving without acknowledging reception
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR0 & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR0;
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
	if (i2c_start(devaddr | I2C_WRITE)) return 1;

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
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}