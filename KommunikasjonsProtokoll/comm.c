/*! \file comm.c	
	\brief All the configurations for SPI and I2C
 *Author: Christoffer Boothby.
 *Version: 0.0.1.2.
 *Comments:.
 */

#include <stdint.h>
#include "comm.h" 
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

// SPI defines - Not all ports are correctly set...
#define PORT_SPI			PORTB // PORTB
#define DDR_SPI				DDRB  // Velger hele DDRB
#define DD_MISO				DDRB4  // Master input
#define DD_MOSI				DDRB3  // Master output
#define DD_SCK				DDRB5  // Clock


/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

void spi_init_adc()
{	
	// Output
	DDR_SPI = ((1<<DD_MOSI)|(1<<DD_SCK)|(0<<DD_MISO));
	
	SPCR0 = ((1<<SPE)|	// ENABLE
			(0<<SPIE)|	// no interrupt
			(1<<MSTR)|	//Master/slave sel
			(0<<SPR1)|(1<<SPR0)| // Spi clock rate -- fosc/16
			(0<<CPOL)|	// Clock polarity // 1 for DAC | 0 for ADC
			(0<<CPHA)); // Clock phase    // 1 for DAC | 0 for ADC
			
	SPSR0 = (0<<SPI2X);  // Double Clock Rate
}
/*
Method Name: spiTransmitADC_1
----------------------------
Purpose: To transmit over the SPI line to LTC1859 on PCB1.

Argument: dataout - will recieve two bytes from the LTC1859
datain - Channel byte which goes in to IC.

returns: none

error handling: None necessary, if SPI is broken. Returns FF

*/

void spiTransmitADC_1(uint8_t * dataout, uint8_t datain)
{
	// while((PORTC & (0<<ADC_1_BUSY))); // When busy is high
	
	PORTE &= ~(1<<ADC_READ_1); // low
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		SPDR0 = datain; // Transmit data
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		dataout[0] = SPDR0;	 // Get MSB
		SPDR0 = 0x00; // transmit dummy byte
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		dataout[1] = SPDR0;	 // Get MSB
		PORTE |= (1<<ADC_READ_1); // high
	}
	// Start conversion on off
	PORTE |= (1 << ADV_CONVERSION_START_1); // set convst 1
	_delay_us(0.005);
	PORTE &= ~(1 << ADV_CONVERSION_START_1); // set to 0
}

/*
Method Name: spiTransmitADC_2
----------------------------
Purpose: To transmit over the SPI line to LTC1859 on PCB1.

Argument: dataout - will recieve two bytes from the LTC1859
datain - Channel byte which goes in to IC.

returns: none

error handling: None necessary, if SPI is broken. Returns FF

*/


void spiTransmitADC_2(uint8_t * dataout, uint8_t datain)
{
	// while((PORTC & (0<<ADC_1_BUSY))); // When busy is high
	
	PORTE &= ~(1<<ADC_READ_1); // low
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		SPDR0 = datain; // Transmit data
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		dataout[0] = SPDR0;	 // Get MSB
		SPDR0 = 0x00; // transmit dummy byte
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		dataout[1] = SPDR0;	 // Get MSB
		PORTE |= (1<<ADC_READ_1); // high
	}
	// Start conversion on off
	PORTE |= (1 << ADV_CONVERSION_START_1); // set convst 1
	_delay_us(0.005);
	PORTE &= ~(1 << ADV_CONVERSION_START_1); // set to 0
}

/*

Method Name: 
----------------------------
Purpose: 

Argument: 

returns:

error handling:

*/


void spi_init_dac()
{
	// Output
	DDR_SPI = ((1<<DD_MOSI)|(1<<DD_SCK)|(0<<DD_MISO));
	
	SPCR0 = ((1<<SPE)|	// ENABLE
	(0<<SPIE)|	// no interrupt
	(1<<MSTR)|	//Master/slave sel
	(0<<SPR1)|(1<<SPR0)| // Spi clock rate -- fosc/16
	(1<<CPOL)|	// Clock polarity // 1 for DAC | 0 for ADC
	(1<<CPHA)); // Clock phase    // 1 for DAC | 0 for ADC
	
	SPSR0 = (0<<SPI2X);  // Double Clock Rate
}

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

void spiTransmitDAC_1(uint8_t dacAdress, uint8_t dacData) 
{
		PORTC &= ~(1<<CS_DAC_1); // Chip Select go low
		_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
		// Send data
		SPDR0 = dacAdress;
		while(!(SPSR0 & (1<<SPIF)));
		SPDR0 = dacData;
		while(!(SPSR0 & (1<<SPIF)));
		// End
		_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
		PORTC |= (1<<CS_DAC_1); // Chip Select go high
		// Strobe the Load Data pin
		PORTC &= ~(1<<LD_DAC_1); // Stop data in.
		PORTC |= (1<<LD_DAC_1);  // set to 1
		_delay_ms(1);
}

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

void spiTransmitDAC_2(uint8_t dacAdress, uint8_t dacData)
{
	PORTC &= ~(1<<CS_DAC_2); // Chip Select go low
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	// Send data
	SPDR0 = dacAdress;
	while(!(SPSR0 & (1<<SPIF)));
	SPDR0 = dacData;
	while(!(SPSR0 & (1<<SPIF)));
	// End
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	PORTC |= (1<<CS_DAC_2); // Chip Select go high
	// Strobe the Load Data pin
	PORTC &= ~(1<<LD_DAC_2); // Stop data in.
	PORTC |= (1<<LD_DAC_2);  // set to 1
	_delay_ms(1);
}


#pragma region i2c

// I2C Defines
#define TWI_FREQ 2000
#define Prescaler 64

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

void i2c_init(void)
{
	TWSR0 = (1<<TWPS1)|(1<<TWPS0);
	TWBR0 = ((((F_CPU / TWI_FREQ) / Prescaler) - 16 ) / 2);
	TWCR0 = 0;
	PORTC |= (1<<PORTC5)|(1<<PORTC4);
}

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

uint8_t i2c_start(uint8_t address)
{
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
	if ((TWSR0 & 0xF8) != TW_MT_DATA_ACK) return 1;
	
	return 0;
}

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

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

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

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

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address | I2C_WRITE)) return 1;
	
	for (uint16_t i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}
	
	i2c_stop();
	
	return 0;
}

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length)
{
	if (i2c_start(address | I2C_READ)) return 1;
	
	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();
	
	i2c_stop();
	
	return 0;
}

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

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

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

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

/*

Method Name:
----------------------------
Purpose:

Argument:

returns:

error handling:

*/

void i2c_stop(void)
{
	// transmit STOP condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	while(!(TWCR0 & (1<<TWSTO)));
}

#pragma endregion