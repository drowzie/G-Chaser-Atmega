/*! \file comm.c	
	\brief All the configurations for SPI and I2C
 *Author: Christoffer Boothby and James Alexander Cowie.
 *Comments:
 */

#include <stdint.h>
#include "comm.h" 
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/wdt.h>



// Watchdog timer

void watchdog_enable()
{
	WDTCSR = (1<<WDE)|(0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0);
	wdt_enable(WDTO_15MS);
	//
}

// SPI defines - Not all ports are correctly set...
#define PORT_SPI			PORTB // PORTB
#define DDR_SPI				DDRB  // Velger hele DDRB
#define DD_MISO				DDRB4  // Master input
#define DD_MOSI				DDRB3  // Master output
#define DD_SCK				DDRB5  // Clock


void spi_init_adc()
{	
	// Output
	DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SCK)|(0<<DD_MISO));
	
	SPCR0 = ((1<<SPE)|	// ENABLE
			(0<<SPIE)|	// no interrupt
			(1<<MSTR)|	//Master/slave sel
			(0<<SPR1)|(0<<SPR0)| // Spi clock rate -- fosc/16
			(0<<CPOL)|	// Clock polarity // 1 for DAC | 0 for ADC
			(0<<CPHA)); // Clock phase    // 1 for DAC | 0 for ADC

	SPSR0 = (0<<SPI2X);  // Double Clock Rate
//	//
}


// communicate with adc, get voltages, put them in to the storage of their last communcation, update the new one.

void spiTransmitADC_2(uint8_t * dataout, uint8_t datain)
{
	//while((PORTC & (1<<ADC_1_BUSY))); // When busy is high
	ATOMIC_BLOCK(ATOMIC_FORCEON) {	
		PORTE &= ~(1<<ADC_READ_1); // low
		SPDR0 = datain; // Transmit data
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		//
		dataout[0] = SPDR0;	 // Get MSB
		SPDR0 = 0x00; // transmit dummy byte
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		//
		dataout[1] = SPDR0;	 // Get MSB
		PORTE |= (1<<ADC_READ_1); // high
		
		// Dataout is from last buffer, store it into the buffer of pdata old.
		
		// Update with datain(ID)
	}
	// Start conversion on off
	PORTE |= (1 << ADV_CONVERSION_START_1); // set convst 1
	_delay_us(0.040);
	PORTE &= ~(1 << ADV_CONVERSION_START_1); // set to 0
	_delay_us(5);
}

/*! \fn void spiTransmitADC_1 
 * \brief Transmitting to LTC1859.
 * \param dataout Two bytes of recieved data from LTC1859.
 * \param datain adress to be sent in.
 * \return None.
*/


void spiTransmitADC_1(uint8_t * dataout, uint8_t datain)
{
	//while((PORTB & (1<<ADC_2_BUSY))); // When busy is high
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		PORTD &= ~(1<<ADC_READ_2); // low
		SPDR0 = datain; // Transmit data
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		//
		dataout[0] = SPDR0;	 // Get MSB
		SPDR0 = 0x00; // transmit dummy byte
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		//
		dataout[1] = SPDR0;	 // Get MSB
		PORTD |= (1<<ADC_READ_2); // high
	}
	// Start conversion on off
	PORTD |= (1 << ADV_CONVERSION_START_2); // set convst 1
	_delay_us(0.040);
	PORTD &= ~(1 << ADV_CONVERSION_START_2); // set to 0
	_delay_us(5);
}


void spi_init_dac()
{
	// Output
	DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SCK)|(0<<DD_MISO));
	
	SPCR0 = ((1<<SPE)|	// ENABLE
	(0<<SPIE)|	// no interrupt
	(1<<MSTR)|	//Master/slave sel
	(0<<SPR1)|(1<<SPR0)| // Spi clock rate -- fosc/16
	(1<<CPOL)|	// Clock polarity // 1 for DAC | 0 for ADC
	(1<<CPHA)); // Clock phase    // 1 for DAC | 0 for ADC
	
	SPSR0 = (0<<SPI2X);  // Double Clock Rate
	////
}


void spiTransmitDAC_1(uint8_t dacAdress, uint8_t dacData) 
{
	PORTB &= ~(1<<CS_DAC_1); // Chip Select go low
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	// Send data
	SPDR0 = dacAdress;
	while(!(SPSR0 & (1<<SPIF)));
	//
	SPDR0 = dacData;
	while(!(SPSR0 & (1<<SPIF)));
	//
	// End
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	PORTB |= (1<<CS_DAC_1); // Chip Select go high
	// Strobe the Load Data pin
	PORTB &= ~(1<<LD_DAC_1); // Stop data in.
	PORTB |= (1<<LD_DAC_1);  // set to 1
	_delay_us(5);
}

/*! \fn void spiTransmitDAC_1 
 * \brief Setting grid voltages on DAC8420
 * \param dacAdress First 8 bits of dacAdress and upper 4 bits of voltage
 * \param dacData lower byte of voltage
 * \return None.
*/

void spiTransmitDAC_2(uint8_t dacAdress, uint8_t dacData)
{
	PORTC &= ~(1<<CS_DAC_2); // Chip Select go low
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	// Send data
	SPDR0 = dacAdress;
	while(!(SPSR0 & (1<<SPIF)));
	//
	SPDR0 = dacData;
	while(!(SPSR0 & (1<<SPIF)));
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	PORTC |= (1<<CS_DAC_2); // Chip Select go high
	// Strobe the Load Data pin
	PORTC &= ~(1<<LD_DAC_2); // Stop data in.
	PORTC |= (1<<LD_DAC_2);  // set to 1
	_delay_us(5);
}

//
//#pragma region i2c
//
//// A detailed guide about using I2C and these functions is seen in datasheet for ATMEGA328PB
//// I2C Defines
//#define TWI_FREQ 10000
//#define Prescaler 16
//
//void i2c_init(void)
//{
	//TWSR0 = (1<<TWPS1)|(0<<TWPS0);
	//TWBR0 = ((((F_CPU / TWI_FREQ) / Prescaler) - 16 ) / 2);
	//TWCR0 = (1<<TWEN);
	//PORTC |= (1<<PORTC5)|(1<<PORTC4);
//}
//
//void TWIStart(void)
//{
	//TWCR0 = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	//while ((TWCR0 & (1<<TWINT)) == 0);
	//////
//}
////send stop signal
//void TWIStop(void)
//{
	//TWCR0 = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
//}
//
//void TWIWrite(uint8_t u8data)
//{
	//TWDR0 = u8data;
	//TWCR0 = (1<<TWINT)|(1<<TWEN);
	//while ((TWCR0 & (1<<TWINT)) == 0);
	//////
//}
//
//uint8_t TWIReadACK(void)
//{
	//TWCR0 = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	//while ((TWCR0 & (1<<TWINT)) == 0);
	//////
	//return TWDR0;
//}
////read byte with NACK
//uint8_t TWIReadNACK(void)
//{
	//TWCR0 = (1<<TWINT)|(1<<TWEN);
	//while ((TWCR0 & (1<<TWINT)) == 0);
	//////
	//return TWDR0;
//}
//uint8_t TWIGetStatus(void)
//{
	//uint8_t status;
	////mask status
	//status = TWSR0 & 0xF8;
	//return status;
//}
//uint8_t PWMReadByte(uint8_t address, uint8_t reg, uint8_t* dataout) 
//{
		//TWIStart();
		//if (TWIGetStatus() != TW_START) // check for start
			//{return Error;}
		//TWIWrite(address|TW_WRITE);
		//if (TWIGetStatus() != TW_MT_SLA_ACK) // check for ack
			//{return Error;}
		//TWIWrite(reg);
		//if (TWIGetStatus() != TW_MT_DATA_ACK) // check if data has been transmitted and ack recieved
			//{return Error;}
		//TWIStart();
		//if (TWIGetStatus() != TW_REP_START) // check if data has been transmitted and ack recieved
			//{return Error;}
		//TWIWrite(address|TW_READ);
		//if (TWIGetStatus() != TW_MR_SLA_ACK)    // Master reciever mode ack
			//{return Error;}
		//dataout[0] = TWIReadACK();				// Get the first value of register: 8MSB
		//if (TWIGetStatus() != TW_MR_DATA_ACK)	// Master recieve data ack
			//{return Error;}
		//dataout[1] = TWIReadNACK();				// Get the next value of register: 4LSB
		//if (TWIGetStatus() != TW_MR_DATA_NACK)  // Master recieve nack
			//{return Error;}
		//TWIStop();
	//return Success;
//}
//
//void twiDataHandler (uint8_t address, uint8_t reg, uint8_t* dataout)
//{
	////_delay_us(30); 
	//if(PWMReadByte(address,reg, dataout) == Error) // IF ERROR
	//{		
		//dataout[0] = 0x80;						
		//dataout[1] = 0x80;
		//TWCR0 = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); // send stop condition
	//}
	////
//};



#pragma endregion

#pragma endregion

// CH0: 0-1: CH1: 2-3 CH2: 4-5 CH3: 6-7 CH4: 8-9 CH5: 10-11CH6 12-13 CH7 14-15