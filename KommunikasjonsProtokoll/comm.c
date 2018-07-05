/*! \file comm.c	
	\brief All the configurations for SPI and I2C
 *Author: Christoffer Boothby and James Alexander Cowie.
 *Comments:
 */

#include <stdint.h>
#include "comm.h" 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/wdt.h>



// Watchdog timer

//void watchdog_enable()
//{
	//WDTCSR = (1<<WDE)|(0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0);
	//wdt_enable(WDTO_30MS);
	//// wdt_reset();
//}

// SPI defines - Not all ports are correctly set...
#define PORT_SPI			PORTB // PORTB
#define DDR_SPI				DDRB  // Velger hele DDRB
#define DD_MISO				DDRB4  // Master input
#define DD_MOSI				DDRB3  // Master output
#define DD_SCK				DDRB5  // Clock


void spi_init_adc()
{	
	DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SCK)|(0<<DD_MISO));
	
	SPCR0 = ((1<<SPE)|	// ENABLE
			(0<<SPIE)|	// no interrupt
			(1<<MSTR)|	//Master/slave sel
			(0<<SPR1)|(1<<SPR0)| // Spi clock rate -- fosc/16
			(0<<CPOL)|	// Clock polarity // 1 for DAC | 0 for ADC
			(0<<CPHA)); // Clock phase    // 1 for DAC | 0 for ADC

	SPSR0 = (1<<SPI2X);  // Double Clock Rate
}



void spiTransmitADC_2(uint8_t * dataout, uint8_t datain)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) {	
		PORTE &= ~(1<<ADC_READ_1); // low
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
	//_delay_us(0.005);
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
	// wdt_reset();
	//while((PORTB & (1<<ADC_2_BUSY))); // When busy is high
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		PORTD &= ~(1<<ADC_READ_2); // low
		SPDR0 = datain; // Transmit data
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		// wdt_reset();
		dataout[0] = SPDR0;	 // Get MSB
		SPDR0 = 0x00; // transmit dummy byte
		while(!(SPSR0 & (1<<SPIF)))	// Wait for transmit complete
		// wdt_reset();
		dataout[1] = SPDR0;	 // Get MSB
		PORTD |= (1<<ADC_READ_2); // high
	}
	// Start conversion on off
	PORTD |= (1 << ADV_CONVERSION_START_2); // set convst 1
	//_delay_us(0.005);
	PORTD &= ~(1 << ADV_CONVERSION_START_2); // set to 0
	_delay_us(5);
	// wdt_reset();
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
	// wdt_reset();
	PORTB &= ~(1<<CS_DAC_1); // Chip Select go low
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	// Send data
	SPDR0 = dacAdress;
	while(!(SPSR0 & (1<<SPIF)));
	// wdt_reset();
	SPDR0 = dacData;
	while(!(SPSR0 & (1<<SPIF)));
	// wdt_reset();
	// End
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	PORTB |= (1<<CS_DAC_1); // Chip Select go high
	// Strobe the Load Data pin
	PORTB &= ~(1<<LD_DAC_1); // Stop data in.
	PORTB |= (1<<LD_DAC_1);  // set to 1
	_delay_us(5);
	// wdt_reset();
}

/*! \fn void spiTransmitDAC_1 
 * \brief Setting grid voltages on DAC8420
 * \param dacAdress First 8 bits of dacAdress and upper 4 bits of voltage
 * \param dacData lower byte of voltage
 * \return None.
*/

void spiTransmitDAC_2(uint8_t dacAdress, uint8_t dacData)
{
	// wdt_reset();
	PORTC &= ~(1<<CS_DAC_2); // Chip Select go low
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	// Send data
	SPDR0 = dacAdress;
	while(!(SPSR0 & (1<<SPIF)));
	// wdt_reset();
	SPDR0 = dacData;
	while(!(SPSR0 & (1<<SPIF)));
	// wdt_reset();
	_delay_us(0.010); // data sheet says 15ns for TSS, 10ns + clock time
	PORTC |= (1<<CS_DAC_2); // Chip Select go high
	// Strobe the Load Data pin
	PORTC &= ~(1<<LD_DAC_2); // Stop data in.
	PORTC |= (1<<LD_DAC_2);  // set to 1
	_delay_us(5);
	// wdt_reset();
}



// CH0: 0-1: CH1: 2-3 CH2: 4-5 CH3: 6-7 CH4: 8-9 CH5: 10-11CH6 12-13 CH7 14-15
