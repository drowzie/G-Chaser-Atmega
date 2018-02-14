/*
 * KommunikasjonsProtokoll.c
 *
 * Created: 05.02.2018 12.07.27
 * Author : chris
 */ 
#include <stdint.h>
#include <avr/io.h>
#include "comm.h"


// I2C Devices
#define U7_ADDR		0xD2		
#define U8_ADDR		0xCE
#define U9_ADDR		0xDE

// I2C Register adresses
#define sense_A		0x00
#define sense_B		0x01
#define vIn_MSB		0x02
#define vIn_LSB		0x03
#define adIn_MSB	0x04
#define adIn_LSB	0x05
#define control		0x06


int main(void)
{
    /* Replace with your application code */
	spi_init();
	
	i2c_start(U7_ADDR);
	
return 0;
}
