//
//  spi_ram.c
//  RC_PPM
//
//  Created by Ruedi Heimlicher on 28.07.2013.
//
//

#include <stdio.h>


#include <avr/io.h>
//#include "spi.h"
#include "spi_eeprom.h"

//NOTE: the CS pin on the 23K256 must be brought low before every function
//  call and subsequently brought high after every funcion call. I don't do
//  that in this library so you'll have to do it yourself.


// https://github.com/rememberthe8bit/AVR-libraries/blob/master/spi/spi.c
//shifts out 8 bits of data
//  uint8_t data - the data to be shifted out
//  returns uint8_t - the data received during sending

// Siehe
// http://www.mikrocontroller.net/topic/282013
// code:  http://www.mikrocontroller.net/attachment/164871/25LC128.c

#define EEDELAY 0

/*
uint8_t spi_send(uint8_t value)
{
   uint8_t result;
   
   //shift the first byte of the value
   SPDR = value;
   //wait for the SPI bus to finish
   while(!(SPSR & (1<<SPIF)));
   //get the received data
   //result = SPDR;
   
   return SPDR;
}
*/
//sets the mode and enables or disables the hold pin
//  uint8_t mode - the access mode. valid values are:
//                 0 - byte mode
//                 1 - sequential mode
//                 2 - page mode
//  uint8_t enhold - if 1: disables the hold pin, 0 enables

void spieeprom_init()
{
   
   SPCR=0;
   SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);
   
   // CPOL 0, CPHA 0 > CKE 1 (Microchip bezeichnung)
 
   // SPCR |= (1<<CPOL); // Gleich wie CKP
  // SPCR |= (1<<CPHA); // Gegenteil von CKE

}

uint8_t spieeprom_getmode(void)
{
  
   spi_send(0x05);; // 0x05
   uint8_t mode = spi_send(0xFF);
  
   
   return mode;
}


// WREN schicken, anschliessend CS auf HI
void spieeprom_wren()
{
   _delay_us(EEDELAY);
   spi_send(0x06);
   _delay_us(EEDELAY);

}

// nach WREN WSR schicken 01, 00
void spieeprom_write_status()
{
   //send write status instruction WREN, WRSR, 00 (AN1193 p 39)
   _delay_us(EEDELAY);
   spi_send(01);
   _delay_us(EEDELAY);
   spi_send(00);
   _delay_us(EEDELAY);
   
   
}


// Nach WREN WRITE schicken und msb (02, xy)
void spieeprom_writecmd(uint8_t addr)
{
   _delay_us(EEDELAY);
   //send write instruction
   spi_send(0x02);
   _delay_us(EEDELAY);
   
}

//WRITE schicken, dann msb, lsb, data
// uint16_t addr - the address to write to
// uint8_t data - the data to write
void spieeprom_wrbyte(uint16_t addr, uint8_t data)
{
   
   _delay_us(EEDELAY);
   //send write instruction
   spi_send(0x02);
   _delay_us(EEDELAY);
   //send address

   spi_send((addr>>8)); //most significant byte
   _delay_us(EEDELAY);
   spi_send(addr); //least significant byte
   _delay_us(EEDELAY);
   
   //send data
   spi_send(data);
   _delay_us(EEDELAY);
   
}

//read a memory location
// uint16_t addr - the address to read from
// returns uint8_t - the data read
uint8_t spieeprom_rdbyte(uint16_t addr)
{
   
   uint8_t result = 0x00;
   _delay_us(EEDELAY);
   //send read instruction
   spi_send(0x03);
   _delay_us(EEDELAY);
   //send address
   
   spi_send(addr>>8); //most significant byte
   _delay_us(EEDELAY);
   spi_send(addr); //least significant byte
   _delay_us(EEDELAY);
   
   //read data
   result = spi_send(0x66); //send clock pulses, get result
   _delay_us(EEDELAY);
   
   return result;
}


uint8_t spieeprom_read_status()
{
   uint8_t val;
   _delay_us(EEDELAY);
   spi_send(0x05);
   _delay_us(EEDELAY);
   val = spi_send(0xff);
   _delay_us(EEDELAY);
   return val;
}



