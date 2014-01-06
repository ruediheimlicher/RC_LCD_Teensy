//
//  spi_ram.c
//  RC_PPM
//
//  Created by Ruedi Heimlicher on 28.07.2013.
//
//

#include <stdio.h>

/***
 Copyright (C) 2011 David DiPaola
 https://github.com/rememberthe8bit/AVR-libraries/blob/master/23K256/spiram.c
 ***/

#include <avr/io.h>
//#include "spi.h"
#include "spi_ram.h"
#include "def.h"

//NOTE: the CS pin on the 23K256 must be brought low before every function
//  call and subsequently brought high after every funcion call. I don't do
//  that in this library so you'll have to do it yourself.

//instructions
const uint8_t __WRITE_INST = 0b00000010; //write to memory        0x02
const uint8_t __READ_INST  = 0b00000011; //read from memory       0x03
const uint8_t __WRSR_INST  = 0b00000001; //write STATUS register  0x01
const uint8_t __RDSR_INST  = 0b00000101; //read STATUS register   0x05
const uint8_t __NULL_INST  = 0b10111110; //invalid/do nothing instruction
//modes of operation
const uint8_t __MODE_MASK = 0b11000000;      // 0xC0
const uint8_t __MODE_BITS = 6; //mode bits start at bit 6
//hold pin
const uint8_t __HOLD_MASK = 0b00000001;

// https://github.com/rememberthe8bit/AVR-libraries/blob/master/spi/spi.c
//shifts out 8 bits of data
//  uint8_t data - the data to be shifted out
//  returns uint8_t - the data received during sending

#define RAMDELAY 0
#define M1 1   // 1MB Device

uint8_t spi_send(uint8_t value)
{
   uint8_t result;
   
   //shift the first byte of the value
   SPDR = value;
   //wait for the SPI bus to finish
   while(!(SPSR & (1<<SPIF)))
   {
      //OSZI_A_TOGG;//
     ;
   }
   //get the received data
   //result = SPDR;
   
   return SPDR;
}

//sets the mode and enables or disables the hold pin
//  uint8_t mode - the access mode. valid values are:
//                 0 - byte mode
//                 1 - sequential mode
//                 2 - page mode
//  uint8_t enhold - if 1: disables the hold pin, 0 enables

void spiram_init()
{
   
   SPCR=0;
   SPCR = (1<<SPE)|(1<<MSTR);
 //  SPCR |= (1<<SPR0);
//   SPCR |= (1<<SPR1);
   
   //SPCR |= (1<<CPOL); // Gleich wie CKP
   
   //SPCR |= (1<<CPHA); // Gegenteil von CKE

}

uint8_t spiram_getmode(void)
{
  
   spi_send(0x05);; // 0x05
   uint8_t mode = spi_send(0xFF);
  
   
   return mode;
}


void spiram_write_status(uint8_t mode)
{
   //send write status instruction
   _delay_us(RAMDELAY);
   spi_send(0x01);
   _delay_us(RAMDELAY);
   spi_send(mode);
   _delay_us(RAMDELAY);
   
   
}


uint8_t spiram_read_status()
{
   uint8_t val;
   _delay_us(RAMDELAY);
   spi_send(0x05);
   _delay_us(RAMDELAY);
   val = spi_send(0xff);
   _delay_us(RAMDELAY);
   return val;
}


//write a memory location
// uint16_t addr - the address to write to
// uint8_t data - the data to write
void spiram_wrbyte(uint16_t addr, uint8_t data)
{
   
   _delay_us(RAMDELAY);
   //send write instruction
   spi_send(0x02);
   _delay_us(RAMDELAY);
   //send address
   if (M1) // 24-Bit-Adresse
   {
      spi_send(0x00); //first byte of 24 bit Address
      _delay_us(RAMDELAY);
   }
   spi_send((addr>>8)); //most significant byte
   _delay_us(RAMDELAY);
   spi_send(addr); //least significant byte
   _delay_us(RAMDELAY);
   
   //send data
   spi_send(data);
   _delay_us(RAMDELAY);
   
}

//read a memory location
// uint16_t addr - the address to read from
// returns uint8_t - the data read
uint8_t spiram_rdbyte(uint16_t addr)
{
   
   uint8_t result = 0x00;
   _delay_us(RAMDELAY);
   //send read instruction
   spi_send(0x03);
   _delay_us(RAMDELAY);
   //send address
   if (M1)// 24-Bit-Adresse
   {
      spi_send(0x00); //first byte of 24 bit Address
      _delay_us(RAMDELAY);
   }
   
   spi_send(addr>>8); //most significant byte
   _delay_us(RAMDELAY);
   spi_send(addr); //least significant byte
   _delay_us(RAMDELAY);
   //   spi_send(addr); //least significant byte
   //  _delay_us(10);
   
   //read data
   result = spi_send(0x66); //send clock pulses, get result
   _delay_us(RAMDELAY);
   
   return result;
}


