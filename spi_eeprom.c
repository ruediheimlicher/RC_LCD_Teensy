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

#define EEDELAY 50

#define EE_READ_DELAY 0

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
    EE_CS_LO;
   _delay_us(EEDELAY);
   spi_send(0x06);
   _delay_us(EEDELAY);
    EE_CS_HI;

}

// nach WREN WSR schicken 01, 00
void spieeprom_write_status()
{
   //send write status instruction WREN, WRSR, 00 (AN1193 p 39)
    EE_CS_LO;
   _delay_us(EEDELAY);
   spi_send(01);
   _delay_us(EEDELAY);
   spi_send(00);
   _delay_us(EEDELAY);
    EE_CS_HI;
   
   
}


// Nach WREN WRITE schicken und msb (02, xy)
void spieeprom_writecmd(uint8_t addr)
{
    EE_CS_LO;
   _delay_us(EEDELAY);
   //send write instruction
   spi_send(0x02);
   _delay_us(EEDELAY);
    EE_CS_HI;
   
}

//WRITE schicken, dann msb, lsb, data
// uint16_t addr - the address to write to
// uint8_t data - the data to write
void spieeprom_wrbyte(uint16_t addr, uint8_t data)
{
    EE_CS_LO;
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
    EE_CS_HI;
   
}

//WRITE schicken zum start einer page, dann msb, lsb
// uint16_t addr - the address to write to
void spieeprom_wrpage_start(uint16_t addr)
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
   //send data after
}

//Weitere Daten schicken
void spieeprom_wrpage_data(uint8_t data)
{
   //send data
   spi_send(data);
   _delay_us(EEDELAY);

}




//read a memory location
// uint16_t addr - the address to read from
// returns uint8_t - the data read
uint8_t spieeprom_rdbyte(uint16_t addr)
{
    EE_CS_LO;
   uint8_t result = 0x00;
   _delay_us(EE_READ_DELAY);
   //send read instruction
   
   spi_send(0x03);
   
   _delay_us(EE_READ_DELAY);
   
   //send address
  
   spi_send(addr>>8); //most significant byte
   _delay_us(EE_READ_DELAY);
   spi_send(addr); //least significant byte
   _delay_us(EE_READ_DELAY);
   
   //read data
   result = spi_send(0x66); //send clock pulses, get result
   _delay_us(EE_READ_DELAY);
    EE_CS_HI;
   
   return result;
}

//READ schicken zum start einer page, dann msb, lsb
// uint16_t addr - the address to write to
void spieeprom_rdpage_start(uint16_t addr)
{
   
   _delay_us(EEDELAY);
   //send read instruction
   spi_send(0x03);
   _delay_us(EEDELAY);
   //send address
   
   spi_send(addr>>8); //most significant byte
   _delay_us(EEDELAY);
   spi_send(addr); //least significant byte
   _delay_us(EEDELAY);

}


//Weitere Daten schicken
uint8_t spieeprom_rdpage_data()
{
   uint8_t result = 0x00;
   //send dummy data, read result
   result = spi_send(0x66); //send clock pulses, get result
   _delay_us(EEDELAY);
   return result;
}

//writes an page to an address
//  uint16_t startaddr - the address the first byte will be written to
//  const uint8_t* data - the array to be written
//  uint16_t length - the number of bytes to be written from the array

void spieeprom_wrpage(uint16_t startaddr, const volatile uint8_t* data)

//void spieeprom_wrpage(uint16_t startaddr, const char* data)
{
   uint16_t i;
   
   //send the write instruction
   spi_send(0x02);
   //send address
   spi_send(startaddr>>8);
   spi_send(startaddr);
   //send data
   for(i=0; i<EE_PAGESIZE; i++)
   {
      spi_send(data[i]);
   }
}

//reads a page of memory into an array
//  uint16_t startaddr - the address the first byte will be read from
//  uint8_t* data - the array to be written to

void spieeprom_rdpage(uint16_t startaddr, volatile uint8_t* data)
//void spieeprom_rdpage(uint16_t startaddr, char* data)
{
   uint16_t i;
   
   //send the read instruction
   spi_send(0x03);
   //send address
   spi_send(startaddr>>8);
   spi_send(startaddr);
   //read in data
   for(i=0; i<EE_PAGESIZE; i++)
   {
      data[i] = spi_send(0x77);
   }
}



uint8_t spieeprom_read_status()
{
    EE_CS_LO;
   uint8_t val;
   _delay_us(EEDELAY);
   spi_send(0x05);
   _delay_us(EEDELAY);
   val = spi_send(0xff);
   _delay_us(EEDELAY);
    EE_CS_HI;
   return val;
}


//writes an array to an address
//  uint16_t startaddr - the address the first byte will be written to
//  const uint8_t* data - the array to be written
//  uint16_t length - the number of bytes to be written from the array
void spiram_wrseq(uint16_t startaddr, const uint8_t* data, uint16_t length)
{
   uint16_t i;
   
   //send the write instruction
   spi_send(__WRITE_INST);
   //send address
   spi_send(startaddr>>8);
   spi_send(startaddr);
   //send data
   for(i=0; i<length; i++){
      spi_send(data[i]);
   }
}

//reads a portion of memory into an array
//  uint16_t startaddr - the address the first byte will be read from
//  uint8_t* data - the array to be written to
//  uint16_t length - the number of bytes to be read from memory
void spiram_rdseq(uint16_t startaddr, uint8_t* data, uint16_t length){
   uint16_t i;
   
   //send the read instruction
   spi_send(__READ_INST);
   //send address
   spi_send(startaddr>>8);
   spi_send(startaddr);
   //read in data
   for(i=0; i<length; i++){
      data[i] = spi_send(__NULL_INST);
   }
}

void test_rdpage(uint16_t startaddr, volatile char* data)
{
   
}

void test_rdpage_uint(uint16_t startaddr, volatile uint8_t* data)
{
   
}

