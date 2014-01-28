//
//  spi_ram.h
//  RC_PPM
//
//  Created by Ruedi Heimlicher on 28.07.2013.
//
//

#ifndef RC_PPM_spi_ram_h
#define RC_PPM_spi_ram_h
/***
 Copyright (C) 2011 David DiPaola
 
 ***/


#define SPI_RAM_DDR            DDRD    //
#define SPI_RAM_PORT           PORTD   //

#define SPI_RAM_CS_PIN                     1




#define RAM_CS_HI SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN)
#define RAM_CS_LO SPI_RAM_PORT &= ~(1<<SPI_RAM_CS_PIN)


//NOTE: the CS pin on the 23K256 must be brought low before every function
//  call and subsequently brought high after every funcion call. I don't do
//  that in this library so you'll have to do it yourself.

//sets the mode and enables or disables the hold pin
//  uint8_t mode - the access mode. valid values are:
//                 0 - byte mode
//                 1 - sequential mode
//                 2 - page mode
//  uint8_t enhold - if 1: disables the hold pin, 0 enables
//void spiram_init(uint8_t mode, uint8_t enhold);

void spiram_init(void);
uint8_t spiram_read_status(void);
                 

//write a memory location
// uint16_t addr - the address to write to
// uint8_t data - the data to write
void spiram_wrbyte(uint16_t addr, uint8_t data);

//read a memory location
// uint16_t addr - the address to read from
// returns uint8_t - the data read
uint8_t spiram_rdbyte(uint16_t addr);

//writes an array to an address
//  uint16_t startaddr - the address the first byte will be written to
//  const uint8_t* data - the array to be written
//  uint16_t length - the number of bytes to be written from the array
void spiram_wrseq(uint16_t startaddr, const uint8_t* data, uint16_t length);

//reads a portion of memory into an array
//  uint16_t startaddr - the address the first byte will be read from
//  uint8_t* data - the array to be written to
//  uint16_t length - the number of bytes to be read from memory
void spiram_rdseq(uint16_t startaddr, uint8_t* data, uint16_t length);

void readSettings(void);

#endif
