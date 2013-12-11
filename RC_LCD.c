//
//  RC_PPM.c
//
//
//  Created by Sysadmin on 20.07.13
//  Copyright Ruedi Heimlicher 2013. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <math.h>

#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
#include "def.h"

//#include "spi.c"
#include "spi_adc.c"
#include "spi_ram.c"
#include "spi_eeprom.c"

// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LOOPDELAY 5

#define SERVOMAX  4400
#define SERVOMIN  1400


#define USB_DATENBREITE 64
#define EE_PARTBREITE 32

volatile uint8_t do_output=0;
static volatile uint8_t testbuffer[USB_DATENBREITE]={};


static volatile uint8_t buffer[USB_DATENBREITE]={};
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

static volatile uint8_t outbuffer[USB_DATENBREITE]={};
static volatile uint8_t inbuffer[USB_DATENBREITE]={};

static volatile uint8_t kontrollbuffer[USB_DATENBREITE]={};

static volatile uint8_t eeprombuffer[USB_DATENBREITE]={};

#define TIMER0_STARTWERT	0x40

volatile uint8_t timer0startwert=TIMER0_STARTWERT;

//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

static volatile uint8_t             adcstatus=0x00;
static volatile uint8_t             usbstatus=0x00;

static volatile uint8_t             usbtask=0x00; // was ist zu tun

static volatile uint8_t             eepromstatus=0x00;
static volatile uint8_t             potstatus=0x00; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t             impulscounter=0x00;

static volatile uint8_t             masterstatus = 0;

#define USB_RECV  0

volatile uint8_t status=0;

volatile uint8_t                    PWM=0;
static volatile uint8_t             pwmposition=0;
static volatile uint8_t             pwmdivider=0;


volatile char SPI_data='0';
volatile char SPI_dataArray[SPI_BUFSIZE];
volatile uint16_t Pot_Array[SPI_BUFSIZE];

volatile uint16_t Mitte_Array[8];

volatile uint8_t Level_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal
volatile uint8_t Expo_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal

volatile uint16_t Mix_Array[8];// Mixings, 2 8-bit-bytes pro Mixing


volatile uint16_t RAM_Array[SPI_BUFSIZE];

volatile uint16_t Batteriespannung =0;

volatile uint16_t adc_counter =0; // zaehlt Impulspakete bis wieder die Batteriespannung gelesen werden soll

volatile short int received=0;

volatile uint16_t abschnittnummer=0;

volatile uint16_t usbcount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;

volatile uint16_t eepromstartadresse=0;

volatile uint16_t inchecksumme=0;

volatile uint16_t bytechecksumme=0;
volatile uint16_t outchecksumme=0;

volatile uint8_t eeprom_databyte=0;
volatile uint8_t anzahlpakete=0;
volatile uint8_t eeprom_errcount = 0;

volatile uint8_t  eeprom_indata=0;

volatile    uint8_t task_in=0;      // Task von RC_PPM
volatile    uint8_t task_indata=0;  // Taskdata von RC_PPM

volatile    uint8_t task_out=0;     // Task an RC_PPM
volatile    uint8_t task_outdata=0; // Taskdata an RC_PPM



// MARK: Proto
uint8_t eeprombyteschreiben(uint8_t code, uint16_t writeadresse,uint8_t eeprom_writedatabyte);
uint8_t eeprombytelesen(uint16_t readadresse); // 300 us ohne lcd_anzeige
uint8_t eeprompartlesen(uint16_t readadresse); //   us ohne lcd_anzeige
uint16_t eeprompartschreiben(void); // 23 ms


void startTimer2(void)
{
   //timer2
   TCNT2   = 0;
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}

void Master_Init(void)
{
	//OSZIPORTDDR |= (1<<PULS);	// Output
	//OSZIPORT |= (1<<PULS);		// HI
	
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
   
   
   OSZIPORTDDR |= (1<<OSZI_PULS_C);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_C);		//Pin   von   als Ausgang fuer OSZI
   
   OSZIPORTDDR |= (1<<OSZI_PULS_D);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_D);		//Pin   von   als Ausgang fuer OSZI
	
   /*
    TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
    TASTENPORT |= (1<<TASTE0);	//Pull-up
    */
   
   
   /**
	 * Pin Change Interrupt enable on PCINT0 (PD7)
	 */
   
   PCIFR |= (1<<PCIF0);
   PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT7);
   
   
   
   
   INTERRUPT_DDR &= ~(1 << MASTER_EN_PIN); // Clear the PB7 pin
   // PB7 (PCINT7 pin) are now inputs
   
   INTERRUPT_PORT |= (1 << MASTER_EN_PIN) ; // turn On the Pull-up
   // PB7 are now inputs with pull-up enabled
   
   MASTER_DDR |= (1 << SUB_BUSY_PIN); // BUSY-Pin als Ausgang
   // PB7 (PCINT7 pin) are now inputs
   
   MASTER_PORT |= (1 << SUB_BUSY_PIN) ; // turn On the Pull-up
   // PB7 are now inputs with pull-up enabled
   
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
   ADC_DDR &= ~(1<<PORTF0);
   
   MEM_EN_DDR |= (1<<MEM_EN_PIN);
   MEM_EN_PORT |= (1<<MEM_EN_PIN);
   
   
}

void SPI_PORT_Init(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // HI   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
}

void SPI_ADC_init(void) // SS-Pin fuer EE aktivieren
{
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
}


void SPI_RAM_init(void) // SS-Pin fuer RAM aktivieren
{
   SPI_RAM_DDR |= (1<<SPI_RAM_CS_PIN); // RAM-CS-PIN Ausgang
   SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);// HI
   
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // LO
}

void SPI_EE_init(void) // SS-Pin fuer EE aktivieren
{
   SPI_EE_DDR |= (1<<SPI_EE_CS_PIN); // EE-CS-PIN Ausgang
   SPI_EE_PORT |= (1<<SPI_EE_CS_PIN);// HI
}

void spi_start(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // LO
   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
   
   // RAM-CS bereit
   SPI_RAM_DDR |= (1<<SPI_RAM_CS_PIN); // RAM-CS-PIN Ausgang
   SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);// HI
   
   // EE-CS bereit
   SPI_EE_DDR |= (1<<SPI_EE_CS_PIN); // EE-CS-PIN Ausgang
   SPI_EE_PORT |= (1<<SPI_EE_CS_PIN);// HI
   
   
}

void spi_end(void) // SPI-Pins deaktivieren
{
   SPCR=0;
   
   SPI_DDR &= ~(1<<SPI_MOSI_PIN); // MOSI off
   SPI_DDR &= ~(1<<SPI_SCK_PIN); // SCK off
   SPI_DDR &= ~(1<<SPI_SS_PIN); // SS off
   
   SPI_RAM_DDR &= ~(1<<SPI_RAM_CS_PIN); // RAM-CS-PIN off
   SPI_EE_DDR &= ~(1<<SPI_EE_CS_PIN); // EE-CS-PIN off
   
}


void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

// http://www.co-pylit.org/courses/COSC2425/lectures/AVRNetworks/index.html





void timer1_init(void)
{
   /*
    OSZI_C_LO;
    // Quelle http://www.mikrocontroller.net/topic/103629
    
    OSZI_A_HI ; // Test: data fuer SR
    _delay_us(5);
    //#define FRAME_TIME 20 // msec
    KANAL_DDR |= (1<<KANAL_PIN); // Kanal Ausgang
    
    DDRD |= (1<<PORTD5); //  Ausgang
    PORTD |= (1<<PORTD5); //  Ausgang
    
    //TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
    //TCCR1B = (1<<WGM13) | (1<<WGM12) ;		// TOP = ICR1, clk = sysclk/8 (->1us)
    TCCR1B |= (1<<CS11);
    TCNT1  = 0;														// reset Timer
    
    // Impulsdauer
    OCR1B  = 0x80;				// Impulsdauer des Kanalimpulses
    
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
    
     */
   
   
   
} // end timer1

void timer1_stop(void)
{
   // TCCR1A = 0;
   
}


/*
 
 ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
 {
 
 KANAL_HI;
 impulscounter++;
 
 if (impulscounter < ANZ_POT)
 {
 // Start Impuls
 
 TCNT1  = 0;
 //KANAL_HI;
 
 // Laenge des naechsten Impuls setzen
 
 //OCR1A  = POT_FAKTOR*Pot_Array[1]; // 18 us
 //OCR1A  = POT_FAKTOR*Pot_Array[impulscounter]; // 18 us
 
 }
 }
 */

/*
 ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 ms
 {
 //OSZI_A_LO ;
 //PORTB &= ~(1<<PORTB5); // OC1A Ausgang
 //OSZI_A_HI ;
 KANAL_LO;
 
 if (impulscounter < ANZ_POT)
 {
 }
 else
 {
 timer1_stop();
 OSZI_B_HI ;
 
 }
 }
 */

void timer0 (void) // nicht verwendet
{
   // Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	TCCR0B |= (1 << CS10); // Set up timer
	OCR0A = 0x02;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//RŸcksetzen des Timers
   
}

/*
 void timer2 (uint8_t wert)
 {
 //timer2
 TCNT2   = 0;
 //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
 TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
 //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
 TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
 //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
 TCCR2A = 0x00;
 
 
 OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
 }
 */

volatile uint16_t timer2Counter=0;
volatile uint16_t timer2BatterieCounter=0;

/*
 ISR (TIMER2_OVF_vect)
 {
 timer2Counter ++;
 
 if (timer2Counter >= 0x474) // Laenge des Impulspakets 20ms teensy
 
 //	if (timer2Counter >= 0x8E8) // Laenge des Impulspakets 20ms
 {
 
 
 //potstatus |= (1<<SPI_START); // Potentiometer messen
 
 
 timer2BatterieCounter++; // Intervall fuer Messung der Batteriespannung
 if (timer2BatterieCounter >= 0xF)
 {
 adcstatus |= (1<<ADC_START); // Batteriespannung messen
 timer2BatterieCounter = 0;
 
 }
 
 timer2Counter = 0;
 ;
 }
 TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
 }
 */
/*
 ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
 {
 //		lcd_clr_line(1);
 //		lcd_puts("Timer2 Comp\0");
 TCCR20=0;
 }
 */


//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328


ISR (PCINT0_vect)
{
   
   if(INTERRUPT_PIN & (1<< MASTER_EN_PIN))// LOW to HIGH pin change, Sub ON
   {
      //OSZI_C_LO;
      
      masterstatus |= (1<<SUB_TASK_BIT); // Zeitfenster fuer Task offen
      adc_counter ++; // loest adc aus
      
   }
   else // HIGH to LOW pin change, Sub ON
   {
      //masterstatus &= ~(1<<SUB_TASK_BIT);
   }
   
}


void setMitte(void)
{
   for (uint8_t i=0;i< ANZ_POT;i++)
   {
      Mitte_Array[i] = Pot_Array[i];
   }
}


void setLevel(uint8_t level, uint8_t kanal)
{
   eeprombyteschreiben(0xF2,LEVEL_OFFSET + kanal,level);
}


uint8_t eeprombytelesen(uint16_t readadresse) // 300 us ohne lcd_anzeige
{
   OSZI_B_LO;
   cli();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   spi_start();
   SPI_PORT_Init();
   spieeprom_init();
   
   //lcd_gotoxy(1,0);
   //lcd_putc('r');
   //lcd_putint12(readadresse);
   //lcd_putc('*');
   
   eeprom_indata = 0xaa;
   uint8_t readdata=0;
   
   // Byte  read 270 us
   EE_CS_LO;
   _delay_us(EE_READ_DELAY);
   
   readdata = (uint8_t)spieeprom_rdbyte(readadresse);
   
   _delay_us(EE_READ_DELAY);
   EE_CS_HI;
   
   sendbuffer[0] = 0xD5;
   
   sendbuffer[1] = readadresse & 0xFF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   sendbuffer[3] = readdata;
   
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   abschnittnummer =0;
   
   usb_rawhid_send((void*)sendbuffer, 50);
   
   sei();
   OSZI_B_HI;
   //lcd_putc('*');
   return readdata;
}

uint8_t eeprompartlesen(uint16_t readadresse) //   us ohne lcd_anzeige
{
   OSZI_B_LO;
   cli();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   spi_start();
   
   SPI_PORT_Init();
   
   spieeprom_init();
   
   //lcd_gotoxy(0,1);
   //lcd_putint12(readadresse);
   //lcd_putc('*');
   eeprom_indata = 0xaa;
   uint8_t readdata=0;
   // Byte  read 270 us
   
   _delay_us(LOOPDELAY);
   //     OSZI_B_LO;
   
   uint8_t i=0;
   for (i=0;i<EE_PARTBREITE;i++)
   {      
      EE_CS_LO;
      _delay_us(LOOPDELAY);
         readdata = (uint8_t)spieeprom_rdbyte(readadresse+i); // 220 us
         sendbuffer[EE_PARTBREITE+i] = readdata;
         _delay_us(LOOPDELAY);
      EE_CS_HI;
      
   }
   //     OSZI_B_HI;
   
   //OSZI_C_HI;
   
   sendbuffer[0] = 0xDB;
    
   sendbuffer[1] = readadresse & 0xFF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   sendbuffer[3] = readdata;
   sendbuffer[4] = 0xDB;
   
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   abschnittnummer =0;
   
   usb_rawhid_send((void*)sendbuffer, 50);
   
   sei();
   OSZI_B_HI;
   return readdata;
}

uint8_t eeprombyteschreiben(uint8_t code, uint16_t writeadresse,uint8_t eeprom_writedatabyte) //   1 ms ohne lcd-anzeige
{
   OSZI_B_LO;
   uint8_t byte_errcount=0;
   uint8_t checkbyte=0;
   cli();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   spi_start();
   SPI_PORT_Init();
   
   //   lcd_gotoxy(3,0);
   //   lcd_putc('w');
   //   lcd_putint12(writeadresse);
   //   lcd_putc('*');
   
   spieeprom_init();
   
   // Test 131210
   
   // WREN schicken: Write ermoeglichen
   
   _delay_us(LOOPDELAY);
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   spieeprom_wren();
   _delay_us(LOOPDELAY);
   EE_CS_HI; // SS HI End
   
   //  lcd_putc('a');
   // End Test
   
   
   // Data schicken
   EE_CS_LO;
   spieeprom_wrbyte(writeadresse,eeprom_writedatabyte);
   EE_CS_HI;
   uint8_t w=0;
   
   //  while (spieeprom_read_status()) // Blockierte SPI > entfernt 131209
   {
      //     w++;
   }
   
   //   lcd_putc('c');
   
   // Kontrolle
   _delay_us(LOOPDELAY);
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   
   //checkbyte = spieeprom_rdbyte(writeadresse);
   checkbyte = eeprombytelesen(writeadresse);
   
   //   lcd_putc('d');
   _delay_us(LOOPDELAY);
   EE_CS_HI;
 
   //lcd_putc('*');
   
   if ((eeprom_writedatabyte - checkbyte)||(checkbyte - eeprom_writedatabyte))
   {
      byte_errcount++;
      eeprom_errcount ++;
   }
   //   lcd_putc('e');
   
   lcd_gotoxy(0,1);
   lcd_putc('e');
   
   lcd_puthex(byte_errcount);
   lcd_putc(' ');
   lcd_puthex(eeprom_writedatabyte);
   lcd_putc(' ');
   lcd_puthex(checkbyte);
   
   
   sendbuffer[1] = writeadresse & 0xFF;
   sendbuffer[2] = (writeadresse & 0xFF00)>>8;
   sendbuffer[3] = byte_errcount;
   sendbuffer[4] = eeprom_writedatabyte;
   sendbuffer[5] = checkbyte;
   sendbuffer[6] = w;
   sendbuffer[7] = 0x00;
   sendbuffer[8] = 0xF9;
   sendbuffer[9] = 0xFA;
   
   sendbuffer[0] = code;
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_WRITE_BYTE_TASK);
   
   //lcd_putc('+');
   usb_rawhid_send((void*)sendbuffer, 50);
   //lcd_putc('+');
   
   sei();
   // end Daten an EEPROM
   //OSZI_D_HI ;
   OSZI_B_HI;
   return byte_errcount;
}

uint16_t eeprompartschreiben(void) // 23 ms
{
   OSZI_B_LO;
   spi_start();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   uint16_t result = 0;
   
   eeprom_errcount=0;
   
   cli();
   //OSZI_D_LO ;
   SPI_PORT_Init();
   
   uint16_t abschnittstartadresse = eepromstartadresse ; // Ladeort im EEPROM
   
   /*
   lcd_gotoxy(4,1);
   lcd_putint12(abschnittstartadresse);
   lcd_putc(' ');
   lcd_puthex(eeprombuffer[32]);
   lcd_puthex(eeprombuffer[33]);
   lcd_putc(' ');
   lcd_puthex(eeprombuffer[34]);
   lcd_puthex(eeprombuffer[35]);
   */
   
   spieeprom_init();
   _delay_us(5);
   
   // WREN schicken 220 us
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   spieeprom_wren();
   _delay_us(LOOPDELAY);
   EE_CS_HI; // SS HI End
   _delay_us(LOOPDELAY);
   
   uint8_t w=0;
   uint8_t i=0;
   for (i=0;i<EE_PARTBREITE;i++)
   {
      uint16_t tempadresse = abschnittstartadresse+i;
      uint8_t databyte = eeprombuffer[EE_PARTBREITE+i]& 0xFF; // ab byte 32
      {
         sendbuffer[EE_PARTBREITE+i] = databyte;
      }
      result += databyte;
       
      _delay_us(LOOPDELAY);

      // WREN schicken 220 us
      EE_CS_LO;
      _delay_us(LOOPDELAY);
      spieeprom_wren();
      _delay_us(LOOPDELAY);
      EE_CS_HI; // SS HI End
      _delay_us(LOOPDELAY);
      
      EE_CS_LO;
      _delay_us(LOOPDELAY);
      
      // Byte 0-31: codes
      // Byte 32-63: data
      
      spieeprom_wrbyte(tempadresse,databyte); // an abschnittstartadresse und folgende

       //spieeprom_wrbyte(0,13); // an abschnittstartadresse und folgende
      _delay_us(LOOPDELAY);
      
//      while (spieeprom_read_status())
      {
//         w++;
      };
      
      EE_CS_HI; // SS HI End
     //   while (spieeprom_read_status());
      _delay_us(LOOPDELAY);
      // Byte  read 270 us
      EE_CS_LO;
      _delay_us(LOOPDELAY);
      //     OSZI_B_LO;
      _delay_us(LOOPDELAY);
      
      eeprom_indata = (uint8_t)spieeprom_rdbyte(tempadresse);
      kontrollbuffer[EE_PARTBREITE+i] = eeprom_indata;
      
      //eeprom_indata = (uint8_t)spieeprom_rdbyte(0);
      _delay_us(LOOPDELAY);
      EE_CS_HI;
 
      if ((databyte - eeprom_indata)||(eeprom_indata - databyte))
      {
         eeprom_errcount++;
      }
   }
   _delay_us(LOOPDELAY);
   
   EE_CS_HI; // SS HI End
   
   /*
    for (i=0;i<64;i++)
    {
    // Data schicken 350 us
    EE_CS_LO;
    _delay_us(LOOPDELAY);
    spieeprom_wrbyte(eeprom_testaddress, eeprom_testdata);
    _delay_us(LOOPDELAY);
    EE_CS_HI; // SS HI End
    }
    */
   _delay_us(5);
   
   /*
    // Byte  read 270 us
    EE_CS_LO;
    _delay_us(LOOPDELAY);
    //     OSZI_B_LO;
    _delay_us(LOOPDELAY);
    eeprom_indata = (uint8_t)spieeprom_rdbyte(eeprom_testaddress);
    _delay_us(LOOPDELAY);
    //     OSZI_B_HI;
    EE_CS_HI;
    //OSZI_C_HI;
    
    
    
    lcd_gotoxy(0,1);
    //lcd_putc('*');
    lcd_puthex(eeprom_testdata);
    
    lcd_putc(' ');
    lcd_puthex(eeprom_indata);
    lcd_putc('e');
    if ((eeprom_testdata - eeprom_indata)||(eeprom_indata-eeprom_testdata))
    {
    eeprom_errcount++;
    lcd_putc('!');
    }
    //lcd_putc(' ');
    
    lcd_puthex(eeprom_errcount);
    lcd_putc('e');
    */
   //lcd_puthex(eeprom_testdata-eeprom_indata);
   //lcd_puthex(eeprom_indata - eeprom_testdata);
   //lcd_putc(' ');
   /*
    */
   //lcd_putc('*');
   //lcd_puthex(abschnittnummer);
   
   
   //sendbuffer[12] = eeprom_testdata;
   /*
    lcd_gotoxy(0,1);
    
    lcd_puthex(buffer[0]);
    lcd_putc('$');
    lcd_puthex(buffer[1]);
    lcd_putc('$');
    lcd_puthex(buffer[2]);
    lcd_putc('$');
    lcd_puthex(buffer[3]);
    lcd_putc('$');
    */
   
   kontrollbuffer[0] = 0xCB;
   kontrollbuffer[1] = abschnittstartadresse & 0xFF;
   kontrollbuffer[2] = (abschnittstartadresse & 0xFF00)>>8;
   kontrollbuffer[3] = eeprom_errcount;
   kontrollbuffer[8] = 0xA1;
   kontrollbuffer[9] = 0xA2;
    
   usb_rawhid_send((void*)kontrollbuffer, 50);
   
   sei();
   // end Daten an EEPROM
   //OSZI_D_HI ;
   
  // MEM_EN_PORT |= (1<<MEM_EN_PIN);
   OSZI_B_HI;
   return result;
}

// MARK: readSettings
void readSettings(void)
{
   RAM_CS_LO;
   _delay_us(RAMDELAY);
   /*
    modelindex = buffer[3]; // welches model soll gelesen werden
    uint8_t pos=0;
    
    // Level lesen
    uint16_t readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
    // startadresse fuer Settings des models
    for (pos=0;pos<8;pos++)
    {
    sendbuffer[EE_PARTBREITE + pos] = eeprombytelesen(readstartadresse+pos);
    }
    
    // Expo lesen
    readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
    for (pos=0;pos<8;pos++)
    {
    sendbuffer[EE_PARTBREITE + 0x08 + pos] = eeprombytelesen(readstartadresse+pos);
    }
    
    // Mix lesen
    readstartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
    for (pos=0;pos<8;pos++)
    {
    sendbuffer[EE_PARTBREITE + 0x10 + pos] = eeprombytelesen(readstartadresse+pos);
    
    }
    */
   _delay_us(RAMDELAY);
   
   RAM_CS_HI;
}



// MARK:  - main
int main (void)
{
   int8_t r;
   
   uint16_t count=0;
   
	// set for 16 MHz clock
	CPU_PRESCALE(0);
   
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
   
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(100);
   
	sei();
	Master_Init();
   
   volatile    uint8_t outcounter=0;
   volatile    uint8_t testdata =0x00;
   volatile    uint8_t testaddress =0x00;
   volatile    uint8_t errcount =0x00;
   volatile    uint8_t ram_indata=0;
   
   volatile    uint8_t eeprom_indata=0;
   volatile    uint8_t eeprom_testdata =0x00;
   volatile    uint8_t eeprom_testaddress =0x00;
   volatile    uint8_t eeprom_errcount =0x00;
   
   
   //MCP3208_spi_Init();
   
   // SPI_RAM_PORT &= ~(1<<SPI_RAM_CS_PIN);
   // _delay_us(10);
   
   // spiram_init();
   
   // SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);
   // _delay_us(10);
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);
   
	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	initADC(0);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint8_t loopcount1=0;
	/*
    Bit 0: 1 wenn wdt ausgelšst wurde
    */
	uint8_t i=0;
   
   sei();
   
   PWM = 0;
   
   char* versionstring = (char*) malloc(4);
   strncpy(versionstring, VERSION+9, 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi(versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   uint8_t anzeigecounter=0;
   uint8_t ind = 32;
   
   masterstatus |= (1<<SUB_READ_EEPROM_BIT); // sub soll EE lesen
   
// MARK:  while
	while (1)
	{
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
		if (loopcount0==0xAFFF)
		{
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         
         if (loopcount1%8 == 0)
         {
            if (masterstatus & (1<<SUB_READ_EEPROM_BIT)) // beim Start ee lesen
            {
               masterstatus &= ~(1<<SUB_READ_EEPROM_BIT);
               // Beim Start RAM_SEND_PPM_STATUS schicken
               task_out |= (1<< RAM_SEND_PPM_TASK);
               task_outdata = 0;
            }
         }
         
          if(loopcount1%16 == 0)
         {
            anzeigecounter = 0;
            if (anzeigecounter)
            {
               if (anzeigecounter > 10)
               {
                  anzeigecounter=0;
                  ind = 32;
               }
               else
               {
                  lcd_gotoxy(0,1);
                  lcd_puthex(sendbuffer[32]);
                  lcd_putc(' ');
                  lcd_puthex(sendbuffer[33]);
                  lcd_putc(' ');
                  lcd_puthex(sendbuffer[34]);
                  lcd_putc(' ');
                  lcd_puthex(sendbuffer[35]);
                  lcd_putc(' ');
                  
                  
                  /*
                  
                  lcd_puthex(eeprombuffer[0]);
                  lcd_putc('*');
                  lcd_putint(eepromstartadresse);
                  lcd_putc('*');
                  
                  lcd_puthex(eeprombuffer[3]);
                  lcd_puthex(eeprombuffer[4]);
                  
                  lcd_gotoxy(0,1);
                  
                  lcd_puthex(eeprombuffer[ind+0]);
                  lcd_putc(' ');
                  lcd_puthex(eeprombuffer[ind+1]);
                  lcd_putc(' ');
                  lcd_puthex(eeprombuffer[ind+2]);
                  lcd_putc(' ');
                  lcd_puthex(eeprombuffer[ind+3]);
                  lcd_putc(' ');
                  lcd_puthex(eeprombuffer[ind+4]);
                  lcd_putc(' ');
                  lcd_puthex(eeprombuffer[ind+5]);
                  lcd_putc(' ');
                  ind += 6;
                   */
                  anzeigecounter++;
                  
                   
               }
               /*
               uint16_t wert = eeprombuffer[ind+0]+(eeprombuffer[ind+1]<<8);
               
               lcd_putint12(wert);
               wert = eeprombuffer[ind+2]+(eeprombuffer[ind+3]<<8);
               lcd_putc(' ');
               lcd_putint12(wert);
               wert = eeprombuffer[ind+4]+(eeprombuffer[ind+5]<<8);
               lcd_putc(' ');
               lcd_putint12(wert);
               wert = eeprombuffer[ind+6]+(eeprombuffer[ind+7]<<8);
               lcd_putc(' ');
               lcd_putint12(wert);
                */
               /*
               lcd_putc('*');
               lcd_puthex(buffer[ind+0]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+1]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+2]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+3]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+4]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+5]);
               lcd_putc('*');
                */
               //sei();
            }
         }
// MARK:  USB send
         // neue Daten abschicken
//         if ((usbtask & (1<<EEPROM_WRITE_PAGE_TASK) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         OSZI_C_LO;
         uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
         OSZI_C_HI;
         if ((masterstatus & (1<< HALT_BIT) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         {
            // Write im Gang, nichts senden
            masterstatus |= (1<<SUB_TASK_BIT);
         
         }
         else
         {
                           //OSZI_C_LO;
            //uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
                     //OSZI_C_HI;
         }
      } // if loopcount0
      
      /**	ADC	***********************/
      
      if (adc_counter > 0x000F) // ADC starten
      {
         adc_counter =0;
         
         Batteriespannung = adc_read(0);
         
      }
      
      if ((masterstatus & (1<<SUB_TASK_BIT) ) )//|| (masterstatus & (1<< HALT_BIT)))// SPI starten, in PCINT0 gesetzt
      {
         if (masterstatus & (1<< HALT_BIT)) // SUB_TASK_BIT nicht zuruecksetzen
         {
            //masterstatus &= ~(1<< HALT_BIT);
         }
         //else
         {
            masterstatus &= ~(1<<SUB_TASK_BIT); // SUB nur fuer Fenster vom Master oeffnen
         }
         
         //OSZI_C_HI;
         _delay_us(1);
         uint8_t i=0;
         
         // SPI fuer device einschschalten
         spi_start();
         
         // Daten an RAM oder an EEPROM
         
         MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
         
         _delay_us(1);
         SPI_PORT_Init();


         if (usbtask & (1<<EEPROM_WRITE_BYTE_TASK))// MARK:  EEPROM_WRITE_BYTE_TASK
         {
            
         }
         else if (usbtask & (1<<EEPROM_READ_BYTE_TASK))// MARK:  EEPROM_READ_BYTE_TASK
         {
         }

          else// MARK:  SPI_RAM

         {
            SPI_RAM_init();
            spiram_init();
            
            _delay_us(1);
// MARK: F0
            // Daten von Potentiometern vom RAM lesen und senden
            
            sendbuffer[0] = 0xF0;
            for (i=0;i< 8;i++)
            {
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               sendbuffer[1+2*i] = spiram_rdbyte(2*i); // LO
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               sendbuffer[1+2*i+1] = spiram_rdbyte(2*i+1); // HI
               RAM_CS_HI;
               
            }
            anzeigecounter = 0;
            
            // MARK: task_in
            // task von PPM lesen
            _delay_us(LOOPDELAY);
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            task_in = spiram_rdbyte(READ_TASKADRESSE); // Task
            RAM_CS_HI;
            _delay_us(LOOPDELAY);
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            task_indata = spiram_rdbyte(READ_TASKDATA); // Task AData
            RAM_CS_HI;
            
            
            // Batteriespannung senden
            sendbuffer[0x3E] = Batteriespannung & 0x00FF; // LO
            sendbuffer[0x3F] = (Batteriespannung & 0xFF00) >>8; // HI
            
            
            // statusregister schreiben
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            spiram_write_status(0x00);
            _delay_us(LOOPDELAY);
            RAM_CS_HI; // SS HI End
            _delay_us(1);
            
            // err
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            //      OSZI_A_LO;
            spiram_wrbyte(0, errcount);
            //     OSZI_A_HI;
            RAM_CS_HI;
            
            _delay_us(1);

            // MARK: task_out
            
            if (task_out & (1<<RAM_SEND_PPM_TASK)) // task an PPM senden
               {
                   RAM_CS_LO;
                   _delay_us(LOOPDELAY);
                   //      OSZI_A_LO;
                   spiram_wrbyte(WRITE_TASKADRESSE, task_out);
                   //     OSZI_A_HI;
                   RAM_CS_HI;
                   RAM_CS_LO;
                   _delay_us(LOOPDELAY);
                   //      OSZI_A_LO;
                   spiram_wrbyte(WRITE_TASKDATA, task_outdata);
                   //     OSZI_A_HI;
                   RAM_CS_HI;
                   _delay_us(1);
                  
                  /*
                  lcd_gotoxy(0,1);
                  lcd_puthex(task_out);
                  lcd_putc('t');
                  lcd_puthex(task_outdata);
                  lcd_putc('t');
                   */
                  task_out &= ~(1<<RAM_SEND_PPM_TASK); // Bit reset
                }

            
            // Fehler ausgeben
            if (outcounter%0x40 == 0)
            {
               lcd_gotoxy(0,0);
                lcd_putint1(errcount);
               //lcd_putc('+');
               testdata++;
               testaddress = 32;
               //testaddress--;
               
            }
            outcounter++;
            _delay_us(LOOPDELAY);
            
            // end Daten an RAM
            
            MEM_EN_PORT |= (1<<MEM_EN_PIN);
            
            // EEPROM Test
            //
            sei();
         }
         spi_end(); // SPI von Sub ausschalten
         
         //         MASTER_PORT |= (1<<SUB_BUSY_PIN); // Sub schickt ende busy an Master
         
      } // end Task
      else
      {
         
      }
      
      /**	END ADC	***********************/
      
      /**	Begin USB-routinen	***********************/
// MARK USB read
      // Start USB
      //OSZI_D_LO;
      r = usb_rawhid_recv((void*)buffer, 0); // 2us
      //OSZI_D_HI;
      // MARK: USB_READ
      if (r > 0)
      {
         
         OSZI_D_LO;
         cli();
         uint8_t code = 0x00;
         usbstatus |= (1<<USB_RECV);
         {
            code = buffer[0];
            
            switch (code)
            {
                  
               case 0xC0: // Write EEPROM Page start
               {
                  //MASTER_PORT &= ~(1<<SUB_BUSY_PIN);
                  eeprom_errcount=0;
                  abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  anzahlpakete = buffer[3];
                  //eepromstatus |= (1<<EE_WRITE);
                  lcd_gotoxy(8,0);
                  lcd_putc('E');
                  lcd_puthex(code);
                  lcd_putc('*');
                  lcd_putint(anzahlpakete);
                  lcd_putc('*');
                  
                  sendbuffer[0] = 0xC1;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  lcd_putc('*');
                  lcd_gotoxy(18,1);
                  lcd_putc('W');
                  lcd_putc('P');
                  
                  usbtask |= (1<<EEPROM_WRITE_PAGE_TASK);
                  masterstatus |= (1<<SUB_TASK_BIT);
               }break;
                  
// MARK: C4 Write EEPROM Byte
               case 0xC4: // Write EEPROM Byte  // 10 ms
               {
                  //OSZI_A_TOGG;
                  //MASTER_PORT &= ~(1<<SUB_BUSY_PIN);
                  eeprom_errcount=0;
                  //abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  eeprom_databyte = buffer[3];
                  
                  //anzahlpakete = buffer[3];
                  //eepromstatus |= (1<<EE_WRITE);
                  lcd_gotoxy(18,1);
                  lcd_putc('W');
                  lcd_putc('1');
                  //lcd_putc('*');
                  
                  uint16_t check = eeprombyteschreiben(0xE5,eepromstartadresse,buffer[3]); // 8 ms
                  
                  
                  sendbuffer[0] = 0xC5;
                  
                  sendbuffer[1] = eepromstartadresse & 0xFF;
                  sendbuffer[2] = (eepromstartadresse & 0xFF00)>>8;
                  
                  sendbuffer[3] = buffer[3];
                  sendbuffer[4] = check;// ist bytechecksumme
                  sendbuffer[5] = eeprom_errcount;
                  
                  sendbuffer[6] = 0xFF;
                  sendbuffer[7] = 0xFF;
                  
                  sendbuffer[8] = 0xF8;
                  sendbuffer[9] = 0xFB;
                  
                  usb_rawhid_send((void*)sendbuffer, 50);

                  masterstatus |= (1<<SUB_TASK_BIT);
                  //usbtask |= (1<<EEPROM_WRITE_BYTE_TASK);
                  
               }break;
                  
               case 0xC6: // Ausgabe von Daten Start
               {
                  //abschnittnummer++;
                  //lcd_gotoxy(16,1);
                  //lcd_putc('A');
                  //lcd_gotoxy(17+abschnittnummer,0);
                  //lcd_putint1(abschnittnummer);
                  {
                     if (abschnittnummer==0)
                     {
                        eepromstartadresse = buffer[1] | (buffer[2]<<8);
                        inchecksumme = buffer[3] | (buffer[4]<<8);
                        //anzahlpakete = buffer[3];
                     }
                     uint8_t index=0;
                     //eeprombuffer[4] = buffer[4];
                     //eeprombuffer[5] = buffer[5];
                     for (index=0;index<USB_DATENBREITE;index++)
                     {
                       // eeprombuffer[index] = buffer[index];
                        //sendbuffer[index] = buffer[index];
                     }
                     if (abschnittnummer==0)
                     {
                     //eepromstartadresse = eeprombuffer[1] | (eeprombuffer[2]<<8);
                     }
                     
                     //lcd_gotoxy(0,0);
                     //for (index=0;index<8;index++)
                     {
                        //lcd_puthex(eeprombuffer[index]);
                        
                     }

                   }
                  
                  usbtask |= (1<<EEPROM_AUSGABE_TASK);
                  
               
               }break;
                  
 // MARK: CA EEPROM Part schreiben
               case 0xCA: // EEPROM Part schreiben 5 ms
               {
                 // lcd_gotoxy(18,1);
                 // lcd_putint2(32);
                  uint8_t index;
                  bytechecksumme=0;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  /*
                  lcd_gotoxy(4,0);
                  lcd_putint12(eepromstartadresse);
                  lcd_putc(' ');
                  lcd_puthex(buffer[32]);
              
                  lcd_puthex(buffer[33]);
                  lcd_putc(' ');
                  lcd_puthex(buffer[34]);
               
                  lcd_puthex(buffer[35]);
                  */
                  
                  
                   inchecksumme = buffer[3] | (buffer[4]<<8);
                  // Byte 0-31: codes
                  // Byte 32-63: data
                  
                  for (index=0;index<USB_DATENBREITE;index++)
                  {
                     eeprombuffer[index] = buffer[index];
                     if (index > EE_PARTBREITE) // ab 32
                     {
                        bytechecksumme+= buffer[index]; // bytes aufaddieren
                        //sendbuffer[index] = buffer[index];
                     }
                  }
                  
                  
                  uint16_t erfolg =  eeprompartschreiben(); // return bytechecksumme
                  
                  sendbuffer[0] = 0xEC;
                  
                  sendbuffer[1] = eepromstartadresse & 0xFF;
                  sendbuffer[2] = (eepromstartadresse & 0xFF00)>>8;
                  
                  sendbuffer[3] = eeprom_errcount;
                  sendbuffer[4] = erfolg & 0xFF;// ist bytechecksumme
                  sendbuffer[5] = (erfolg & 0xFF00)>>8;
                  
                  sendbuffer[6] = bytechecksumme & 0x00FF; // aus Eingabe
                  sendbuffer[7] = (bytechecksumme & 0xFF00)>>8;
                  
                   sendbuffer[8] = 0xF9;
                  sendbuffer[9] = 0xFA;

                  usb_rawhid_send((void*)sendbuffer, 50);

             //     anzeigecounter=1;
                  masterstatus |= (1<<SUB_TASK_BIT);
               
               }break;
                  
                
                  
               case 0xA2: // writeUSB
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('U');
                  sendbuffer[0] = 0xA3;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
               }break;
                  
                  
 // MARK: D4 read 1 EEPROM Byte
               case 0xD4: // read 1 EEPROM  // 9ms
               {
                  //abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  //sendbuffer[0] = 0xD5;
                  //usb_rawhid_send((void*)sendbuffer, 50);
                  lcd_gotoxy(18,1);
                  lcd_putc('R');
                  lcd_putc('1');
                  
                  eeprombytelesen(eepromstartadresse); // 7 ms
                  //usbtask |= (1<<EEPROM_READ_BYTE_TASK);
                  
               }break;

               case 0xDA: // read  EEPROM Part // 9ms
               {
                  //abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  //sendbuffer[0] = 0xD5;
                  //usb_rawhid_send((void*)sendbuffer, 50);
                  lcd_gotoxy(18,1);
                  lcd_putc('D');
                  lcd_putc('A');
                  
                  eeprompartlesen(eepromstartadresse); // 7 ms
                  //usbtask |= (1<<EEPROM_READ_BYTE_TASK);
                  
               }break;
                  
// MARK: F4 Fix Settings
               case 0xF4: // Fix Settings
               {
                  /*
                   pro Kanal:
                   art = 0;      Offset: 2   EXPO_OFFSET
                   expoa = 0;    Offset: 0
                   expob = 2;    Offset: 4
                   go = 1;
                   kanal = 0;
                   levela = 1;   Offset: 0   LEVEL_OFFSET
                   levelb = 1;   Offset: 4
                   
                   nummer = 0;
                   richtung = 0; Offset: 7  
                   state = 1;
                   */
                 // uint16_t fixstartadresse =  buffer[1] | (buffer[2]<<8);
                  lcd_gotoxy(0,1);
                  //lcd_putc('A');
                  uint8_t changecode = buffer[3];// Bits fuer zu aendernde kanaele
                  uint8_t modelindex = buffer[4]; // Nummer des models
                  uint16_t fixstartadresse =  TASK_OFFSET + modelindex * SETTINGBREITE; // Startadresse fuer Settings
                  
                  uint8_t datastartbyte = 16; // Beginn  der Settings auf dem buffer
                  uint8_t errcount0=0;
                  uint8_t errcount1=0;
                  uint8_t changeposition=0; // position des bytes im sendbuffer. Nur zu aendernde bytes sind darin.
                  //lcd_putc('X');
                  //changecode |= (1<<1);
                  for (uint8_t kanal=0;kanal < 8;kanal++)
                  {
                     //lcd_putc('A'+kanal);
                     if (changecode & (1<<kanal)) // kanal ist zu aendern
                     {
                        //lcd_putc('A'+kanal);
                        
                        // Level schreiben
                        uint8_t levelwert = buffer[datastartbyte + 2*changeposition + 1]; // wert an position im buffer // 0x20
                         //lcd_putint(levelwert);
                        //levelwert = 1;
 
      
                      errcount0 += eeprombyteschreiben(0xF5,fixstartadresse + LEVEL_OFFSET + kanal,levelwert); // adresse im EEPROM
               
                        //       lcd_putint(errcount);
                  /*
                         sendbuffer[EE_PARTBREITE + 2*kanal+1] = levelwert;
                        */
                        //lcd_putc('A'+kanal);
                        // Expo schreiben
                        
                        uint8_t expowert = buffer[datastartbyte + 2*changeposition];
                        
                        //Test
                       // expowert= 0;
                        //
                        
                        errcount1 += eeprombyteschreiben(0xF5,fixstartadresse + EXPO_OFFSET + kanal,expowert); // 0x30
                        
                        sendbuffer[EE_PARTBREITE + 2*kanal] = expowert;
                        
                        //lcd_putc('A'+kanal);
                        //lcd_putc('*');
                        /*
                        lcd_gotoxy(0,1);
                        lcd_putc('k');
                        lcd_putint2(kanal);
                        lcd_putc('w');
                        lcd_puthex(levelwert);
                        lcd_putc('e');
                        lcd_puthex(expowert);
                        lcd_putc('*');
                         */
                        sendbuffer[0x0F+2*kanal] = levelwert;
                        sendbuffer[0x0F+2*kanal+1] = expowert;

                        changeposition++;
                     }
                     //lcd_putc('M'+kanal);
                     // RAM_SEND_PPM_STATUS schicken: Daten haben geaendert
                     task_out |= (1<< RAM_SEND_PPM_TASK);
                     task_outdata = modelindex;
                     
                  }// for kanal
                  //lcd_putc('C');
                  sendbuffer[1] = fixstartadresse & 0x00FF;
                  sendbuffer[2] = (fixstartadresse & 0xFF00)>>8;
                  sendbuffer[3] = errcount0;
                  sendbuffer[4] = changecode;
                  sendbuffer[5] = modelindex;
                  sendbuffer[6] = task_out;
                  sendbuffer[7] = task_outdata;
                  sendbuffer[8] = errcount1;
                  

                  sendbuffer[0] = 0xF4;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
                  //lcd_putc('D');
                  
               }break;
                  
// MARK: F5 read KanalSettings
               case 0xF5: // read KanalSettings
               {
                  uint8_t modelindex =0;
                  modelindex = buffer[3]; // welches model soll gelesen werden
                  uint8_t pos=0;
                  
                  // Level lesen
                  uint16_t readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
                  // startadresse fuer Settings des models
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x20 32
                  }
                  
                  // Expo lesen
                  readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x08 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x28 40
                  }
                 
                  // Mix lesen
                  readstartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x10 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x30 48
                  
                  }
                  
                  
                  
                  sendbuffer[1] = readstartadresse & 0x00FF;
                  sendbuffer[2] = (readstartadresse & 0xFF00)>>8;
                  sendbuffer[3] = modelindex;
                  sendbuffer[0] = 0xF5;
                  usb_rawhid_send((void*)sendbuffer, 50);
                
               
               
               }break;
                  
                  
               case 0xF6: // HALT
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('H');
                  masterstatus |= (1<<HALT_BIT); // Halt-Bit aktiviert Task bei ausgeschaltetem Slave
                  MASTER_PORT &= ~(1<<SUB_BUSY_PIN);
                  abschnittnummer=0;
                  
               }break;
                  
               case 0xF7: // GO
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('G');
                  masterstatus &= ~(1<<HALT_BIT);
                  MASTER_PORT |= (1<<SUB_BUSY_PIN);
                  abschnittnummer=0;

               }break;

// MARK: FA Fix Mixing
               case 0xFA: // Fix Mixing
               {
                  /*
                   pro mixing:
                   mixart: typ
                   mixcanals: wer mit welchem Kanal Bits 0-2: Kanal A,  Bits 4-6: Kanal B
                   */
                  
                  uint8_t changecode = buffer[3];// Bits fuer zu aendernde mixes
                  uint8_t modelindex = buffer[4];// model
                  uint16_t fixstartadresse =  TASK_OFFSET + modelindex * SETTINGBREITE; // Startadresse fuer Settings
                  uint8_t datastartbyte = 16; // Beginn  der Settings auf dem buffer
                  uint8_t errcount=0;
                  uint8_t changeposition=0; // position des bytes im sendbuffer. Nur zu aendernde bytes sind darin.
                  uint8_t writeposition=0; // schreibposition im EEPROM
                  uint8_t schleifencounter=0;
                  for (uint8_t mixing=0;mixing < 4;mixing++) //
                  {
                     if (changecode & (1<<mixing)) // mixing ist zu aendern
                     {
                        // mixwert schreiben
                        
                        uint8_t mixwert = buffer[datastartbyte + 2*changeposition];
                        errcount += eeprombyteschreiben(0xFB,fixstartadresse + MIX_OFFSET + 2*mixing,mixwert);
                        
                        sendbuffer[EE_PARTBREITE + 2*mixing] = mixwert;
                        sendbuffer[EE_PARTBREITE + 16 + 2*mixing] = (fixstartadresse + MIX_OFFSET + writeposition)&0x00FF;
                        sendbuffer[EE_PARTBREITE + 16 + 2*mixing+1] = ((fixstartadresse + MIX_OFFSET + writeposition)&0xFF00)>>8;
                        
                        writeposition++;
                        
                        uint8_t canalwert = (buffer[datastartbyte + 2*changeposition+1]);
                        
                        errcount += eeprombyteschreiben(0xFB,fixstartadresse + MIX_OFFSET + 2*mixing+1,canalwert);  // 0x40
                        
                        sendbuffer[EE_PARTBREITE + 2*mixing+1] = canalwert;
                        writeposition++;
                        changeposition++;
                     }
                     
                  }// for mixing
                  
                  // RAM_SEND_PPM_STATUS schicken: Daten haben geaendert
                  
                  task_out |= (1<< RAM_SEND_PPM_TASK);
                  task_outdata = modelindex;
                  
                  sendbuffer[1] = fixstartadresse & 0x00FF;
                  sendbuffer[2] = (fixstartadresse & 0xFF00)>>8;
                  sendbuffer[3] = errcount;
                  sendbuffer[4] = changecode;
                  sendbuffer[5] = modelindex;
                  
                  sendbuffer[0] = 0xFA;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
                  
               }break;
                  
                 case 0xFC: // Refresh
               {
                  
                  // RAM_SEND_PPM_STATUS schicken: Daten haben geaendert
                  task_out |= (1<< RAM_SEND_PPM_TASK);
                  uint8_t modelindex = buffer[4];// model

                  task_outdata = modelindex;

                  sendbuffer[0] = 0xFC;
                  usb_rawhid_send((void*)sendbuffer, 50);

               }break;
                  
            } // switch code
         }
         /*
         else
         {
            // Data
            //lcd_clr_line(1);
            //lcd_gotoxy(18,1);
            //lcd_putc('X');
            //lcd_putc('X');
            
            
            
            //abschnittnummer=0;
            OSZI_B_TOGG ;
         }
         */
         //lcd_putc('$');
         code=0;
         sei();
         
       OSZI_D_HI;
         
		} // r>0, neue Daten
      else
      {
         //OSZI_B_LO;
      }
      
      /**	End USB-routinen	***********************/
 		
		/* **** rx_buffer abfragen **************** */
		
 // MARK:  Tasten
		//	Daten von USB vorhanden
      // rxdata
		
		//lcd_gotoxy(16,0);
      //lcd_putint(StepCounterA & 0x00FF);
		
		if (!(TASTENPIN & (1<<TASTE0))) // Taste 0
		{
			//lcd_gotoxy(8,1);
			//lcd_puts("T0 Down\0");
			
			if (!(TastenStatus & (1<<TASTE0))) //Taste 0 war noch nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<TASTE0);
				
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount +=1;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
               
					Tastencount=0;
               if (TastenStatus & (1<<TASTE0))
               {
                  //sendbuffer[0]=loopcount1;
                  //sendbuffer[1]=0xAB;
                  //usbstatus |= (1<<USB_RECV);
                  //lcd_gotoxy(2,1);
                  //lcd_putc('1');
                  
                  //usb_rawhid_send((void*)sendbuffer, 50);
               }
					TastenStatus &= ~(1<<TASTE0);
               //lcd_gotoxy(3,1);
               //lcd_puts("ON \0");
               //delay_ms(400);
               //lcd_gotoxy(3,1);
               // lcd_puts("  \0");
               //lcd_putint(TastenStatus);
               
               
				}
			}//else
			
		}	// Taste 0
		
		if (!(TASTENPIN & (1<<TASTE1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("T1 Down\0");
			
			if (! (TastenStatus & (1<<TASTE1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<TASTE1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount +=1;
				if (Tastencount >= Tastenprellen)
				{
					Tastencount=0;
					TastenStatus &= ~(1<<TASTE1);
				}
			}//	else
			
		} // Taste 1
		
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
      
		//OSZI_B_HI;
      if (usbstatus & (1<< USB_RECV))
      {
      }
      
	}//while
   //free (sendbuffer);
   
   // return 0;
}
