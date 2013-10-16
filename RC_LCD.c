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

#define LOOPDELAY 50

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
volatile uint16_t Mitte_Array[ANZ_POT];


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
    
    //KANAL_PORT |= (1<<PORTB5); // Ausgang HI
    //OSZI_A_LO ;
    OSZI_B_LO ;
    
    
    KANAL_HI;
    
    impulscounter = 0;
    //OCR1A  = POT_FAKTOR*Pot_Array[1];
    //OCR1A  = POT_FAKTOR*Pot_Array[impulscounter];
    
    
    if (Pot_Array[impulscounter] > SERVOMAX)
    {
    Pot_Array[impulscounter] = SERVOMAX;
    }
    
    if (Pot_Array[impulscounter] < SERVOMIN)
    {
    Pot_Array[impulscounter] = SERVOMIN;
    }
    
    
    OCR1A  = Pot_Array[impulscounter]; // POT_Faktor schon nach ADC
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
 
 
 if (Pot_Array[impulscounter] > SERVOMAX)
 {
 Pot_Array[impulscounter] = SERVOMAX;
 }
 
 if (Pot_Array[impulscounter]<  SERVOMIN)
 {
 Pot_Array[impulscounter] = SERVOMIN;
 }
 
 OCR1A  = Pot_Array[impulscounter]; //
 }
 else
 {
 // Ende Impulspaket
 OCR1A  = 0x4FF;
 //OSZI_A_HI ;
 // _delay_us(200);
 KANAL_LO;
 
 // Alle Impulse gesendet, Timer1 stop. Timer1 wird bei Beginn des naechsten Paketes wieder gestartet
 timer1_stop();
 
 // SPI fuer device ausschalten
 spi_end();
 potstatus |= (1<<SPI_END); //
 OSZI_B_HI ;
 //OSZI_A_LO ;
 
 MASTER_PORT |= (1<<SUB_BUSY_PIN); // Sub schickt ende busy an Master
 _delay_us(2);
 //PORTE |= (1<<PORTE0);
 MEM_EN_PORT |= (1<<MEM_EN_PIN);
 OSZI_C_HI;
 }
 
 //OSZI_B_LO ;
 //_delay_us(10);
 
 
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

uint8_t eeprombytelesen(uint16_t readadresse) // 300 us ohne lcd_anzeige
{
   OSZI_B_LO;
   cli();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   spi_start();

   SPI_PORT_Init();
   
   spieeprom_init();
   
   lcd_gotoxy(1,0);
   lcd_putc('r');
   lcd_putint12(readadresse);
   lcd_putc('*');
   
   
   eeprom_indata = 0xaa;
   uint8_t readdata=0;
   
   // Byte  read 270 us
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   
   readdata = (uint8_t)spieeprom_rdbyte(readadresse);
   
   _delay_us(LOOPDELAY);
   EE_CS_HI;
   
   lcd_puthex(readdata);

   
   sendbuffer[0] = 0xD5;
   
   sendbuffer[1] = readadresse & 0xFF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;

   sendbuffer[3] = readdata;
   
   
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   
   //           MASTER_PORT |= (1<<SUB_BUSY_PIN); // busy beenden
   
   abschnittnummer =0;
   
   //lcd_putc('+');
   usb_rawhid_send((void*)sendbuffer, 50);
   //lcd_putc('+');
   
   sei();
   OSZI_B_HI;
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
   
   lcd_gotoxy(0,1);
   lcd_putint12(readadresse);
   lcd_putc('*');
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
   //lcd_puthex(eeprom_indata);
   //lcd_putc('*');
   
   sendbuffer[1] = readadresse & 0xFF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   
   sendbuffer[3] = readdata;
   
   sendbuffer[4] = 0xDB;
   
   
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   
   //           MASTER_PORT |= (1<<SUB_BUSY_PIN); // busy beenden
   
   abschnittnummer =0;
   
   //lcd_putc('+');
   usb_rawhid_send((void*)sendbuffer, 50);
   //lcd_putc('+');
   
   sei();
   OSZI_B_HI;
   return readdata;
}



uint16_t eeprombyteschreiben(uint16_t writeadresse,uint8_t eeprom_writedatabyte) //   1 ms ohne lcd-anzeige
{
   OSZI_B_LO;
   uint8_t byte_errcount=0;
   uint8_t checkbyte=0;
   cli();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   spi_start();
   SPI_PORT_Init();
   
   lcd_gotoxy(1,1);
   lcd_putc('w');
   lcd_putint12(writeadresse);
   lcd_putc('*');

   
   spieeprom_init();
   
   // statusregister schreiben
   
 /*
   // WREN
  // EE_CS_LO;
   
         _delay_us(LOOPDELAY);
         spieeprom_wren(); // 0x06
         _delay_us(LOOPDELAY);
   
 //  EE_CS_HI; // SS HI End
   
   _delay_us(LOOPDELAY);
   _delay_us(50);
  
  
   //Write status
 //  EE_CS_LO;
   
         _delay_us(LOOPDELAY);
         spieeprom_write_status(); // 0x01, 0x00
         _delay_us(LOOPDELAY);
   
//   EE_CS_HI; // SS HI End
 */  
   _delay_us(50);
   
   // Byte  write
   
   // WREN schicken 220 us
 //  EE_CS_LO;
   
         _delay_us(LOOPDELAY);
         spieeprom_wren(); // 0x06
         _delay_us(LOOPDELAY);
   
 //  EE_CS_HI; // SS HI End
   
   _delay_us(LOOPDELAY);
    _delay_us(50);
    // Byte  write
 //  EE_CS_LO;
   _delay_us(LOOPDELAY);
   
      spieeprom_wrbyte(writeadresse,eeprom_writedatabyte);
 //  EE_CS_HI;
   
   uint8_t w=0;
   
   while (spieeprom_read_status())
   {
      w++;
   };

   _delay_us(10);
  
  // EE_CS_HI; // SS HI End
      _delay_us(100);
   
   // Byte  read 270 us
 //  EE_CS_LO;
   
         _delay_us(LOOPDELAY);
         checkbyte = (uint8_t)spieeprom_rdbyte(writeadresse);
         _delay_us(LOOPDELAY);
   
 //  EE_CS_HI;
   
   if ((eeprom_writedatabyte - checkbyte)||(checkbyte - eeprom_writedatabyte))
   {
      byte_errcount++;
      eeprom_errcount ++;
   }
   
  // lcd_gotoxy(0,0);
   lcd_putc('e');
   
   lcd_puthex(byte_errcount);
   lcd_putc(' ');
   
   lcd_puthex(eeprom_writedatabyte);
   lcd_putc(' ');
   lcd_puthex(checkbyte);
   
   /*
    */
   
   
   sendbuffer[1] = writeadresse & 0xFF;
   sendbuffer[2] = (writeadresse & 0xFF00)>>8;
   sendbuffer[3] = byte_errcount;
   sendbuffer[4] = eeprom_writedatabyte;
   sendbuffer[5] = checkbyte;
   sendbuffer[6] = w;
   sendbuffer[7] = 0x00;
   sendbuffer[8] = 0xF9;
   sendbuffer[9] = 0xFA;

   
   //sendbuffer[12] = eeprom_testdata;
   
   
   sendbuffer[0] = 0xE5;
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_WRITE_BYTE_TASK);
//   MASTER_PORT |= (1<<SUB_BUSY_PIN); // busy beenden
   
   //lcd_putc('+');
   usb_rawhid_send((void*)sendbuffer, 50);
   //lcd_putc('+');
   
   sei();
   // end Daten an EEPROM
   //OSZI_D_HI ;
  // MEM_EN_PORT |= (1<<MEM_EN_PIN); // blockiert USB
   OSZI_B_HI;
   return byte_errcount;
}

uint16_t eeprompartschreiben(void) // 23 ms
{
   OSZI_B_LO;
   spi_start();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   uint16_t result = 0;
   cli();
   //OSZI_D_LO ;
   SPI_PORT_Init();
   
   uint16_t abschnittstartadresse = eepromstartadresse ; // Ladeort im EEPROM
   spieeprom_init();
   
   _delay_us(5);
   
   //OSZI_C_LO;
   // statusregister schreiben
   /*
   // WREN
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   spieeprom_wren();
   _delay_us(LOOPDELAY);
   EE_CS_HI; // SS HI End
   _delay_us(LOOPDELAY);
   
   //Write status
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   spieeprom_write_status();
   _delay_us(LOOPDELAY);
   EE_CS_HI; // SS HI End
   */
   
   _delay_us(5);
   
   // Byte  write
   
   
   // WREN schicken 220 us
   EE_CS_LO;
   _delay_us(LOOPDELAY);
   spieeprom_wren();
   _delay_us(LOOPDELAY);
   EE_CS_HI; // SS HI End
   _delay_us(LOOPDELAY);
   
   
   eeprom_errcount=0;
   
   uint8_t w=0;
   uint8_t i=0;
   for (i=0;i<EE_PARTBREITE;i++)
   {
      uint8_t tempadresse = abschnittstartadresse+i;
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
      
      while (spieeprom_read_status())
      {
         w++;
      };

      
      EE_CS_HI; // SS HI End
        while (spieeprom_read_status());
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
      
       
      //     OSZI_B_HI;
      

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

   /*
   sendbuffer[0] = 0xCB;
   
   sendbuffer[1] = abschnittstartadresse & 0xFF;
   sendbuffer[2] = (abschnittstartadresse & 0xFF00)>>8;
   
   sendbuffer[3] = eeprom_errcount;
   sendbuffer[4] = result & 0xFF;
   sendbuffer[5] = (result & 0xFF00)>>8;
   
   sendbuffer[6] = 0xFF;
   sendbuffer[7] = 0xFF;
   
   sendbuffer[8] = 0xA3;
   sendbuffer[9] = 0xA4;
   */
   
   /*
    else
    {
    sendbuffer[0] = 0xC1;
    sendbuffer[1] = abschnittnummer+1;
    sendbuffer[2] = abschnittnummer+2;
    sendbuffer[3] = abschnittnummer+3;
    sendbuffer[4] = abschnittnummer+4;
    sendbuffer[5] = 0xFF;
    sendbuffer[6] = abschnittnummer;
    
    sendbuffer[8] = abschnittstartadresse & 0xFF;
    sendbuffer[9] = (abschnittstartadresse & 0xFF00)>>8;
    sendbuffer[10] = 0xA3;
    sendbuffer[11] = 0xA4;
    
    }
    */
   
   usb_rawhid_send((void*)kontrollbuffer, 50);
   
   sei();
   // end Daten an EEPROM
   //OSZI_D_HI ;
   
  // MEM_EN_PORT |= (1<<MEM_EN_PIN);
   OSZI_B_HI;
   return result;
}


#pragma mark - main
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
   
#pragma mark while
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
         //PORTD ^= (1<<PORTD6);
         
 			//
			//timer0();
         
          if(loopcount1%16 == 0)
         {
            
            if (anzeigecounter)
            {
               if (anzeigecounter > 3)
               {
                  anzeigecounter=0;
                  ind = 32;
               }
               else
                  
               {
                  lcd_gotoxy(3,0);
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
                  
                  anzeigecounter++;
                  ind += 6;
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
         
         
 
         
         
#pragma mark USB send
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
         /*
          lcd_gotoxy(0,1);
          lcd_putint(anz);
          lcd_putc('*');
          lcd_putint(Pot_Array[0]);
          lcd_putc('*');
          lcd_putint(sendbuffer[9]);
          lcd_putc('*');
          */
         
         //
      } // if loopcount0
      
      
      /**	ADC	***********************/
      
      if (adc_counter > 0x000F) // ADC starten
      {
         adc_counter =0;
         Batteriespannung = adc_read(0);
         
         lcd_gotoxy(14,0);
         
         lcd_putint12(Batteriespannung);
         /*
          lcd_putc('*');
          uint8_t high = (Batteriespannung & 0xFF00)>>8; // *0xFF rechnen und zu low dazuzaehlen
          lcd_putint(high);
          uint8_t low = (Batteriespannung & 0xFF);
          lcd_putc('*');
          lcd_putint(low);
          //lcd_puthex(loopcount1);
          */
         ;
         
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
#pragma mark EEPROM_WRITE_BYTE_TASK
         //if (eepromstatus & (1<<EE_WRITE)) // write an eeprom
         
         if (usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         {
         }
 
#pragma mark EEPROM_READ_BYTE_TASK
         else if (usbtask & (1<<EEPROM_READ_BYTE_TASK))
         {
            cli();
            SPI_PORT_Init();
            
            spieeprom_init();
            
            lcd_gotoxy(0,1);
            lcd_putint12(eepromstartadresse);
            lcd_putc('*');
            eeprom_indata = 0xaa;
            // Byte  read 270 us
            EE_CS_LO;
            _delay_us(LOOPDELAY);
            //     OSZI_B_LO;
            _delay_us(LOOPDELAY);
            
            eeprom_indata = (uint8_t)spieeprom_rdbyte(eepromstartadresse);
            
            _delay_us(LOOPDELAY);
            //     OSZI_B_HI;
            EE_CS_HI;
            //OSZI_C_HI;
            
            lcd_puthex(eeprom_indata);
            lcd_putc('*');
            
            sendbuffer[1] = eeprom_indata;
            
            sendbuffer[0] = 0xD5;
            eepromstatus &= ~(1<<EE_WRITE);
            usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
            
            
 //           MASTER_PORT |= (1<<SUB_BUSY_PIN); // busy beenden
            
            abschnittnummer =0;
            
            //lcd_putc('+');
            usb_rawhid_send((void*)sendbuffer, 50);
            lcd_putc('+');
            
            sei();
            
         }

#pragma mark EEPROM_AUSGABE_TASK
#pragma mark EEPROM_WRITE_PAGE_TASK
#pragma mark EEPROM_READ_PAGE_TASK
         else
         {
            SPI_RAM_init();
            
            spiram_init();
            
            _delay_us(1);
            // statusregister schreiben
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            spiram_write_status(0x00);
            _delay_us(LOOPDELAY);
            RAM_CS_HI; // SS HI End
            _delay_us(1);
            
            
            // testdata in-out
            RAM_CS_LO;
            
            _delay_us(LOOPDELAY);
            //      OSZI_A_LO;
            spiram_wrbyte(testaddress, testdata);
            //     OSZI_A_HI;
            RAM_CS_HI;
            
            // Kontrolle
            _delay_us(2);
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            //     OSZI_B_LO;
            _delay_us(LOOPDELAY);
            ram_indata = spiram_rdbyte(testaddress);
            _delay_us(LOOPDELAY);
            //     OSZI_B_HI;
            RAM_CS_HI;
            
            //OSZI_A_LO ;
            
            
            
            // Daten von Potentiometern vom RAM lesen und senden
            sendbuffer[0] = 0xF0;
            for (i=0;i< 8;i++)
            {
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               sendbuffer[1+2*i] = spiram_rdbyte(2*i);
               RAM_CS_HI;
               _delay_us(LOOPDELAY);
               RAM_CS_LO;
               _delay_us(LOOPDELAY);
               sendbuffer[1+2*i+1] = spiram_rdbyte(2*i+1);
               RAM_CS_HI;
               
            }
            //OSZI_A_HI ;
            
            
            // Batteriespannung senden
            sendbuffer[18] = Batteriespannung & 0x00FF; // LO
            sendbuffer[19] = (Batteriespannung & 0xFF00) >>8; // HI
            
            
            
            
            // Fehler zaehlen
            if (!(testdata == ram_indata))
            {
               errcount++;
            }
            
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
            
            // Daten aendern
            if (outcounter%0x40 == 0)
            {
               lcd_gotoxy(0,0);
               /*
                lcd_putint12(timer2Counter);
                lcd_putc('*');
                lcd_putint(testdata);
                lcd_putc('*');
                lcd_putint(ram_indata);
                lcd_putc('+');
                */
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
#pragma mark USB read
      // Start USB
      //OSZI_D_LO;
      r = usb_rawhid_recv((void*)buffer, 0); // 2us
      //OSZI_D_HI;
      if (r > 0)
      {
         
         OSZI_D_LO;
         cli();
         uint8_t code = 0x00;
         usbstatus |= (1<<USB_RECV);
//         if (abschnittnummer == 0) // erster Abschnitt enthaelt code
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
                  
                  
                  //
                  
                  uint16_t check = eeprombyteschreiben(eepromstartadresse,buffer[3]); // 8 ms
                  
                  
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
                  
               case 0xCA: // EEPROM Part laden 5 ms
               {
                  lcd_gotoxy(18,1);
                  lcd_putint2(32);
                  uint8_t index;
                  bytechecksumme=0;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
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

                  anzeigecounter=1;
                  masterstatus |= (1<<SUB_TASK_BIT);
               
               }break;
                  
                
                  
               case 0xA2: // writeUSB
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('U');
                  sendbuffer[0] = 0xA3;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
               }break;
                  
                  
                  
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
                  
               case 0xF0:
               {
                 
                  //lcd_gotoxy(14,1);
                  //lcd_putc('S');
                  //lcd_putc(' ');
               }break;
                  
                  
#pragma mark default
               default:
               {
                  
                  if (usbtask & (1<<EEPROM_WRITE_PAGE_TASK)) // noch nicht fertig
                  {
                     //lcd_gotoxy(16,1);
                     //lcd_putc('E');
                     //lcd_puthex(abschnittnummer);
                     //lcd_putc('*');
                     //lcd_putint(buffer[0]);
                     //lcd_putint(buffer[1]);
                     
                  }
                  
                  else
                     
                  {
                     /*
                     OSZI_D_LO;
                      //lcd_gotoxy(11,1);
                      //lcd_putc('D');
                      //lcd_putc('x');
                     
                     uint8_t index;
                     for (index=0;index<USB_DATENBREITE;index++)
                     {
                        eeprombuffer[index] = buffer[index];
                     }
                     OSZI_D_HI;
                     */
                      
                  }
               }break; // default
                  
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
		
#pragma mark Tasten
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
