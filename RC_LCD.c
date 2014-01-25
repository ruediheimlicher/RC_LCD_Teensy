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

#include "font.h"
#include "display.c"

#include "text.h"


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

#define EEPROM_STARTADRESSE   0x7FF

volatile uint8_t timer0startwert=TIMER0_STARTWERT;

//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

static volatile uint8_t             displaystatus=0x00; // Tasks fuer Display


static volatile uint16_t                   displaycounter=0;





static volatile uint8_t             substatus=0x00; // Tasks fuer Sub
// Task fuer sub
#define TASTATUR_READ   0
#define TASTATUR_OK     1
#define SETTINGS_READ    2


#define SCREEN_OK       4
#define SCREEN_REFRESH  5
#define UHR_OK          6
#define UHR_REFRESH     7




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

//volatile uint8_t Level_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal
//volatile uint8_t Expo_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal

volatile uint16_t Mix_Array[8];// Mixings, 2 8-bit-bytes pro Mixing


volatile uint16_t RAM_Array[SPI_BUFSIZE];

volatile uint8_t testdataarray[8]={};
volatile uint16_t teststartadresse=0xA0;


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

// Mark Screen

#define CLOCK_DIV 15 // timer0 1 Hz bei Teilung /4 in ISR


volatile uint16_t                TastaturCount=0;
volatile uint16_t                manuellcounter=0; // Countr fuer Timeout
volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint8_t                 settingstartcounter=0; // Counter fuer Klicks auf Taste 5
volatile uint16_t                mscounter=0; // Counter fuer ms in timer-ISR
uint16_t TastenStatus=0;
uint16_t Tastencount=0;
uint16_t Tastenprellen=0x01F;
volatile uint8_t                 Taste=0;
volatile uint8_t                 programmstatus=0x00;

volatile uint8_t                 senderstatus=0x00;

volatile uint8_t levelwert=0x32;
volatile uint8_t levelb=0x12;

volatile uint8_t expowert=0;
volatile uint8_t expob=0;

/*
volatile uint8_t                 default_settingarray[8][2]=
{
   {0x12,0x21},
   {0x00,0x00},
   {0x00,0x00},
   {0x00,0x00},
   {0x00,0x00},
   {0x00,0x00},
   {0x00,0x00},
   {0x00,0x00}
};
*/
volatile uint8_t                 default_levelarray[8]=
{
   0x12,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00
};


volatile uint8_t                 default_expoarray[8]=
{
   0x21,
   0x00,
   0x00,
   0x00,
   0x04, // Schieber
   0x04,
   0x08, // Schalter
   0x08
};

volatile uint8_t                 default_mixarray[8]=
{
   // index gerade  :  mixa(parallel) mit (0x70)<<4, mixb(opposite) mit 0x07
   // index ungerade: typ mit 0x03
   0x00, // V-mix
   0x88, // OFF
   0x00,
   0x88,
   0x00,
   0x00,
   0x00,
   0x00
};

volatile uint8_t                 default_funktionarray[8]=
{
   //
   // bit 0-2: Steuerfunktion bit 4-6: Kanal von Steuerfunktion
   0x00,
   0x11,
   0x22,
   0x33,
   0x44,
   0x55,
   0x66,
   0x77
};

volatile uint8_t                 default_devicearray[8]=
{
   //
   // bit 0-2: Steuerfunktion bit 4-6: Kanal von Steuerfunktion
   0x00,
   0x01,
   0x02,
   0x03,
   0x04,
   0x05,
   0x06,
   0x07
};

volatile uint8_t                 default_ausgangarray[8]=
{
   //
   // bit 0-2: Kanal bit 4-6:
   0x00,
   0x01,
   0x02,
   0x03,
   0x04,
   0x05,
   0x06,
   0x07
};


volatile uint8_t              curr_levelarray[8];
volatile uint8_t              curr_expoarray[8];
volatile uint8_t              curr_mixarray[8]={};
volatile uint8_t              curr_funktionarray[8];
 volatile uint8_t              curr_devicearray[8] = {};
 volatile uint8_t              curr_ausgangarray[8];

volatile uint16_t                updatecounter; // Zaehler fuer Update des screens

volatile uint8_t                 curr_screen=0; // aktueller screen

volatile uint8_t                 curr_page=7; // aktuelle page
volatile uint8_t                 curr_col=0; // aktuelle colonne

volatile uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
volatile uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
volatile uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
volatile uint8_t                 last_cursorspalte=0; // letzte colonne des cursors


volatile uint8_t                 curr_model=0; // aktuelles modell
uint8_t              EEMEM       speichermodel=0;
volatile uint8_t                 curr_kanal=0; // aktueller kanal
volatile uint8_t                 curr_richtung=0; // aktuelle richtung


volatile uint8_t                 curr_setting=0; // aktuelles Setting fuer Modell
uint8_t              EEMEM       speichersetting=0;


volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor
volatile uint16_t                 blink_cursorpos=0xFFFF;

volatile uint16_t laufsekunde=0;
uint8_t  EEMEM speichersekunde;



volatile uint8_t laufminute=0;
uint8_t  EEMEM speicherminute;

volatile uint8_t laufstunde=0;
uint8_t  EEMEM speicherstunde;


volatile uint16_t motorsekunde=0;
uint8_t  EEMEM speichermotorsekunde;

volatile uint16_t motorminute=0;
uint8_t  EEMEM speichermotorminute;

volatile uint16_t stopsekunde=0;
uint8_t  EEMEM speicherstopsekunde;

volatile uint16_t stopminute=0;
uint8_t  EEMEM speicherstopminute;


volatile uint16_t batteriespannung =0;



// MARK: Proto
uint8_t eeprombyteschreiben(uint8_t code, uint16_t writeadresse,uint8_t eeprom_writedatabyte);
uint8_t eeprombytelesen(uint16_t readadresse); // 300 us ohne lcd_anzeige
uint8_t eeprompartlesen(uint16_t readadresse); //   us ohne lcd_anzeige
uint16_t eeprompartschreiben(void); // 23 ms

void read_eeprom_zeit(void);
void write_eeprom_zeit(void);


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
    TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang f�r Taste 0
    TASTENPORT |= (1<<TASTE0);	//Pull-up
    */
   
   
   /**
	 * Pin Change Interrupt enable on PCINT0 (PD7)
	 */
   
   PCIFR |= (1<<PCIF0);
   PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT7);
  
   // POWER_OFF
   
   POWER_OFF_DDR &= ~(1<<POWER_OFF_DETECT_PIN); // Eingang fuer Power_off_detection
   POWER_OFF_PORT |= (1<<POWER_OFF_DETECT_PIN); // hi
   EICRA |= (1<<ISC31); // falling edge
   EIMSK |= (1<<INTF3); // Interrupt en
   
   
   INTERRUPT_DDR &= ~(1 << MASTER_EN_PIN); // Clear the PB7 pin
   // PB7 (PCINT7 pin) are now inputs
   
   INTERRUPT_PORT |= (1 << MASTER_EN_PIN) ; // turn On the Pull-up
   // PB7 are now inputs with pull-up enabled
   
   MASTER_DDR |= (1 << SUB_BUSY_PIN); // BUSY-Pin als Ausgang
   // PB7 (PCINT7 pin) are now inputs
   
   MASTER_PORT |= (1 << SUB_BUSY_PIN) ; // turn On the Pull-up
   // PB7 are now inputs with pull-up enabled
   
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		// PIN als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
   ADC_DDR &= ~(1<<ADC_AKKUPIN);
   
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
   SPI_PORT &= ~(1<<SPI_MOSI_PIN); // LO
   
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
	//TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	
   TCCR0B |= (1 << CS02);//
   //TCCR0B |= (1 << CS00);

   
   TCCR0B |= (1 << CS10); // Set up timer
	
   OCR0A = 0x02;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = 0;					//R�cksetzen des Timers
   
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

#pragma mark TIMER0_OVF

ISR (TIMER0_OVF_vect)
{
   
   mscounter++;
   
   if (mscounter > 2*CLOCK_DIV) // 0.5s
   {
     // displaycounter++;
      
      
      //OSZI_A_TOGG;
      programmstatus ^= (1<<MS_DIV); // Teilung /2
      mscounter=0;
      
      if (programmstatus & (1<<SETTINGWAIT))
      {
         startcounter++;
         
      }
      
      if (programmstatus & (1<<MS_DIV))
      {
         
         laufsekunde++;
         if (laufsekunde==60)
         {
            laufminute++;
            if (laufminute==60)
            {
               laufstunde++;
               laufminute=0;
            }
            laufsekunde=0;
         }
         
         {
            manuellcounter++;
         }
         
         if (programmstatus & (1<<MOTOR_ON))
         {
            motorsekunde++;
            if (motorsekunde==60)
            {
               motorminute++;
               motorsekunde=0;
            }
            if (motorminute >= 60)
            {
               motorminute = 0;
            }
            
         }
         
         if (programmstatus & (1<<STOP_ON))
         {
            lcd_gotoxy(15,0);
            lcd_putint2(stopsekunde);

            stopsekunde++;
            if (stopsekunde == 60)
            {
               stopminute++;
               stopsekunde=0;
            }
            if (stopminute >= 60)
            {
               stopminute = 0;
            }

         }
      }
   }
   
   
   
}


#pragma mark PIN_CHANGE
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328


ISR (PCINT0_vect)
{
   
   if(INTERRUPT_PIN & (1<< MASTER_EN_PIN))// LOW to HIGH pin change, Sub ON
   {
      OSZI_C_LO;
     
      masterstatus |= (1<<SUB_TASK_BIT); // Zeitfenster fuer Task offen
      adc_counter ++; // loest adc aus
      
   }
   else // HIGH to LOW pin change, Sub ON
   {
      displaystatus |= (1<<UHR_UPDATE);
      //masterstatus &= ~(1<<SUB_TASK_BIT);
   }
   
}

#pragma mark POWER_OFF
ISR (INT3_vect) // Interrupt bei powerOff
{
   /*
   lcd_gotoxy(0,0);
   lcd_putc('+');
   lcd_putint(laufminute);
   eeprom_update_byte ((uint8_t*)0, (laufsekunde));
   
   eeprom_update_byte ((uint8_t*)1, laufminute);
   laufminute=0;
   laufminute = eeprom_read_byte((uint8_t*)1);
    */
   //lcd_putc(' ');
   //lcd_putint(laufminute);
   
   //eeprom_update_byte ((uint8_t*)2, laufstunde);
   
   programmstatus |= (1<<EEPROM_TASK);
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
   //OSZI_B_LO;
   cli();
   MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
   _delay_us(EE_READ_DELAY);
   spi_start();
   _delay_us(EE_READ_DELAY);
   SPI_PORT_Init();
   _delay_us(EE_READ_DELAY);
   spieeprom_init();
   _delay_us(EE_READ_DELAY);
   
   
   //lcd_gotoxy(1,0);
   //lcd_putc('r');
   //lcd_putint12(readadresse);
   //lcd_putc('*');
   
   eeprom_indata = 0xaa;
   uint8_t readdata=0xaa;
   
   // Byte  read 270 us
   EE_CS_LO;
   _delay_us(EE_READ_DELAY);
   OSZI_D_LO;
   readdata = (uint8_t)spieeprom_rdbyte(readadresse);
   OSZI_D_HI;
   _delay_us(EE_READ_DELAY);
   _delay_us(10);
   EE_CS_HI;
  
   /*
   sendbuffer[0] = 0xD5;
   
   sendbuffer[1] = readadresse & 0x00FF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   sendbuffer[3] = readdata;
   
   eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   abschnittnummer =0;
   
   // wird fuer Darstellung der Read-Ergebnisse im Interface benutzt.
   
//   usb_rawhid_send((void*)sendbuffer, 50);
   */
   sei();
   //OSZI_B_HI;
   //lcd_putc('*');
   return readdata;
}

uint8_t eepromverbosebytelesen(uint16_t readadresse) // 300 us ohne lcd_anzeige
{
   //OSZI_B_LO;
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
   OSZI_D_LO;
   readdata = (uint8_t)spieeprom_rdbyte(readadresse);
   OSZI_D_HI;
   _delay_us(EE_READ_DELAY);
   EE_CS_HI;
   
   sendbuffer[0] = 0xD5;
   
   sendbuffer[1] = readadresse & 0x00FF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   sendbuffer[3] = readdata;
   
  // eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   abschnittnummer =0;
   
   // wird fuer Darstellung der Read-Ergebnisse im Interface benutzt.
   
   usb_rawhid_send((void*)sendbuffer, 50);
   
   sei();
   //OSZI_B_HI;
   //lcd_putc('*');
   return readdata;
}


uint8_t eeprompartlesen(uint16_t readadresse) //   us ohne lcd_anzeige
{
   //OSZI_B_LO;
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
    
   sendbuffer[1] = readadresse & 0x00FF;
   sendbuffer[2] = (readadresse & 0xFF00)>>8;
   sendbuffer[3] = readdata;
   sendbuffer[4] = 0xDB;
   
   //eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_READ_BYTE_TASK);
   
   abschnittnummer =0;
   
   usb_rawhid_send((void*)sendbuffer, 50);
   
   sei();
   //OSZI_B_HI;
   return readdata;
}

uint8_t eeprombyteschreiben(uint8_t code, uint16_t writeadresse,uint8_t eeprom_writedatabyte) //   1 ms ohne lcd-anzeige
{
   //OSZI_B_LO;
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
   
   //OSZI_B_LO;
   // Notewndig fuer schreiben der Expo-Settings (???)
   _delay_ms(4);
   
   /*
   lcd_gotoxy(0,1);
   lcd_putc('e');
   lcd_puthex(byte_errcount);
   lcd_putc(' ');
   lcd_puthex(eeprom_writedatabyte);
   lcd_putc(' ');
   lcd_puthex(checkbyte);
   */
   //OSZI_B_HI;
   
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
   //eepromstatus &= ~(1<<EE_WRITE);
   usbtask &= ~(1<<EEPROM_WRITE_BYTE_TASK);
   
   //lcd_putc('+');
   usb_rawhid_send((void*)sendbuffer, 50);
   //lcd_putc('+');
   
   sei();
   // end Daten an EEPROM
   //OSZI_D_HI ;
   
   return byte_errcount;
}

uint16_t eeprompartschreiben(void) // 23 ms
{
   //OSZI_B_LO;
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
   //OSZI_B_HI;
   return result;
}

void writeRamByte(uint16_t adresse , uint8_t data)
{
   RAM_CS_LO;
   _delay_us(LOOPDELAY);
   spiram_wrbyte(adresse, data);
   RAM_CS_HI;
   _delay_us(LOOPDELAY);
}


uint8_t readRAMbyteAnAdresse(uint16_t adresse)
{
   RAM_CS_LO;
   _delay_us(LOOPDELAY);
   //     OSZI_B_LO;
   uint8_t ramtestdata = spiram_rdbyte(adresse);
   _delay_us(LOOPDELAY);
   //     OSZI_B_HI;
   RAM_CS_HI;
   _delay_us(LOOPDELAY);
   return ramtestdata;
}


// MARK: readSettings
void read_Ext_EEPROM_Settings(void)
{
   //EE_CS_LO;
   _delay_us(LOOPDELAY);
   uint16_t readstartadresse=0;
   uint8_t modelindex = curr_model; // welches model soll gelesen werden
    uint8_t pos=0;
   
    // Level lesen
   cli();
    readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
   sei();
    // startadresse fuer Settings des models
    for (pos=0;pos<8;pos++)
    {
       curr_levelarray[pos] = eeprombytelesen(readstartadresse+pos);
        
    }
    _delay_us(100);
   
    // Expo lesen
    readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
    for (pos=0;pos<8;pos++)
    {
       curr_expoarray[pos] = eeprombytelesen(readstartadresse+pos);
       
    }
    _delay_us(100);
   
   
   // Mix lesen
   cli();
    readstartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
   sei();
   /*
   lcd_gotoxy(0,0);
   //lcd_putc('+');
   //lcd_putint1(modelindex);
   //lcd_putc('+');
   lcd_putint12(readstartadresse);
   lcd_putc('*');
   lcd_puthex((readstartadresse & 0xFF00)>>8);
   lcd_puthex((readstartadresse & 0x00FF));
 */
   
    for (pos=0;pos<8;pos++)
    {
       if (pos==0)
       {
       //OSZI_D_LO;
       }
       cli();
       curr_mixarray[pos] = eeprombytelesen(readstartadresse+pos);
       //OSZI_D_HI;

    }
   
   _delay_us(RAMDELAY);
   
   //EE_CS_HI;
}

void write_Ext_EEPROM_Settings(void)
{
   //EE_CS_LO;
   _delay_us(LOOPDELAY);
   uint16_t readstartadresse=0;
   uint8_t modelindex = curr_model; // welches model soll gelesen werden
   uint8_t pos=0;
   
   // Level schreiben
   cli();
   readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
   sei();
   // startadresse fuer Settings des models
   for (pos=0;pos<8;pos++)
   {
      eeprombyteschreiben(0xF5,readstartadresse+pos,curr_levelarray[pos]);
      
   }
   _delay_us(100);
   
   // Expo lesen
   readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
   for (pos=0;pos<8;pos++)
   {
      eeprombyteschreiben(0xF5,readstartadresse+pos,curr_expoarray[pos]);

      
   }
   _delay_us(100);
   
   
   // Mix lesen
   cli();
   readstartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
   sei();
   /*
    lcd_gotoxy(0,0);
    //lcd_putc('+');
    //lcd_putint1(modelindex);
    //lcd_putc('+');
    lcd_putint12(readstartadresse);
    lcd_putc('*');
    lcd_puthex((readstartadresse & 0xFF00)>>8);
    lcd_puthex((readstartadresse & 0x00FF));
    */
   
   for (pos=0;pos<8;pos++)
   {
      if (pos==0)
      {
         //OSZI_D_LO;
      }
      cli();
      eeprombyteschreiben(0xF5,readstartadresse+pos,curr_mixarray[pos]);
      //OSZI_D_HI;
      
   }
   sei();
   _delay_us(RAMDELAY);
   
   //EE_CS_HI;
}



uint8_t Tastenwahl(uint8_t Tastaturwert)
{
   /*
    // Atmega168
    
    #define TASTE1		19
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		186
    #define TASTE9		212
    #define TASTE_L	234
    #define TASTE0		248
    #define TASTE_R	255
    
    
    // Atmega328
    #define TASTE1		17
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		166
    #define TASTE9		214
    #define TASTE_L	234
    #define TASTE0		252
    #define TASTE_R	255
    */
   //lcd_gotoxy(0,0);
   //lcd_putint(Tastaturwert);
   if (Tastaturwert < TASTE1)
      return 1;
   if (Tastaturwert < TASTE2)
      return 2;
   if (Tastaturwert < TASTE3)
      return 3;
   if (Tastaturwert < TASTE4)
      return 4;
   if (Tastaturwert < TASTE5)
      return 5;
   if (Tastaturwert < TASTE6)
      return 6;
   if (Tastaturwert < TASTE7)
      return 7;
   if (Tastaturwert < TASTE8)
      return 8;
   if (Tastaturwert < TASTE9)
      return 9;
   if (Tastaturwert < TASTE_L)
      return 10;
   if (Tastaturwert < TASTE0)
      return 0;
   if (Tastaturwert <= TASTE_R)
      return 12;
   
   return -1;
}

void resetcursorpos(void)
{
   // cursorpositionen init. Wert ist 0xFF wenn nicht verwendet
   uint8_t i=0,k=0;
   for (i=0;i<8;i++)
   {
      for (k=0;k<4;k++)
      {
         cursorpos[i][k]=0xFFFF;
         
      }
   }
   
}


void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
   /* Wait for completion of previous write */ while(EECR & (1<<EEPE))
      ;
   /* Set up address and Data Registers */ EEAR = uiAddress;
   EEDR = ucData;
   /* Write logical one to EEMPE */
   EECR |= (1<<EEMPE);
   /* Start eeprom write by setting EEPE */ EECR |= (1<<EEPE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
   
   /* Wait for completion of previous write */
   while(EECR & (1<<EEPE))
      ;
   /* Set up address register */
   EEAR = uiAddress;
   /* Start eeprom read by writing EERE */ EECR |= (1<<EERE);
   /* Return data from Data Register */ return EEDR;
   
}

void write_eeprom_status(void)
{
   eeprom_write_byte(&speichermodel, curr_model);
   eeprom_write_byte(&speichersetting, curr_setting);
   
}

void read_eeprom_status(void)
{
   
   curr_model = eeprom_read_byte(&speichermodel);
   curr_setting = eeprom_read_byte(&speichersetting);

}


void write_eeprom_zeit(void)
{
   eeprom_write_byte(&speichersekunde, laufsekunde);
   //lcd_gotoxy(0,0);
   //lcd_putc('*');
   //lcd_putint(eeprom_read_byte(&speichersekunde));
   eeprom_write_byte(&speicherminute, laufminute);
   eeprom_write_byte(&speicherstunde, laufstunde);

   
   eeprom_write_byte(&speichermotorsekunde, motorsekunde);
   eeprom_write_byte(&speichermotorminute, motorminute);

   eeprom_write_byte(&speicherstopsekunde, stopsekunde);
   eeprom_write_byte(&speicherstopminute, stopminute);

}

void read_eeprom_zeit(void)
{
   laufsekunde=0;
   laufsekunde = eeprom_read_byte(&speichersekunde);
   //lcd_gotoxy(0,1);
   //lcd_putc('*');
   //lcd_putint(laufsekunde);
   laufminute = eeprom_read_byte(&speicherminute);
   laufstunde = eeprom_read_byte(&speicherstunde);
   
   motorsekunde = eeprom_read_byte(&speichermotorsekunde);
   motorminute = eeprom_read_byte(&speichermotorminute);
   
   stopsekunde = eeprom_read_byte(&speicherstopsekunde);
   stopminute = eeprom_read_byte(&speicherstopminute);

   
   
}


void setdefaultsetting(void)
{
   uint8_t i=0,k=0;
   
   for (i=0;i<8;i++)
   {
      curr_levelarray[i] = default_levelarray[i];
      curr_expoarray[i] = default_expoarray[i];
      curr_mixarray[i] = default_mixarray[i];
      curr_funktionarray[i] = default_funktionarray[i];
      curr_devicearray[i] = default_devicearray[i];
      curr_ausgangarray[i] = default_ausgangarray[i];

   }
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
   
	uint16_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	initADC(0);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint16_t loopcount1=0;
	/*
    Bit 0: 1 wenn wdt ausgel�st wurde
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
   
   display_soft_init();
   
   
   //Display wird gel�scht
	display_clear();
	
   _delay_us(50);
  
   
   
   char_height_mul = 1;
   char_width_mul = 1;
   
   strcpy_P(titelbuffer, (PGM_P)pgm_read_word(&(TitelTable[5])));
   
   char_x=0;
   char_y = 3;
   //display_write_symbol(pfeilvollrechts);
   char_x += FONT_WIDTH;
   //display_write_str(titelbuffer,1);
   
   char_x=0;
   char_y = 4;
   
   // display_write_prop_str(char_y,char_x,0,(unsigned char*)titelbuffer);
   //display_write_str((unsigned char*)titelbuffer,1);
   
   char_x=0;
   char_y = 5;
	lcd_gotoxy(1,2);
	lcd_puts("RC_LCD\0");
   delay_ms(1000);
   lcd_cls();
   //substatus |= (1<<SETTINGS_READ);; // Settings beim Start lesen
   eepromstatus |= (1<<READ_EEPROM_START);
   read_eeprom_zeit();
   read_eeprom_status();

   //setdefaultsetting();
   

   

   
   sethomescreen();
   
   timer0();
   
   
   
// MARK:  while
	while (1)
	{
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
      if(INTERRUPT_PIN & (1<< MASTER_EN_PIN))
      {
         //OSZI_A_HI;
         if (substatus & (1<< TASTATUR_READ)) // Bit noch nicht reset
         {
            //substatus &= ~(1<< TASTATUR_READ);
            
         }
      }
      else
      {
         //OSZI_A_LO;
         //lcd_gotoxy(0,0); // Kein guter Platz, delay
         //lcd_putint12(laufsekunde);
         if (displaycounter)
         {
            //displaycounter=0;
            //OSZI_B_LO;
           // update_time();
           // update_screen();
            // OSZI_B_HI;
            /*
             if (!(substatus & (1<<UHR_OK)))
             {
             
             OSZI_A_LO;
             substatus |= 1<<UHR_REFRESH;
             OSZI_A_HI;
             }
             */
         }
         if (programmstatus & (1<<EEPROM_TASK))
         {
            programmstatus &= ~(1<<EEPROM_TASK);
            //cli();
            write_eeprom_zeit();
            write_eeprom_status();
            //sei();
         }

         if (!(substatus & (1<<TASTATUR_READ))) // Bit noch nicht gesetzt, nur einmal in Zyklus
         {
            //OSZI_A_LO;
            //if (mscounter%2)
            {
               
               
               substatus |= (1<< TASTATUR_READ);
               //OSZI_A_HI;
            }
         }
         
      }
      
		if (loopcount0==0xAFFF)
		{
         
         //batteriespannung = adc_read(0); // ca. 6V
  
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         
         if (programmstatus & (1<<SETTINGWAIT))
         {
         //lcd_gotoxy(14,0); // Kein guter Platz, delay
         //lcd_putint12(startcounter);
         }
         
         if (loopcount1%4 == 0) // nach etwas Zeit soll Master die Settings lesen
         {
 
            if (masterstatus & (1<<SUB_READ_EEPROM_BIT)) // beim Start ee lesen
            {
               masterstatus &= ~(1<<SUB_READ_EEPROM_BIT);
               // Beim Start RAM_SEND_PPM_STATUS schicken
               task_out |= (1<< RAM_SEND_PPM_TASK);
               task_outdata = 0;
               
               // Sub soll erst jetzt die Settings lesen. >> in RAM_TASK verschoben
               //substatus |= (1<<SETTINGS_READ);
               
            }
            //lcd_gotoxy(0,0);
         }
         
         
        
          if(loopcount1%16 == 0)
         {
            //write_eeprom_zeit();
            //lcd_gotoxy(0,0);
            //lcd_putint(eeprom_read_byte((uint8_t*)(EEPROM_STARTADRESSE+0)));
            
            //cli();
            //write_eeprom_zeit();
            //sei();
            //lcd_putint(t);
            
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
         } //

         
         
			if ((manuellcounter > MANUELLTIMEOUT) )
			{
				{
               programmstatus &= ~(1<<MANUELL);
               manuellcounter=0;
					//MANUELL_PORT &= ~(1<<MANUELLPIN);
               
               if (curr_screen) // nicht homescreen
               {
                  display_clear();
                  curr_screen=0;
                  curr_cursorspalte=0;
                  curr_cursorzeile=0;
                  last_cursorspalte=0;
                  last_cursorzeile=0;
                  sethomescreen();
               }
               
				}
				//
			}

// MARK:  USB send
         // neue Daten abschicken
//         if ((usbtask & (1<<EEPROM_WRITE_PAGE_TASK) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         //OSZI_C_LO;
         
         uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
         //OSZI_C_HI;
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
         
         batteriespannung = adc_read(0);
         
      }
      
      //if (displaycounter)
      if (displaystatus & (1<<UHR_UPDATE))
      {
         displaystatus &= ~(1<<UHR_UPDATE);
         displaycounter++;
         if ((displaycounter ==4)&& (curr_screen==0))
         {
             //OSZI_A_LO;
            display_akkuanzeige(batteriespannung);
             //OSZI_A_HI;
         }
         if (displaycounter >8)
         {
            //OSZI_B_LO;
            update_screen();
            //OSZI_B_HI;
            displaycounter=0;
         }
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
         
         OSZI_C_HI;
         _delay_us(1);
         uint8_t i=0;
         
         // SPI fuer device einschschalten
         spi_start();
         
         // Daten an RAM oder an EEPROM
         
         MEM_EN_PORT &= ~(1<<MEM_EN_PIN);
         
         _delay_us(1);
         SPI_PORT_Init();
        
         
         if (substatus & (1<<SETTINGS_READ))
         {
            //OSZI_D_LO;
            substatus &= ~(1<<SETTINGS_READ);
            read_Ext_EEPROM_Settings();
            lcd_clr_line(1);
            
            lcd_gotoxy(0,1);
            lcd_putc('L');
            //lcd_putc(' ');
            lcd_puthex(curr_levelarray[0]);
            lcd_puthex(curr_levelarray[1]);
            //lcd_putc(' ');
            lcd_puthex(curr_levelarray[2]);
            lcd_puthex(curr_levelarray[3]);
            
            lcd_putc(' ');
            lcd_putc('M');
            //lcd_putc(' ');
            lcd_puthex(curr_mixarray[0]);
            lcd_puthex(curr_mixarray[1]);
            //lcd_putc(' ');
            lcd_puthex(curr_mixarray[2]);
            lcd_puthex(curr_mixarray[3]);
            //OSZI_D_HI;
            
         }
 /*
         if (usbtask & (1<<EEPROM_WRITE_BYTE_TASK))// MARK:  EEPROM_WRITE_BYTE_TASK
         {
            
         }
         else if (usbtask & (1<<EEPROM_READ_BYTE_TASK))// MARK:  EEPROM_READ_BYTE_TASK
         {
            
         }

          else// MARK:  SPI_RAM
*/
         {
            substatus &= ~(1<<UHR_OK);
            
            substatus &= ~(1<< TASTATUR_READ);
            substatus &= ~(1<< TASTATUR_OK);
            
      //      OSZI_A_HI;
            
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
            
            // Testdaten lesen
            
            for (i=0;i<8;i++)
            {
               sendbuffer[EE_PARTBREITE+i] = readRAMbyteAnAdresse(teststartadresse+i);
            
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
            sendbuffer[0x3E] = batteriespannung & 0x00FF; // LO
            sendbuffer[0x3F] = (batteriespannung & 0xFF00) >>8; // HI
            
            
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
                  
                  // Sub soll  beim Start erst jetzt die Settings lesen.
                  if (eepromstatus & (1<<READ_EEPROM_START))
                  {
                     eepromstatus &= ~(1<<READ_EEPROM_START);
                     substatus |= (1<<SETTINGS_READ);
                  }
                  

               }
            
            
            // Fehler ausgeben
            if (outcounter%0x40 == 0)
            {
               //lcd_gotoxy(0,0);
                //lcd_putint1(errcount);
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
      r=0;
      r = usb_rawhid_recv((void*)buffer, 0); // 2us
      //OSZI_D_HI;
      // MARK: USB_READ
      if (r > 0)
      {
         
         //OSZI_D_LO;
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
                  lcd_gotoxy(0,0);
                  lcd_putc('M');
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
                        lcd_putc('0'+ kanal);
                        
                        // Level schreiben
                        // Der Wert ist auf (16 + 2* changeposition) an der ungeraden Stelle
                        uint8_t levelwert = buffer[datastartbyte + 2*changeposition + 1]; // wert an position im buffer // 0x20
                         //lcd_puthex(levelwert);
                        //lcd_putc('C');
                        //levelwert = 1;
 
                     //
                      errcount0 += eeprombyteschreiben(0xF9,fixstartadresse + LEVEL_OFFSET + kanal,levelwert); // adresse im EEPROM
               
                        //       lcd_putint(errcount);
                  /*
                         sendbuffer[EE_PARTBREITE + 2*kanal+1] = levelwert;
                        */
                        //lcd_putc('A'+kanal);
                        
                        // Expo schreiben
                        // Der Wert ist auf (16 + 2* changeposition) an der geraden Stelle
                        uint8_t expowert = buffer[datastartbyte + 2*changeposition];
                        
                        //Test
                       // expowert= 0;
                        //
                        
                     errcount1 += eeprombyteschreiben(0xF9,fixstartadresse + EXPO_OFFSET + kanal,expowert); // 0x30
                        
                        //sendbuffer[EE_PARTBREITE + 2*kanal] = expowert;
                        
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
                        //sendbuffer[0x10+2*kanal] = levelwert;
                        //sendbuffer[0x10+2*kanal+1] = expowert;
                        
                        sendbuffer[EE_PARTBREITE+kanal] = levelwert; // EE_PARTBREITE 32
                        sendbuffer[EE_PARTBREITE+0x08+kanal] = expowert;
                        

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
                  
                  uint8_t pos=0, verbose=buffer[4];
                  
                  // Level lesen
                  uint16_t readstartadresse = TASK_OFFSET  + LEVEL_OFFSET + modelindex*SETTINGBREITE;
                  // startadresse fuer Settings des models
                  for (pos=0;pos<8;pos++)
                  {
                     if (buffer[4])
                     {
                        uint8_t leveldata = eepromverbosebytelesen(readstartadresse+pos); // ab 0x20 32
                        //sendbuffer[EE_PARTBREITE + pos] = eepromverbosebytelesen(readstartadresse+pos); // ab 0x20 32
                        sendbuffer[EE_PARTBREITE + pos] = leveldata;
                        curr_levelarray[pos] = leveldata;
                     }
                     else
                     {
                     sendbuffer[EE_PARTBREITE + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x20 32
                     }
                  }
                  
                  // Expo lesen
                  readstartadresse = TASK_OFFSET  + EXPO_OFFSET + modelindex*SETTINGBREITE;
                  //Im Sendbuffer ab pos EE_PARTBREITE + 0x08 (8)
                  for (pos=0;pos<8;pos++)
                  {
                     if (buffer[5])
                     {
                        sendbuffer[EE_PARTBREITE + 0x08 + pos] = eepromverbosebytelesen(readstartadresse+pos); // ab 0x28 40
                     }
                     else
                     {
                        sendbuffer[EE_PARTBREITE + 0x08 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x28 40
                        
                     }
                  }
                  
                  
                  // Mix lesen
                  readstartadresse = TASK_OFFSET  + MIX_OFFSET + modelindex*SETTINGBREITE;
                  
                  //Im Sendbuffer ab pos EE_PARTBREITE + 0x10 (16)
                  for (pos=0;pos<8;pos++)
                  {
                     if (buffer[6])
                     {
                        uint8_t mixdata = eepromverbosebytelesen(readstartadresse+pos); // ab 0x30 48
                        //sendbuffer[EE_PARTBREITE + 0x10 + pos] = eepromverbosebytelesen(readstartadresse+pos); // ab 0x30 48
                        curr_mixarray[pos] = mixdata;
                        sendbuffer[EE_PARTBREITE + 0x10 + pos] = mixdata;
                     }
                     else
                     {
                        sendbuffer[EE_PARTBREITE + 0x10 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x30 48
                     }
                  }
                  
                  
                  sendbuffer[1] = readstartadresse & 0x00FF;
                  sendbuffer[2] = (readstartadresse & 0xFF00)>>8;
                  sendbuffer[3] = modelindex;
                  sendbuffer[4] = 0xFF;
                  sendbuffer[5] = buffer[6];
                  
                  // code
                  sendbuffer[0] = 0xF5;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  lcd_clr_line(0);
                  lcd_gotoxy(0,0);
                  lcd_putc('L');
                  //lcd_putc(' ');
                  lcd_puthex(curr_levelarray[0]);
                  lcd_puthex(curr_levelarray[1]);
                  //lcd_putc(' ');
                  lcd_puthex(curr_levelarray[2]);
                  lcd_puthex(curr_levelarray[3]);
                  
                  lcd_putc(' ');
                  lcd_putc('M');
                  //lcd_putc(' ');
                  lcd_puthex(curr_mixarray[0]);
                  lcd_puthex(curr_mixarray[1]);
                  //lcd_putc(' ');
                  lcd_puthex(curr_mixarray[2]);
                  lcd_puthex(curr_mixarray[3]);
                  //OSZI_D_HI;

               
               
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
                  lcd_putc('*');
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
                  
               case 0xFD: // read Sendersettings
               {
                  
                  uint8_t modelindex =0;
                  modelindex = buffer[3]; // welches model soll gelesen werden
                  uint8_t pos=0;
                  
                  // funktion lesen
                  uint16_t readstartadresse = TASK_OFFSET  + FUNKTION_OFFSET + modelindex*SETTINGBREITE;
                  // startadresse fuer Settings des models
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x20 32
                  }
                  
                  // device lesen
                  readstartadresse = TASK_OFFSET  + DEVICE_OFFSET + modelindex*SETTINGBREITE;
                  //Im Sendbuffer ab pos 0x08 (8)
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x08 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x28 40
                  }
                  
                  // Ausgang lesen
                  readstartadresse = TASK_OFFSET  + AUSGANG_OFFSET + modelindex*SETTINGBREITE;
                  
                  //Im Sendbuffer ab pos 0x10 (16)
                  for (pos=0;pos<8;pos++)
                  {
                     sendbuffer[EE_PARTBREITE + 0x10 + pos] = eeprombytelesen(readstartadresse+pos); // ab 0x30 48
                     
                  }


                  
                  sendbuffer[1] = readstartadresse & 0x00FF;
                  sendbuffer[2] = (readstartadresse & 0xFF00)>>8;
                  sendbuffer[3] = modelindex;
                  
                  sendbuffer[0] = 0xFD;
                  
                  
                  usb_rawhid_send((void*)sendbuffer, 50);

               }
                  
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
         
       //OSZI_D_HI;
         
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
		
		if (!(TASTENPIN & (1<<E_TASTE0))) // Taste 0
		{
			//lcd_gotoxy(8,1);
			//lcd_puts("T0 Down\0");
			
			if (!(TastenStatus & (1<<E_TASTE0))) //Taste 0 war noch nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<E_TASTE0);
				
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
               if (TastenStatus & (1<<E_TASTE0))
               {
                  //sendbuffer[0]=loopcount1;
                  //sendbuffer[1]=0xAB;
                  //usbstatus |= (1<<USB_RECV);
                  //lcd_gotoxy(2,1);
                  //lcd_putc('1');
                  
                  //usb_rawhid_send((void*)sendbuffer, 50);
               }
					TastenStatus &= ~(1<<E_TASTE0);
               //lcd_gotoxy(3,1);
               //lcd_puts("ON \0");
               //delay_ms(400);
               //lcd_gotoxy(3,1);
               // lcd_puts("  \0");
               //lcd_putint(TastenStatus);
               
               
				}
			}//else
			
		}	// Taste 0
		
		if (!(TASTENPIN & (1<<E_TASTE1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("T1 Down\0");
			
			if (! (TastenStatus & (1<<E_TASTE1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<E_TASTE1);
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
					TastenStatus &= ~(1<<E_TASTE1);
				}
			}//	else
			
		} // Taste 1
		// MARK:  Tastatur
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
      if ((substatus & (1<< TASTATUR_READ)) && (mscounter%2))
      {
         
         Tastenwert=0;
         //OSZI_B_LO;
         //substatus &= ~(1<< TASTATUR_READ);
         // OSZI_B_TOGG;
         Tastenwert=adc_read(TASTATURPIN)>>2;
        // lcd_gotoxy(4,1);
         //lcd_putint12(Tastenwert);
         //
         if (Tastenwert>5)
         {
            /*
             0:											1	2	3
             1:											4	5	6
             2:											7	8	9
             3:											x	0	y
             4: Schalterpos -
             5: enter
             6: Schalterpos +
             7:
             8:
             9:
             
             12: Manuell aus
             */
            
            TastaturCount++;
            if ((TastaturCount>=100) )
            {
               substatus |= (1<<TASTATUR_OK);
                              /*
               lcd_gotoxy(10,1);
               lcd_puts("T:\0");
               lcd_putint12(Tastenwert);
               lcd_putc(' ');
               lcd_putc(' ');
               
               
               lcd_gotoxy(18,1);
               */
               Taste=Tastenwahl(Tastenwert);
               //lcd_putint2(Taste);
               //lcd_putc(' ');
               lcd_gotoxy(0,1);
               //lcd_putint(TastaturCount);
              // lcd_putc(' ');
               //lcd_putint2(Taste);
               //lcd_putc('*');
               TastaturCount=0;
               Tastenwert=0x00;
               //uint8_t i=0;
               //uint8_t pos=0;
               
               programmstatus |= (1<<UPDATESCREEN);
               
               switch (Taste)
               {
#pragma mark Taste 0
                  case 0:// Schalter auf Null-Position
                  {
                     
                     if (programmstatus & (1<<MANUELL))
                     {
                        manuellcounter=0;
                     }
                     
                  }break;
                     
                  case 1:
                  {
#pragma mark Taste 1
                     if (manuellcounter)
                     {
                        programmstatus ^= (1<<MOTOR_ON);
                        manuellcounter=0;
                        
                     }

                  }break;
                     
                  case 2://
                  {
#pragma mark Taste 2
                     switch (curr_screen)
                     {
                        case HOMESCREEN: // home
                        {
                           if (manuellcounter > 2)
                           {
                           substatus |= (1<<SETTINGS_READ);; // Settings beim Start lesen
                              manuellcounter=0;
                           }
 
                        }break;
                           
                        case SETTINGSCREEN: // Settings
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              //lcd_gotoxy(0,1);
                              if (curr_cursorzeile ) // curr_cursorzeile ist >0,
                              {
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile--;
                                 //lcd_putc('+');
                              }
                              else
                              {
                                 
                                 //lcd_putc('-');
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter)
                           {
                              /*
                               lcd_gotoxy(0,1);
                               lcd_puthex((blink_cursorpos & 0xFF00)>>8);
                               lcd_putc('*');
                               lcd_puthex((blink_cursorpos & 0x00FF));
                               */
                              //switch((blink_cursorpos & 0xFF00)>>8) // Blink-Zeile
                              switch(curr_cursorzeile) // Blink-Zeile
                              {
                                 case 0: // modell
                                 {
                                    //switch (blink_cursorpos & 0x00FF)
                                    switch (curr_cursorspalte)
                                    {
                                       case 0:
                                       {
                                          //lcd_putc('0');
                                          if (curr_model )
                                          {
                                             curr_model--;
                                          }
                                          
                                       }break;
                                          
                                       case 1:
                                       {
                                          //lcd_putc('1');
                                          if (curr_setting )
                                          {
                                             curr_setting--;
                                          }
                                          
                                       }break;
                                    } // switch Spalte
                                    
                                 }break;
                                    
                                 case  1:
                                 {
                                    
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter=0;
                           }
                           
                           
                        }break;
                           
                        case KANALSCREEN: // Kanalsettings
                        {
                           /*
                            lcd_gotoxy(5,1);
                            lcd_puthex(curr_cursorzeile);
                            lcd_putc('*');
                            lcd_puthex((blink_cursorpos & 0xFF00)>>8); // Zeile
                            lcd_putc('*');
                            //lcd_putc('*');
                            */
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                              
                           {
                              if (curr_cursorzeile )//
                              {
                                 if (curr_cursorzeile<8)
                                 {
                                    char_height_mul=1;
                                 }
                                 else
                                 {
                                    char_height_mul=1;
                                 }
                                 
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile--;
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc('+');
                              }
                              else
                              {
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc('-');
                              }
                              //lcd_putint2(curr_cursorzeile);
                              
                              //lcd_putc(' ');
                              
                              
                              manuellcounter=0;
                           }
                           else if (manuellcounter) // blinken ist on
                           {
                              
                              //switch((blink_cursorpos & 0xFF00)>>8) // zeile
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanal
                                 {
                                    //uint8_t tempspalte = (blink_cursorpos & 0x00FF);
                                    //lcd_puthex(curr_cursorspalte);
                                    //blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          if (curr_kanal )
                                          {
                                             curr_kanal--;
                                          }
                                       }break;
                                          
                                       case 1: // Richtung
                                       {
                                          //if (curr_settingarray[curr_kanal][1] & 0x80)
                                          if (curr_expoarray[curr_kanal] & 0x80)
                                          {
                                             curr_expoarray[curr_kanal] &= ~0x80;
                                          }
                                          else
                                          {
                                             curr_expoarray[curr_kanal] |= 0x80;
                                          }
                                       }break;
                                          
                                       case 2: // Funktion
                                       {
                                          //lcd_gotoxy(5,1);
                                          //lcd_putc('*');
                                          //Bezeichnung von: FunktionTable[curr_funktionarray[curr_kanal]]
                                          //uint8_t tempfunktion = curr_funktionarray[curr_kanal]& 0x07; // Bit 0-3
                                          //lcd_puthex(tempfunktion);
                                          //lcd_putc('*');
                                          
                                          //tempfunktion--; // decrementieren
                                          //tempfunktion &= 0x07; // Begrenzen auf 0-7
                                          //lcd_puthex(tempfunktion);
                                          //curr_funktionarray[curr_kanal] |= tempfunktion; // cycle in FunktionTable
                                          
                                          uint8_t tempfunktion = curr_funktionarray[curr_kanal]&0x07; //bit 0-2
                                          tempfunktion--;
                                          tempfunktion &= 0x07;
                                          
                                          curr_funktionarray[curr_kanal] = (curr_funktionarray[curr_kanal]&0xF0)|tempfunktion; // cycle in FunktionTable
                                          
                                       }break;
                                          
                                          
                                          
                                    }// switch tempspalte
                                    
                                 }break;
                                    
                                 case  1: // level
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // Levelwert A
                                       {
                                          if ((curr_levelarray[curr_kanal] & 0x70)>>4)
                                          {
                                             curr_levelarray[curr_kanal] -= 0x10;
                                          }
                                          
                                       }break;
                                       case 1: // Levelwert B
                                       {
                                          if ((curr_levelarray[curr_kanal] & 0x07))
                                          {
                                             curr_levelarray[curr_kanal] -= 0x01;
                                          }
                                          
                                       }break;
                                          
                                       case 2: //
                                       {
                                          curr_cursorspalte = 1; // fehler, back
                                          
                                       }break;
                                          
                                    }// switch tempspalte
                                    
                                 }break;
                                    
                                 case  2: // Expo
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // expowert A
                                       {
                                          if ((curr_expoarray[curr_kanal] & 0x70)>>4)
                                          {
                                             curr_expoarray[curr_kanal] -= 0x10;
                                          }
                                       }break;
                                          
                                       case 1: // Expowert B
                                       {
                                          if ((curr_expoarray[curr_kanal] & 0x07))
                                          {
                                             curr_expoarray[curr_kanal] -= 0x01;
                                          }
                                       }break;
                                          
                                       case 2: //
                                       {
                                          curr_cursorspalte = 1; // fehler, back
                                       }break;
                                          
                                    }// switch tempspalte
                                 }break;
                                    
                                 case  4:
                                 {
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter=0;
                           } // if manuellcounter
                        }break; // canalscreen
                           
                        case MIXSCREEN:
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (curr_cursorzeile )//
                              {
                                 if (curr_cursorzeile<8)
                                 {
                                    char_height_mul=1;
                                 }
                                 else
                                 {
                                    char_height_mul=1;
                                 }
                                 
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile--;
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter) // blinken ist on
                           {
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Mix weiterschalten
                                 {
                                    
                                    if (curr_mixarray[2*curr_cursorzeile+1])
                                    {
                                       curr_mixarray[2*curr_cursorzeile+1] -= 0x01;// Mix ist auf ungerader zeile
                                    }
                                    
                                 }break;
                                    
                                 case 1: // Kanal A zurueckschalten
                                 {
                                    uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0x70)>>4;// kanal a ist auf gerader zeile in bit 4-6
                                    
                                    if (tempdata) //
                                    {
                                       curr_mixarray[2*curr_cursorzeile] -= 0x10;
                                    }
                                    
                                    
                                 }break;
                                    
                                 case 2: // Kanal B zurueckschalten
                                 {
                                    uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0x07);// kanal b ist auf gerader zeile in bit 0-2
                                    
                                    if (tempdata)
                                    {
                                       curr_mixarray[2*curr_cursorzeile] -= 0x01;
                                    }
                                    
                                 }break;
                                    
                              }// switch curr_cursorspalte
                              
                              manuellcounter = 0;
                           }
                        }break; // mixscreen
                           
                           
                        case ZUTEILUNGSCREEN:
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (curr_cursorzeile )//
                              {
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile--;
                                 //lcd_puthex(curr_cursorzeile);
                                 //lcd_putc('+');
                              }
                              else
                              {
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter) // blinken ist on
                           {
                              //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                              /*
                               const char funktion0[] PROGMEM = "Seite  \0";
                               const char funktion1[] PROGMEM = "Hoehe  \0";
                               const char funktion2[] PROGMEM = "Quer   \0";
                               const char funktion3[] PROGMEM = "Motor  \0";
                               const char funktion4[] PROGMEM = "Quer L\0";
                               const char funktion5[] PROGMEM = "Quer R\0";
                               const char funktion6[] PROGMEM = "Lande  \0";
                               const char funktion7[] PROGMEM = "Aux    \0";
                               
                               */
                              /*
                               lcd_gotoxy(0,0);
                               lcd_puthex(curr_cursorzeile);
                               lcd_putc(' ');
                               lcd_puthex(curr_cursorspalte);
                               lcd_putc(' ');
                               */
                              switch (curr_cursorzeile)
                              {
                                 case 0: // pitch vertikal
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // L_V index 1
                                       {
                                          // Kanalnummer decrement
                                          if (((curr_devicearray[1]& 0x07)))
                                          {
                                             curr_devicearray[1]-= 0x01;
                                          }
                                       }break;
                                          
                                       case 1: // R_V index 3
                                       {
                                          if (((curr_devicearray[3]& 0x07)))
                                          {
                                             curr_devicearray[3]-= 0x01;
                                          }
                                       }break;
                                    }// switch curr_cursorspalte
                                 }break; // pitch v
                                    
                                 case 1: // pitch horizontal
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // L_H index 0
                                       {
                                          if (((curr_devicearray[0]& 0x07)))
                                          {
                                             // Kanalnummer fuer Device decrement
                                             curr_devicearray[0]-= 0x01;
                                          }
                                       }break;
                                          
                                       case 1: // R_H index 2
                                       {
                                          if (((curr_devicearray[2]& 0x07)))
                                          {
                                             curr_devicearray[2]-= 0x01;
                                          }
                                       }break;
                                    }// switch curr_cursorspalte
                                 }break; // case spalte
                                    
                                 case 2: // schieber
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // S_L index 4
                                       {
                                          if (((curr_devicearray[4]& 0x07)))
                                          {
                                             // Kanalnummer fuer Device increment
                                             curr_devicearray[4]-= 0x01;
                                          }
                                       }break;
                                          
                                       case 1: // S_R index 5
                                       {
                                          if (((curr_devicearray[5]& 0x07)))
                                          {
                                             curr_devicearray[5]-= 0x01;
                                          }
                                       }break;
                                    }// switch curr_cursorspalte
                                 }break; // case spalte
                                    
                              }//switch curr_cursorzeile
                              manuellcounter = 0;
                              
                           } // else if manuellcounter
                           
                        }break; // zuteilungscreen
                           
                        case AUSGANGSCREEN:
                        {
#pragma mark 2 AUSGANGSCREEN
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (curr_cursorzeile)// noch nicht zuoberst
                              {
                                 char_height_mul=1;
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 if ((curr_impuls < 4) || (curr_impuls > 4))
                                 {
                                    curr_cursorzeile--;
                                 }
                                 else // zurueckscrollen
                                 {
                                    curr_cursorzeile = 3;
                                 }
                                 
                                 curr_impuls--;
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter) // blinken ist on
                           {
                              //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                              /*
                               const char funktion0[] PROGMEM = "Seite  \0";
                               const char funktion1[] PROGMEM = "Hoehe  \0";
                               const char funktion2[] PROGMEM = "Quer   \0";
                               const char funktion3[] PROGMEM = "Motor  \0";
                               const char funktion4[] PROGMEM = "Quer L\0";
                               const char funktion5[] PROGMEM = "Quer R\0";
                               const char funktion6[] PROGMEM = "Lande  \0";
                               const char funktion7[] PROGMEM = "Aux    \0";
                               */
                              /*
                               lcd_gotoxy(0,0);
                               lcd_puthex(curr_cursorzeile);
                               lcd_putc(' ');
                               lcd_puthex(curr_cursorspalte);
                               lcd_putc(' ');
                               */
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Kanal
                                 {
                                    // Kanalnummer im Devicearray increment
                                    if (((curr_ausgangarray[curr_cursorzeile]& 0x07)))
                                    {
                                       curr_ausgangarray[curr_cursorzeile]-= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // Zeile  nach oben verschieben
                                 {
                                    if (((curr_ausgangarray[curr_cursorzeile]& 0x07))<8)
                                    {
                                       uint8_t tempzeilenwert =curr_ausgangarray[curr_cursorzeile];
                                       if (curr_impuls) // nicht erste Zeile, auf erster Seite
                                       {
                                          if ((curr_cursorzeile < 4) && (curr_impuls < 4)) // Noch vor scrollen, auf erster Seite
                                          {
                                             tempzeilenwert =curr_ausgangarray[curr_impuls];
                                             
                                             curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls-1]; // Wert von naechster zeile
                                             curr_ausgangarray[curr_impuls -1] = tempzeilenwert;
                                             // cursorzeile verschieben
                                             display_cursorweg();
                                             
                                             curr_cursorzeile--;
                                             // blink-cursorzeile verschieben
                                             blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                             
                                          }
                                          else  if ((curr_cursorzeile == 1) && (curr_impuls == 4))// zweite Zeile auf Seite 2, scrollen.
                                          {
                                             tempzeilenwert =curr_ausgangarray[curr_impuls];
                                             
                                             curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls-1]; // Wert von naechster zeile, noch auf dieser Seite
                                             curr_ausgangarray[curr_impuls -1] = tempzeilenwert;
                                             display_cursorweg();
                                             curr_cursorzeile = 3; // Scroll
                                             // blink-cursorzeile verschieben
                                             blink_cursorpos = cursorpos[3][curr_cursorspalte];
                                          }
                                          else  if ((curr_cursorzeile) && (curr_impuls >4))// zweite Zeile oder mehr auf zweiter Seite
                                          {
                                             tempzeilenwert =curr_ausgangarray[curr_impuls];
                                             curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls-1]; // Wert von naechster zeile
                                             curr_ausgangarray[curr_impuls -1] = tempzeilenwert;
                                             // cursorzeile verschieben
                                             display_cursorweg();
                                             
                                             curr_cursorzeile--;
                                             // blink-cursorzeile verschieben
                                             blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                             
                                          }
                                          curr_impuls--;
                                       }
                                       else // letzte Zeile, mit erster zeile vertauschen
                                       {
                                          /*
                                           tempzeilenwert =curr_ausgangarray[curr_impuls];
                                           curr_ausgangarray[curr_impuls] =curr_ausgangarray[0]; // Wert von erster zeile
                                           curr_ausgangarray[0] = tempzeilenwert;
                                           display_cursorweg();
                                           curr_cursorzeile=0;
                                           curr_impuls =0;
                                           blink_cursorpos = cursorpos[0][curr_cursorspalte];
                                           */
                                       }
                                    }
                                    
                                 }break;
                                    
                                    
                              }// switch curr_cursorspalte
                              manuellcounter = 0;
                              
                           } // else if manuellcounter
                           
                        }break; // case ausgang
                           
                     }// switch
                     
                  }break;
                     
                  case 3: //
                  {
#pragma mark Taste 3
                     if (manuellcounter)
                     {
                        programmstatus ^= (1<<STOP_ON);
                         manuellcounter=0;
                     }
                  }break;
                     
                  case 4:// nach links
                  {
#pragma mark Taste 4
                     switch (curr_screen)
                     {
                        case HOMESCREEN: // home
                        {
                           
                         }break;
                        case SAVESCREEN: // save
                        {
                           if (curr_cursorspalte)
                           {
                              display_cursorweg();
                              
                              char_height_mul=1;
                              last_cursorspalte =curr_cursorspalte;
                              curr_cursorspalte--;
                              //lcd_putc('+');
                           }
                           
                        }break;

                           
                        case SETTINGSCREEN: // Settings
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Modellname
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Modellname
                                       {
                                          
                                       }   break;
                                          
                                       case 1: // Set text
                                       {
                                          display_cursorweg();
                                          char_height_mul=1;
                                          last_cursorspalte =curr_cursorspalte;
                                          curr_cursorspalte--;
                                       }break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  2: // Kanal
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // levelwert A, nicht weiter nach links
                                       {
                                          
                                       }   break;
                                          
                                       case 1: // Levelwert B
                                       {
                                          display_cursorweg();
                                          char_height_mul=1;
                                          last_cursorspalte =curr_cursorspalte;
                                          curr_cursorspalte--;
                                       }break;
                                    }
                                 }break;
                                    
                                 case  3: // Mix
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // expowert A, nicht weiter nach links
                                       {
                                          
                                       }   break;
                                          
                                       case 1: // expowert B
                                       {
                                          display_cursorweg();
                                          char_height_mul=1;
                                          last_cursorspalte =curr_cursorspalte;
                                          
                                          curr_cursorspalte--;
                                          
                                       }break;
                                    }
                                    
                                 }break;
                                    //
                                    
                              }// switch
                              
                           }
                           else
                           {
                              switch(curr_cursorzeile) // zeile
                              
                              {
                                 case 0: // modell
                                 {
                                    
                                 }break;
                                 case  1:
                                 {
                                 }break;
                                    
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                           }
                           
                           
                        }break;
                           
                        case KANALSCREEN: // Kanal
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Kanaltext
                                       {
                                       }   break;
                                          
                                       default: // Richtung
                                       {
                                          display_cursorweg();
                                          char_height_mul=1;
                                          last_cursorspalte =curr_cursorspalte;
                                          
                                          curr_cursorspalte--;
                                          
                                          
                                       }break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1: // Level
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // levelwert A, nicht weiter nach links
                                       {
                                          
                                       }   break;
                                          
                                       default: // Levelwert B
                                       {
                                          display_cursorweg();
                                          char_height_mul=1;
                                          last_cursorspalte =curr_cursorspalte;
                                          
                                          curr_cursorspalte--;
                                          
                                       }break;
                                    }
                                 }break;
                                    
                                 case  2: // Expo
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // expowert A, nicht weiter nach links
                                       {
                                          
                                       }   break;
                                          
                                       case 1: // expowert B
                                       {
                                          display_cursorweg();
                                          char_height_mul=1;
                                          last_cursorspalte =curr_cursorspalte;
                                          
                                          curr_cursorspalte--;
                                          
                                       }break;
                                    }
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter=0;
                           }
                           else if (manuellcounter)
                           {
                              //switch((blink_cursorpos & 0xFF00)>>8) // zeile
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0:
                                 {
                                    
                                    
                                 }break;
                                 case  1:
                                 {
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter=0;
                           }
                        }break; // Kanalscreen
                           
                        case MIXSCREEN: // Mixing
                        {
                           lcd_gotoxy(0,0);
                           
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (curr_cursorspalte)
                              {
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte--;
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc(' ');
                                 lcd_puthex(curr_cursorspalte);
                                 lcd_putc(' ');
                                 lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                              }
                              manuellcounter=0;
                              
                           }
                           else
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          
                                       }   break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1:
                                 {
                                    //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                           }
                           
                           
                        }break;
                           
                        case ZUTEILUNGSCREEN: // Zuteilung
                        {
                           lcd_gotoxy(0,0);
                           
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (curr_cursorspalte)
                              {
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte--;
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc(' ');
                                 lcd_puthex(curr_cursorspalte);
                                 lcd_putc(' ');
                                 lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                              }
                              manuellcounter=0;
                              
                           }
                           else if (manuellcounter)
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          
                                       }   break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1:
                                 {
                                    //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter=0;
                           }
                           
                           
                        }break; // case Zuteilungscreen
                           
                        case AUSGANGSCREEN: // Ausgang
                        {
                           lcd_gotoxy(0,0);
                           
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (curr_cursorspalte)
                              {
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte--;
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc(' ');
                                 lcd_puthex(curr_cursorspalte);
                                 lcd_putc(' ');
                                 lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                              }
                              manuellcounter=0;
                              
                           }
                           else if (manuellcounter)
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          
                                       }   break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1:
                                 {
                                    //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter=0;
                           }
                           
                           
                        }break; // case Ausgangscreen
                           
                           
                           
                     }// switch curr_screen
                     
                     
                  }break;
                     
                  case 5://
                  {
#pragma mark Taste 5
                     switch (curr_screen)
                     {
                           /*
                        case HOMESCREEN:
                        {
                           if (startcounter == 0) // Settings sind nicht aktiv
                           {
                              programmstatus |= (1<< SETTINGWAIT);
                              settingstartcounter++;
                           }
                           else if (startcounter > 5) // Irrtum, kein Umschalten
                           {
                              programmstatus &= ~(1<< SETTINGWAIT);
                              settingstartcounter=0;
                              startcounter=0;
                              manuellcounter = 0;
                           }
                           else
                           {
                              if (programmstatus & (1<< SETTINGWAIT)) // Umschaltvorgang noch aktiv
                              {
                                 settingstartcounter++; // counter fuer klicks
                                 if (settingstartcounter > 2)
                                 {
                                    programmstatus &= ~(1<< SETTINGWAIT);
                                    settingstartcounter=0;
                                    startcounter=0;
                                    
                                    // Umschalten
                                    display_clear();
                                    setsettingscreen();
                                    curr_screen = SETTINGSCREEN;
                                    curr_cursorspalte=0;
                                    curr_cursorzeile=0;
                                    last_cursorspalte=0;
                                    last_cursorzeile=0;
                                    blink_cursorpos=0xFFFF;
                                    manuellcounter = 0;
                                    
                                 } // if settingcounter <
                              }
                           }
                        }break;
                           */
                        case HOMESCREEN:
                        {
                           
               				lcd_putint2(startcounter);
                           lcd_putc('*');
                           if ((startcounter == 0)&& (manuellcounter > 1)) // Settings sind nicht aktiv
                           {
                              lcd_putc('A');
                              programmstatus |= (1<< SETTINGWAIT);
                              settingstartcounter++;
                              manuellcounter = 0;
                           }
                           else if (startcounter > 5) // Irrtum, kein Umschalten
                           {
                              lcd_putc('X');
                              programmstatus &= ~(1<< SETTINGWAIT);
                              settingstartcounter=0;
                              startcounter=0;
                              manuellcounter = 0;
                           }
                           else
                           {
                              if ((programmstatus & (1<< SETTINGWAIT))&& (manuellcounter > 1)) // Umschaltvorgang noch aktiv
                              {
                                 lcd_putc('B');
                                 
                                 settingstartcounter++; // counter fuer klicks
                                 if (settingstartcounter > 2)
                                 {
                                    
                                    //lcd_putc('C');
                                    programmstatus &= ~(1<< SETTINGWAIT);
                                    programmstatus |=(1<<UPDATESCREEN);
                                    settingstartcounter=0;
                                    startcounter=0;
                                    
                                    // Umschalten
                                    display_clear();
                                    //lcd_putc('D');
                                    setsettingscreen();
                                    //lcd_putc('E');
                                    curr_screen = SETTINGSCREEN;
                                    curr_cursorspalte=0;
                                    curr_cursorzeile=0;
                                    last_cursorspalte=0;
                                    last_cursorzeile=0;
                                    blink_cursorpos=0xFFFF;
                                    manuellcounter = 0;
                                    programmstatus |= (1<<MANUELL);
                                    
                                 } // if settingcounter <
                              }
                           }
                        }break;
                           
                        case SAVESCREEN:
                        {
                           switch (curr_cursorspalte)
                           {
                              case 0: // sichern
                              {
                                 
                              }break;
                                 
                              case 1: // abbrechen
                              {
                                 read_Ext_EEPROM_Settings();// zuruecksetzen
                              }break;

                           }// switch curr_cursorspalte
                           
                           display_clear();
                           curr_screen=0;
                           curr_cursorspalte=0;
                           curr_cursorzeile=0;
                           last_cursorspalte=0;
                           last_cursorzeile=0;
                           blink_cursorpos = 0xFFFF;
                           
                           sethomescreen();
                           
                           
                        }break;

                           
                        case SETTINGSCREEN: // setting
                        {
                           #pragma mark  5 SETTINGSCREEN
                           if (manuellcounter)
                           {
                              switch (curr_cursorzeile)
                              {
                                 case 0: // Modell
                                 {
                                    
                                    // lcd_gotoxy(0,0);
                                    //lcd_puthex(curr_cursorzeile);
                                    //lcd_putc('*');
                                    //lcd_puthex(curr_cursorspalte);
                                    if (manuellcounter)
                                    {
                                       blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                                       manuellcounter=0;
                                    } // if manuellcounter
                                    /*
                                     switch (curr_cursorspalte)
                                     {
                                     case 0:
                                     {
                                     // lcd_putc('>');
                                     blink_cursorpos =  cursorpos[0][0]; // Modellname
                                     
                                     }break;
                                     case 1: // Set
                                     {
                                     lcd_putc('1');
                                     blink_cursorpos =  cursorpos[0][1]; // setting nummer
                                     
                                     }break;
                                     case 2:
                                     {
                                     blink_cursorpos =  cursorpos[0][2]; // setting nummer
                                     }break;
                                     
                                     
                                     } // switch curr_cursorspalte
                                     */
                                 }break;
                                    
                                    
                                 case 1: // Kanal
                                 {
                                    // Zu Kanal-Screen
                                    //blink_cursorpos =  cursorpos[2][0]; // canalcursor
                                    if (manuellcounter)
                                    {
                                       display_clear();
                                       
                                       curr_screen = KANALSCREEN;
                                       blink_cursorpos=0xFFFF;
                                       curr_cursorspalte=0;
                                       curr_cursorzeile=0;
                                       last_cursorspalte=0;
                                       last_cursorzeile=0;
                                       setcanalscreen();
                                       manuellcounter=0;
                                    }
                                    
                                    
                                 }break;
                                 case 2: // Mix
                                 {
                                    //zu Mix-Screen
                                    if (manuellcounter)
                                    {
                                       display_clear();
                                       
                                       curr_screen = MIXSCREEN;
                                       blink_cursorpos=0xFFFF;
                                       curr_cursorspalte=0;
                                       curr_cursorzeile=0;
                                       last_cursorspalte=0;
                                       last_cursorzeile=0;
                                       setmixscreen();
                                       manuellcounter=0;
                                    }
                                    
                                 }break;
                                    
                                 case 3: // Zuteilung
                                 {
                                    //zu Zuteilung-Screen
                                    if (manuellcounter)
                                    {
                                       display_clear();
                                       
                                       curr_screen = ZUTEILUNGSCREEN;
                                       blink_cursorpos=0xFFFF;
                                       curr_cursorspalte=0;
                                       curr_cursorzeile=0;
                                       last_cursorspalte=0;
                                       last_cursorzeile=0;
                                       setzuteilungscreen();
                                       manuellcounter=0;
                                    }
                                    
                                 }break;
                                    
                                 case 4: // Ausgang
                                 {
                                    //zu Zuteilung-Screen
                                    if (manuellcounter)
                                    {
                                       display_clear();
                                       
                                       curr_screen = AUSGANGSCREEN;
                                       blink_cursorpos=0xFFFF;
                                       curr_cursorspalte=0;
                                       curr_cursorzeile=0;
                                       last_cursorspalte=0;
                                       last_cursorzeile=0;
                                       setausgangscreen();
                                       manuellcounter=0;
                                    }
                                    
                                 }break;
                                    
                                    
                              }// switch curr_cursorzeile
                           } // if manuellcounter
                           
                        }break;
                           
                        case KANALSCREEN: // Kanal
                        {
                           #pragma mark  5 KANALSCREEN
                           if (manuellcounter)
                           {
                              blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                              manuellcounter=0;
                           } // if manuellcounter
                           
                           /*
                            if (manuellcounter)
                            {
                            switch (curr_cursorzeile)
                            {
                            case 0: // Kanal
                            {
                            
                            switch (curr_cursorspalte)
                            {
                            case 0:
                            {
                            blink_cursorpos =  cursorpos[0][0]; // kanalcursor
                            }break;
                            case 1: // Richtung
                            {
                            
                            blink_cursorpos =  cursorpos[0][1]; // richtungpfeilcursor
                            
                            }break;
                            case 2: // funktion
                            {
                            blink_cursorpos =  cursorpos[0][2];
                            }break;
                            
                            
                            } // switch curr_cursorspalte
                            }break;// case 0
                            
                            case 1: // Level
                            {
                            if (curr_cursorspalte < 2)
                            {
                            switch (curr_cursorspalte)
                            {
                            case 0:// kanalwert A
                            {
                            
                            blink_cursorpos =  cursorpos[1][0]; // kanalwert A
                            }break;
                            
                            case 1: // kanalwert B
                            
                            {
                            blink_cursorpos =  cursorpos[1][1]; // kanalwert B
                            
                            }break;
                            case 2:
                            {
                            blink_cursorpos =  1; //fehler, back
                            }break;
                            
                            } //  case curr_cursorspalte
                            }
                            }break;
                            
                            case 2: // Expo
                            {
                            if (curr_cursorspalte < 2)
                            {
                            switch (curr_cursorspalte)
                            {
                            case 0:// expowert A
                            {
                            
                            blink_cursorpos =  cursorpos[2][0]; // expowert A
                            }break;
                            
                            case 1: // expowert B
                            
                            {
                            
                            blink_cursorpos =  cursorpos[2][1]; // expowert B
                            
                            }break;
                            case 2:
                            {
                            blink_cursorpos =  1; //fehler, back
                            
                            }break;
                            
                            } //  case curr_cursorspalte
                            }
                            }break;
                            
                            case 3:
                            {
                            
                            }break;
                            
                            
                            }// switch cursorzeile                        }break;
                            manuellcounter=0;
                            } // if manuellcounter
                            
                            //display_kanaldiagramm (char_x, uchar_y, level, expo, uint8_t typ )
                            // level: 0-3 expo: 0-3
                            //display_kanaldiagramm (64, 7, curr_levelarray[curr_kanal], curr_expoarray[curr_kanal], 1);
                            
                            //manuellcounter=0;
                            */
                           
                        }break; // case kanalscreen
                           
                           
                        case MIXSCREEN: // Mixing
                        {
                           #pragma mark  5 MIXSCREEN
                           if (manuellcounter)
                           {
                              blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                              manuellcounter=0;
                           } // if manuellcounter
                           
                           /*
                            if (manuellcounter)
                            {
                            lcd_gotoxy(0,0);
                            lcd_puthex(curr_cursorzeile);
                            lcd_putc(' ');
                            lcd_puthex(curr_cursorspalte);
                            lcd_putc(' ');
                            //lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                            
                            switch (curr_cursorzeile)
                            {
                            case 0: // Mix 0
                            {
                            
                            switch (curr_cursorspalte)
                            {
                            case 0:
                            {
                            blink_cursorpos =  cursorpos[0][0]; // Typ
                            }break;
                            
                            case 1: // Kanal A
                            {
                            blink_cursorpos =  cursorpos[0][1]; // richtungpfeilcursor
                            
                            }break;
                            case 2: // Kanal B
                            {
                            blink_cursorpos =  cursorpos[0][2];
                            }break;
                            
                            
                            } // switch curr_cursorspalte
                            }break;// case 0
                            
                            case 1: // Mix 1
                            {
                            
                            switch (curr_cursorspalte)
                            {
                            case 0:// Typ
                            {
                            
                            blink_cursorpos =  cursorpos[1][0]; // Typ
                            }break;
                            
                            case 1: // kanalwert A
                            
                            {
                            blink_cursorpos =  cursorpos[1][1]; // Kanal A
                            
                            }break;
                            case 2:
                            {
                            blink_cursorpos =  cursorpos[1][2];// Kanal B
                            }break;
                            
                            } //  case curr_cursorspalte
                            
                            }break;
                            
                            case 2: //
                            {
                            }break;
                            
                            case 3:
                            {
                            
                            }break;
                            
                            
                            }// switch cursorzeile                        }break;
                            manuellcounter=0;
                            } // if manuellcounter
                            */
                           
                           
                        }break; // case mixscreen
                           
                        case ZUTEILUNGSCREEN: // Zuteilung
                        case AUSGANGSCREEN:
                        {
                           #pragma mark  5 AUSGANGSCREEN
                           if (manuellcounter)
                           {
                              blink_cursorpos =  cursorpos[curr_cursorzeile][curr_cursorspalte];
                              manuellcounter=0;
                           } // if manuellcounter
                           
                           
                           
                        }break; // case zuteilungscreen
                           
                           
                     }// switch curr_screen
                  } break; // 5
                     
                     
                  case 6:// cursor nach rechts
                  {
#pragma mark Taste 6
                     switch (curr_screen)
                     {
                        case HOMESCREEN: // home
                        {
                           
                        }break;

                        case SAVESCREEN: // save
                        {
                           if (posregister[curr_cursorzeile][curr_cursorspalte+1]<0xFFFF)
                           {
                               display_cursorweg();
                              
                              char_height_mul=1;
                              last_cursorspalte =curr_cursorspalte;
                              curr_cursorspalte++;
                              //lcd_putc('+');
                           }

                        }break;

                        case SETTINGSCREEN: // Settings
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              //lcd_gotoxy(0,1);
                              //if (curr_cursorspalte <1) // curr_cursorzeile ist >0,
                              if (posregister[curr_cursorzeile][curr_cursorspalte+1]<0xFFFF)
                              {
                                 if (curr_cursorzeile ==0)
                                 {
                                    char_height_mul=2;
                                 }
                                 display_cursorweg();
                                 
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte++;
                                 //lcd_putc('+');
                              }
                              else
                              {
                                 
                                 //lcd_putc('-');
                              }
                              manuellcounter=0;
                           }
                           else
                           {
                              switch((blink_cursorpos & 0xFF00)>>8) // zeile
                              {
                                 case 0: // modell
                                 {
                                    
                                 }break;
                                 case  4:
                                 {
                                    blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                    //
                                    
                              }// switch
                           }
                           
                           
                        }break;
                           
                        case KANALSCREEN: // Kanal
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile][curr_cursorspalte+1]<0xFFFF)
                              {
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte++;
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter)
                           {
                              switch((blink_cursorpos & 0xFF00)>>8) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (blink_cursorpos & 0x00FF) // cursorspalte
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          
                                       }   break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1:
                                 {
                                    //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter=0;
                           }
                           
                           
                        }break;
                           
                        case MIXSCREEN: // Mixing
                        {
                           lcd_gotoxy(0,0);
                           
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile][curr_cursorspalte+1]< 0xFFFF)
                              {
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte++;
                                 
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc(' ');
                                 lcd_puthex(curr_cursorspalte);
                                 lcd_putc(' ');
                                 lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                              }
                              manuellcounter=0;
                              
                           }
                           else
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          
                                       }   break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1:
                                 {
                                    //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                           }
                           
                           
                        }break;
                           
                           
                        case ZUTEILUNGSCREEN: // Zuteilung der Kanaele
                        {
                           lcd_gotoxy(0,0);
                           
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile][curr_cursorspalte+1]< 0xFFFF)
                              {
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte++;
                                 
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc(' ');
                                 lcd_puthex(curr_cursorspalte);
                                 lcd_putc(' ');
                                 lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                              }
                              manuellcounter=0;
                              
                           }
                           else
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          
                                       }   break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1:
                                 {
                                    //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                           }
                           
                           
                        }break;
                           
                           
                        case AUSGANGSCREEN: // Ausgang
                        {
                           lcd_gotoxy(0,0);
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc(' ');
                           lcd_puthex(curr_cursorspalte);
                           lcd_putc(' ');
                           
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile][curr_cursorspalte+1]< 0xFFFF)
                              {
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorspalte =curr_cursorspalte;
                                 
                                 curr_cursorspalte++;
                                 
                                 lcd_puthex(posregister[curr_cursorzeile][curr_cursorspalte+1]);
                              }
                              manuellcounter=0;
                              
                           }
                           else
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanaltext
                                 {
                                    switch (curr_cursorspalte) // cursorspalte
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          
                                       }   break;
                                    }
                                    // blink_cursorpos =  cursorpos[0][1]; // richtungcursor
                                    
                                    
                                 }break;
                                    
                                    
                                 case  1:
                                 {
                                    //blink_cursorpos =  cursorpos[1][1]; // settingcursor
                                 }break;
                                    
                                 case  2:
                                 {
                                    
                                 }break;
                                 case  3:
                                 {
                                    
                                 }break;
                                    //
                                    
                              }// switch
                           }
                           
                           
                        }break;
                           
                     }// switch
                     
                     manuellcounter=0;
                     
                     
                  }break;
                     
                     
                  case 7://home, in wenn 3* click aus default
                  {
#pragma mark Taste 7
                     //manuellcounter=0; // timeout zuruecksetzen
                     
                     if (curr_screen) // nicht homescreen
                     {
                        switch (curr_screen)
                        {
                           case HOMESCREEN:
                           {
                              display_clear();
                              curr_screen=0;
                              curr_cursorspalte=0;
                              curr_cursorzeile=0;
                              last_cursorspalte=0;
                              last_cursorzeile=0;
                              blink_cursorpos = 0xFFFF;
                              
                              //setsavescreen();
                              
                              sethomescreen();
                              
                              
                           }break;
                              
                           case SETTINGSCREEN: // Settings
                           {
                              if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                              {
                                 
                                 display_clear();
                                 curr_screen=HOMESCREEN;
                                 curr_cursorspalte=0;
                                 //curr_cursorzeile=0;
                                 last_cursorspalte=0;
                                 //last_cursorzeile=0;
                                 blink_cursorpos = 0xFFFF;
                                 settingstartcounter=0;
                                 startcounter=0;
                                 
                                 //sethomescreen();
                                 
                                 setsavescreen();
                                 
                                 manuellcounter=0;
                              }
                              else if (manuellcounter)
                              {
                                 
                                 blink_cursorpos = 0xFFFF;
                                 manuellcounter=0;
                              }
                              
                           }break;
                              
                           case KANALSCREEN: // Settings
                           {
                              if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                              {
                                 manuellcounter=0;
                                 
                                 blink_cursorpos = 0xFFFF;
                                 //
                                 if (curr_cursorspalte==0) // position am linken Rand
                                 {
                                    display_clear();
                                    curr_cursorzeile=0;
                                    last_cursorspalte=0;
                                    last_cursorzeile=0;
                                    curr_screen=SETTINGSCREEN;
                                    setsettingscreen();
                                 }
                                 else
                                 {
                                    
                                 }
                                 manuellcounter=0;
                              }
                              else if (manuellcounter)
                              {
                                 blink_cursorpos = 0xFFFF;
                                 manuellcounter=0;
                              }
                           }break;
                              
                              
                           case LEVELSCREEN: // Level einstellen
                           {
                              if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                              {
                                 manuellcounter=0;
                                 display_clear();
                                 curr_screen=KANALSCREEN;
                                 curr_cursorspalte=0;
                                 curr_cursorzeile=1; // Zeile Level
                                 last_cursorspalte=0;
                                 last_cursorzeile=0;
                                 blink_cursorpos = 0xFFFF;
                                 setcanalscreen();
                                 
                                 manuellcounter=0;
                              }
                              else if (manuellcounter)
                              {
                                 
                                 blink_cursorpos = 0xFFFF;
                                 manuellcounter=0;
                              }
                              
                           }break;
                              
                           case EXPOSCREEN: // Level einstellen
                           {
                              if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                              {
                                 manuellcounter=0;
                                 display_clear();
                                 curr_screen=KANALSCREEN;
                                 curr_cursorspalte=0;
                                 curr_cursorzeile=2; //Zeile Expo
                                 last_cursorspalte=0;
                                 last_cursorzeile=0;
                                 blink_cursorpos = 0xFFFF;
                                 setcanalscreen();
                                 
                                 manuellcounter=0;
                              }
                              else if (manuellcounter)
                              {
                                 
                                 blink_cursorpos = 0xFFFF;
                                 manuellcounter=0;
                              }
                              
                           }break;
                              
                           case MIXSCREEN: // Settings
                           {
                              if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                              {
                                 manuellcounter=0;
                                 
                                 blink_cursorpos = 0xFFFF;
                                 //
                                 if (curr_cursorspalte==0) // position am linken Rand
                                 {
                                    display_clear();
                                    curr_cursorzeile=0;
                                    last_cursorspalte=0;
                                    last_cursorzeile=0;
                                    curr_screen=SETTINGSCREEN;
                                    setsettingscreen();
                                 }
                                 else
                                 {
                                    
                                 }
                                 manuellcounter=0;
                              }
                              else if (manuellcounter)
                              {
                                 
                                 blink_cursorpos = 0xFFFF;
                                 manuellcounter=0;
                              }
                           }break;
                              
                           case ZUTEILUNGSCREEN: // Settings
                              
                           {
                              if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                              {
                                 manuellcounter=0;
                                 
                                 blink_cursorpos = 0xFFFF;
                                 //
                                 if (curr_cursorspalte==0) // position am linken Rand
                                 {
                                    display_clear();
                                    curr_cursorzeile=0;
                                    last_cursorspalte=0;
                                    last_cursorzeile=0;
                                    curr_screen=SETTINGSCREEN;
                                    setsettingscreen();
                                 }
                                 else
                                 {
                                    
                                 }
                                 manuellcounter=0;
                              }
                              else if (manuellcounter)
                              {
                                 
                                 blink_cursorpos = 0xFFFF;
                                 manuellcounter=0;
                              }
                           }break;
                              
                           case AUSGANGSCREEN: // Settings
                              
                           {
                              if ((blink_cursorpos == 0xFFFF) && manuellcounter)
                              {
                                 manuellcounter=0;
                                 
                                 blink_cursorpos = 0xFFFF;
                                 //
                                 if (curr_cursorspalte==0) // position am linken Rand
                                 {
                                    
                                    display_clear();
                                    curr_cursorzeile=0;
                                    last_cursorspalte=0;
                                    last_cursorzeile=0;
                                    curr_screen=SETTINGSCREEN;
                                    setsettingscreen();
                                 }
                                 else
                                 {
                                    
                                 }
                                 manuellcounter=0;
                              }
                              else if (manuellcounter)
                              {
                                 
                                 blink_cursorpos = 0xFFFF;
                                 manuellcounter=0;
                              }
                           }break;
                              
                        }// switch
                     }
                     else // schon homescreen, motorzeit reset
                     {
                        if (manuellcounter) // kurz warten
                        {
                           programmstatus &= ~(1<<MOTOR_ON);
                           motorsekunde=0;
                           motorminute=0;
                           manuellcounter=0; // timeout zuruecksetzen
                        }
                     }
                  }break;
                     
                     
                  case 8://
                  {
#pragma mark Taste 8
                     switch (curr_screen)
                     {
                        case HOMESCREEN: // home
                        {
                           
                        }break;
                           
                        case SETTINGSCREEN: // Settings
                        {
                           if ((blink_cursorpos == 0xFFFF) && manuellcounter) // kein Blinken
                           {
                              lcd_gotoxy(5,1);
                              lcd_puthex(curr_cursorzeile);
                              lcd_putc('*');
                              lcd_puthex((posregister[curr_cursorzeile+1][curr_cursorspalte]&0xFF00)>>8);
                              lcd_putc('*');
                              lcd_puthex((posregister[curr_cursorzeile+1][curr_cursorspalte]&0x00FF));
                              lcd_putc('*');
                              
                              //if (curr_cursorzeile < 3 )//
                              if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)
                              {
                                 if (curr_cursorzeile == 0)
                                 {
                                    char_height_mul = 2;
                                 }
                                 display_cursorweg();
                                 char_height_mul = 1;
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile++;
                                 
                                 lcd_gotoxy(19,1);
                                 lcd_putc('+');
                              }
                              else
                              {
                                 
                                 lcd_putc('-');
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter)
                           {
                              /*
                               lcd_gotoxy(0,1);
                               lcd_puthex((blink_cursorpos & 0xFF00)>>8);
                               lcd_putc('*');
                               lcd_puthex((blink_cursorpos & 0x00FF));
                               lcd_putc(' ');
                               */
                              //switch((blink_cursorpos & 0xFF00)>>8) // zeile
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // modell
                                 {
                                    //switch (blink_cursorpos & 0x00FF) // spalte
                                    switch (curr_cursorspalte) // spalte
                                    {
                                       case 0:
                                       {
                                          //lcd_putc('0');
                                          if (curr_model <8)
                                          {
                                             curr_model++;
                                          }
                                          
                                       }break;
                                          
                                       case 1:
                                       {
                                          //lcd_putc('1');
                                          if (curr_setting <4)
                                          {
                                             curr_setting++;
                                          }
                                          
                                       }break;
                                          
                                       case 2: //
                                       {
                                          
                                          
                                       }break;
                                          
                                    } // switch Spalte
                                    
                                 }break;
                                    
                                 case  1:
                                 {
                                    lcd_putc('1');
                                    
                                 }break;
                                    
                                 case  2:
                                 {
                                    lcd_putc('2');
                                 }break;
                                 case  3:
                                 {
                                    lcd_putc('3');
                                 }break;
                                    //
                                    
                              }// switch
                              manuellcounter =0;
                           }
                        }break;
                           
                        case KANALSCREEN: // Kanalsettings
                        {
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                              {
                                 if (curr_cursorzeile==1)
                                 {
                                    char_height_mul=2;
                                 }
                                 else
                                 {
                                    char_height_mul=1;
                                 }
                                 
                                 display_cursorweg();
                                 char_height_mul=1;
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile++;
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc('+');
                              }
                              else
                              {
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc('-');
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter)
                           {
                              switch(curr_cursorzeile) // zeile
                              {
                                 case 0: // Kanal
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // Kanalnummer
                                       {
                                          if (curr_kanal < 7)
                                          {
                                             curr_kanal++;
                                          }
                                       }break;
                                       case 1: // Richtung
                                       {
                                          if (curr_expoarray[curr_kanal] & 0x80)
                                          {
                                             curr_expoarray[curr_kanal] &= ~0x80;
                                          }
                                          else
                                          {
                                             curr_expoarray[curr_kanal] |= 0x80;
                                          }
                                       }break;
                                          
                                       case 2: // Funktion
                                       {
                                          //lcd_gotoxy(5,1);
                                          //lcd_putc('*');
                                          //Bezeichnung von: FunktionTable[curr_funktionarray[curr_kanal]]&0x07
                                          // Funktion ist bit 0-2, Steuerdevice ist bit 4-6!!
                                          uint8_t tempfunktion = curr_funktionarray[curr_kanal]&0x07; //bit 0-2
                                          tempfunktion++;
                                          tempfunktion &= 0x07;
                                          
                                          //lcd_puthex(tempfunktion);
                                          curr_funktionarray[curr_kanal] = (curr_funktionarray[curr_kanal]&0xF0)|tempfunktion; // cycle in FunktionTable
                                          
                                          
                                          /*
                                           if (tempfunktion<8)
                                           {
                                           curr_funktionarray[curr_kanal] += 1;
                                           }
                                           */
                                       }break;
                                          
                                    }// switch tempspalte
                                    
                                 }break;
                                    
                                 case  1: // Level
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // Levelwert A
                                       {
                                          if (((curr_levelarray[curr_kanal] & 0x70)>>4)<5)
                                          {
                                             curr_levelarray[curr_kanal] += 0x10;
                                          }
                                          
                                       }break;
                                       case 1: // Levelwert B
                                       {
                                          if (((curr_levelarray[curr_kanal] & 0x07))<5)
                                          {
                                             curr_levelarray[curr_kanal] += 0x01;
                                          }
                                          
                                       }break;
                                          
                                       case 2: //
                                       {
                                          curr_cursorspalte = 1; // fehler, back
                                          
                                       }break;
                                    }// switch tempspalte
                                 }break;
                                    
                                 case  2: // Expo
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // Expowert A
                                       {
                                          if (((curr_expoarray[curr_kanal] & 0x70)>>4)<3)
                                          {
                                             curr_expoarray[curr_kanal] += 0x10;
                                          }
                                       }break;
                                          
                                       case 1: // Expowert B
                                       {
                                          if (((curr_expoarray[curr_kanal] & 0x07))<3)
                                          {
                                             curr_expoarray[curr_kanal] += 0x01;
                                          }
                                       }break;
                                          
                                       case 2: //
                                       {
                                          curr_cursorspalte = 1; // fehler, back
                                          
                                       }break;
                                    }// switch tempspalte
                                 }break;
                                 case  4: // Typ
                                 {
                                 }break;
                              }// switch
                              manuellcounter=0;
                           }
                        }break; // kanalscreen
                           
                        case MIXSCREEN:
                        {
                           
                           lcd_gotoxy(5,1);
                           lcd_puthex(curr_cursorzeile);
                           lcd_putc('*');
                           lcd_puthex((blink_cursorpos & 0xFF00)>>8); // Zeile
                           lcd_putc('*');
                           lcd_putc('*');
                           
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                              {
                                 char_height_mul=1;
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile++;
                                 
                                 lcd_puthex(curr_cursorzeile);
                                 lcd_putc('+');
                              }
                              else
                              {
                                 //lcd_puthex(curr_cursorzeile);
                                 //lcd_putc('-');
                              }
                              //lcd_putint2(curr_cursorzeile);
                              
                              //lcd_putc(' ');
                              
                              
                              manuellcounter=0;
                           }
                           else if (manuellcounter) // blinken ist on
                           {
                              /*
                               lcd_gotoxy(0,0);
                               lcd_puthex(curr_cursorzeile);
                               lcd_putc(' ');
                               lcd_puthex(curr_cursorspalte);
                               lcd_putc(' ');
                               */
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Mix weiterschalten
                                 {
                                    if (curr_mixarray[2*curr_cursorzeile+1]<4)
                                    {
                                       curr_mixarray[2*curr_cursorzeile+1] += 0x01;// Mix ist auf ungerader zeile
                                    }
                                 }break;
                                    
                                 case 1: // Kanal A weiterschalten
                                 {
                                    uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0x70)>>4;// kanal a ist auf gerader zeile in bit 4-6
                                    
                                    if (tempdata < 8) //
                                    {
                                       curr_mixarray[2*curr_cursorzeile] += 0x10;
                                    }
                                    
                                    
                                 }break;
                                    
                                 case 2: // Kanal B weiterschalten
                                 {
                                    uint8_t tempdata =(curr_mixarray[2*curr_cursorzeile] & 0x07);// kanal b ist auf gerader zeile in bit 0-2
                                    
                                    if (tempdata < 8)
                                    {
                                       curr_mixarray[2*curr_cursorzeile] += 0x01;
                                    }
                                    
                                 }break;
                                    
                              }// switch curr_cursorspalte
                              
                              manuellcounter = 0;
                              
                           }
                        }break; // mixscreen
                           
                           
                        case ZUTEILUNGSCREEN:
                        {
#pragma mark 8 ZUTEILUNGSCREEN
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                              {
                                 char_height_mul=1;
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 
                                 curr_cursorzeile++;
                                 
                                 //lcd_puthex(curr_cursorzeile);
                                 //lcd_putc('+');
                              }
                              else
                              {
                                 //lcd_puthex(curr_cursorzeile);
                                 //lcd_putc('-');
                              }
                              manuellcounter=0;
                           }
                           else if (manuellcounter) // blinken ist on
                           {
                              //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                              /*
                               const char funktion0[] PROGMEM = "Seite  \0";
                               const char funktion1[] PROGMEM = "Hoehe  \0";
                               const char funktion2[] PROGMEM = "Quer   \0";
                               const char funktion3[] PROGMEM = "Motor  \0";
                               const char funktion4[] PROGMEM = "Quer L\0";
                               const char funktion5[] PROGMEM = "Quer R\0";
                               const char funktion6[] PROGMEM = "Lande  \0";
                               const char funktion7[] PROGMEM = "Aux    \0";
                               
                               */
                              
                              lcd_gotoxy(0,0);
                              lcd_puthex(curr_cursorzeile);
                              lcd_putc(' ');
                              lcd_puthex(curr_cursorspalte);
                              lcd_putc(' ');
                              
                              switch (curr_cursorzeile)
                              {
                                 case 0: // pitch vertikal
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // L_V index 1
                                       {
                                          // Kanalnummer im Devicearray increment
                                          if (((curr_devicearray[1]& 0x07))<8)
                                          {
                                             curr_devicearray[1]+= 0x01;
                                          }
                                       }break;
                                          
                                       case 1: // R_V index 3
                                       {
                                          if (((curr_devicearray[3]& 0x07))<8)
                                          {
                                             curr_devicearray[3]+= 0x01;
                                          }
                                       }break;
                                    }// switch curr_cursorspalte
                                 }break; // pitch v
                                    
                                 case 1: // pitch horizontal
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // L_H index 0
                                       {
                                          if (((curr_devicearray[0]& 0x07))<8)
                                          {
                                             // Kanalnummer fuer Device increment
                                             curr_devicearray[0]+= 0x01;
                                          }
                                       }break;
                                          
                                       case 1: // R_H index 2
                                       {
                                          if (((curr_devicearray[2]& 0x07))<8)
                                          {
                                             curr_devicearray[2]+= 0x01;
                                          }
                                       }break;
                                    }// switch curr_cursorspalte
                                 }break; // case spalte
                                    
                                    
                                    
                                 case 2: // schieber
                                 {
                                    switch (curr_cursorspalte)
                                    {
                                       case 0: // S_L index 4
                                       {
                                          if (((curr_devicearray[4]& 0x07))<8)
                                          {
                                             // Kanalnummer fuer Device increment
                                             curr_devicearray[4]+= 0x01;
                                          }
                                       }break;
                                          
                                       case 1: // S_R index 5
                                       {
                                          if (((curr_devicearray[5]& 0x07))<8)
                                          {
                                             curr_devicearray[5]+= 0x01;
                                          }
                                       }break;
                                    }// switch curr_cursorspalte
                                 }break; // case spalte
                                    
                              }//switch curr_cursorzeile
                              manuellcounter = 0;
                              
                           } // else if manuellcounter
                           
                        }break; // case zuteilung
                           
                        case AUSGANGSCREEN:
                        {
#pragma mark 8 AUSGANGSCREEN
                           if (blink_cursorpos == 0xFFFF && manuellcounter) // Kein Blinken
                           {
                              if (posregister[curr_cursorzeile+1][curr_cursorspalte]<0xFFFF)//
                              {
                                 char_height_mul=1;
                                 display_cursorweg();
                                 last_cursorzeile =curr_cursorzeile;
                                 if ((curr_cursorzeile < 3) || (curr_impuls > 3)) // Noch vor scrollen oder nach umschalten
                                 {
                                    curr_cursorzeile++;
                                 }
                                 else
                                 {
                                    curr_cursorzeile = 1; // Scroll
                                 }
                                 curr_impuls++;
                              }
                              
                              manuellcounter=0;
                           }
                           else if (manuellcounter) // blinken ist on
                           {
                              //funktionarray: bit 0-3: Kanal bit 4-7: Zuteilung an Pitch/Schieber/Schalter
                              /*
                               const char funktion0[] PROGMEM = "Seite  \0";
                               const char funktion1[] PROGMEM = "Hoehe  \0";
                               const char funktion2[] PROGMEM = "Quer   \0";
                               const char funktion3[] PROGMEM = "Motor  \0";
                               const char funktion4[] PROGMEM = "Quer L\0";
                               const char funktion5[] PROGMEM = "Quer R\0";
                               const char funktion6[] PROGMEM = "Lande  \0";
                               const char funktion7[] PROGMEM = "Aux    \0";
                               */
                              
                              lcd_gotoxy(0,0);
                              lcd_puthex(curr_cursorzeile);
                              lcd_putc(' ');
                              lcd_puthex(curr_cursorspalte);
                              lcd_putc(' ');
                              
                              switch (curr_cursorspalte)
                              {
                                 case 0: // Kanal
                                 {
                                    // Kanalnummer im Devicearray increment
                                    if (((curr_ausgangarray[curr_cursorzeile]& 0x07))<8)
                                    {
                                       curr_ausgangarray[curr_cursorzeile]+= 0x01;
                                    }
                                 }break;
                                    
                                 case 1: // Zeile nach unten verschieben
                                 {
                                    if (((curr_ausgangarray[curr_cursorzeile]& 0x07))<8)
                                    {
                                       uint8_t tempzeilenwert =curr_ausgangarray[curr_cursorzeile];
                                       if (curr_impuls < 7) // nicht letzte Zeile
                                       {
                                          if ((curr_cursorzeile < 3) && (curr_impuls < 3)) // Noch vor scrollen, auf erster Seite
                                          {
                                             tempzeilenwert =curr_ausgangarray[curr_impuls];
                                             
                                             curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls+1]; // Wert von naechster zeile
                                             curr_ausgangarray[curr_impuls +1] = tempzeilenwert;
                                             // cursorzeile verschieben
                                             display_cursorweg();
                                             
                                             curr_cursorzeile++;
                                             // blink-cursorzeile verschieben
                                             blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                             
                                          }
                                          else  if ((curr_cursorzeile == 3) && (curr_impuls == 3))// zweitunterste Zeile, scrollen.
                                          {
                                             tempzeilenwert =curr_ausgangarray[curr_impuls];
                                             curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls+1]; // Wert von naechster zeile, noch auf dieser Seite
                                             curr_ausgangarray[curr_impuls +1] = tempzeilenwert;
                                             display_cursorweg();
                                             curr_cursorzeile = 1; // Scroll
                                             // blink-cursorzeile verschieben
                                             blink_cursorpos = cursorpos[1][curr_cursorspalte];
                                          }
                                          else  if ((curr_cursorzeile < 4) && (curr_impuls >3))// zweite Zeile oder mehr auf zweiter Seite
                                          {
                                             tempzeilenwert =curr_ausgangarray[curr_impuls];
                                             curr_ausgangarray[curr_impuls] =curr_ausgangarray[curr_impuls+1]; // Wert von naechster zeile
                                             curr_ausgangarray[curr_impuls +1] = tempzeilenwert;
                                             // cursorzeile verschieben
                                             display_cursorweg();
                                             
                                             curr_cursorzeile++;
                                             // blink-cursorzeile verschieben
                                             blink_cursorpos = cursorpos[curr_cursorzeile][curr_cursorspalte];
                                             
                                          }
                                          curr_impuls++;
                                       }
                                       else // letzte Zeile, mit erster zeile vertauschen
                                       {
                                          /*
                                           tempzeilenwert =curr_ausgangarray[curr_impuls];
                                           curr_ausgangarray[curr_impuls] =curr_ausgangarray[0]; // Wert von erster zeile
                                           curr_ausgangarray[0] = tempzeilenwert;
                                           display_cursorweg();
                                           curr_cursorzeile=0;
                                           curr_impuls =0;
                                           blink_cursorpos = cursorpos[0][curr_cursorspalte];
                                           */
                                       }
                                    }
                                    
                                 }break;
                                    
                                    
                              }// switch curr_cursorspalte
                              manuellcounter = 0;
                              
                           } // else if manuellcounter
                           
                        }break; // case ausgang
                           
                     }// switch
                     
                     
                     
                     
                  }break; // case 8
                     
                  case 9://set, out wenn auf home
                  {
#pragma mark Taste 9
                     if (manuellcounter) // kurz warten
                     {
                        programmstatus &= ~(1<<STOP_ON);
                        stopsekunde=0;
                        stopminute=0;
                        
                        manuellcounter=0; // timeout zuruecksetzen
                     }
                     
                  }break;
                     
                  case 10:// *Manuell einschalten
                  {
#pragma mark Taste 10
                  }break;
                     
                  case 11://
                  {
#pragma mark Taste 11
                  }break;
                     
                  case 12: // # Normalbetrieb einschalten
                  {
#pragma mark Taste 12
                     
                  }break;
                     
               }//switch Tastatur
               
               Tastenwert=0;
            }//if TastaturCount
            
         } // if Tastenwert > 5
         //OSZI_B_HI;
      }
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
