/*
 *  adc.c
 *  TWI_Master
 *
 *  Created by Sysadmin on 12.11.07.
 *  Copyright 2007 Ruedi Heimlicher. All rights reserved.
 *
 */

#include "adc.h"
#include <avr/io.h>


struct adcwert16 ADCWert16;


struct adcwert16 readKanal16Bit(uint8_t kanal)
{
	
	struct adcwert16 tempWert;
	tempWert.wertH=0;
	tempWert.wertL=0;
	tempWert.wert8H=0;
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);    // Frequenzvorteiler auf 32 setzen und ADC aktivieren 
	
	ADMUX = kanal;                      // Ÿbergebenen Kanal waehlen
	//ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen 
	ADMUX |=  (1<<REFS0); // VCC als Referenzspannung nutzen 
	
	/* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
	ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
	while ( ADCSRA & (1<<ADSC) ) {
		;     // auf Abschluss der Wandlung warten 
	}
    ADCSRA |= (1<<ADSC);            // eine Wandlung
    while ( ADCSRA & (1<<ADSC) ) {
		;     // auf Abschluss der Wandlung warten 
    }
	
	tempWert.wertL=ADCL;            //Read 8 low bits first (important)
	tempWert.wertH=ADCH;
	tempWert.wert8H=(ADCW<<2);
	
	// value|=((int)ADCH << 8); //read 2 high bits and shift into top byte
	
	ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
	
	return tempWert;
}

void initADC(uint8_t derKanal)
{
   //ADCSRA = (1<<ADEN) |(1<<ADPS2) | (1<<ADPS0);
   ADCSRA = (1<<ADEN) | ADC_PRESCALER;       // Frequenzvorteiler auf 32 setzen und ADC aktivieren
   ADCSRB = (1<<ADHSM) | (derKanal & 0x20);  // high speed mode
  
   ADMUX = aref | (derKanal & 0x1F);         // configure mux input und Ÿbergebenen Kanal waehlen
   
   
   
   
   ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen
   //ADMUX |= (1<<REFS0); // VCC als Referenzspannung nutzen
   
   /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
    also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
   ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
   while ( ADCSRA & (1<<ADSC) ) {
      ;     // auf Abschluss der Wandlung warten
   }
}


int16_t adc_read(uint8_t derKanal)
{
   uint16_t result = 0;
   uint8_t low, i ;
   ADCSRA = (1<<ADEN) | ADC_PRESCALER;             // enable ADC  f/64
   
   ADCSRB = (1<<ADHSM) | (derKanal & 0x20);             // high speed mode
   //ADMUX = aref | (derKanal & 0x1F);                    // configure mux input
   
   ADMUX = ADC_REF_POWER | (derKanal & 0x1F); // Vcc als Referenz
   
   
   for(i=0;i<4;i++)
   {
      ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0) | (1<<ADSC); // start the conversion
      while (ADCSRA & (1<<ADSC)) ;                    // wait for result
      low = ADCL;                                     // must read LSB first
      result +=(ADCH << 8) | low;
   }
   result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
   return result;
}


uint16_t readKanal(uint8_t derKanal) //Unsere Funktion zum ADC-Channel aus lesen
{
   uint8_t i;
   uint16_t result = 0;         //Initialisieren wichtig, da lokale Variablen
   //nicht automatisch initialisiert werden und
   //zufŠllige Werte haben. Sonst kann Quatsch rauskommen
   ADMUX |= derKanal;
   // Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen
   for(i=0;i<2;i++)
   {
      ADCSRA |= (1<<ADIF);
      ADCSRA |= (1<<ADSC);            // eine Wandlung
      while ( ADCSRA & (1<<ADSC) ) {
         ;     // auf Abschluss der Wandlung warten
      }
      //result += ADCW;            // Wandlungsergebnisse aufaddieren
   result += ADCL;
   }
   //  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
   
   result /= 2;                     // Summe durch vier teilen = arithm. Mittelwert
      
   return result;
}

void closeADC()
{
ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
}

uint16_t readKanalOrig(uint8_t derKanal, uint8_t num) //Unsere Funktion zum ADC-Channel aus lesen
{
  uint8_t i;
  uint16_t result = 0;         //Initialisieren wichtig, da lokale Variablen
                               //nicht automatisch initialisiert werden und
                               //zufŠllige Werte haben. Sonst kann Quatsch rauskommen
 
   ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);    // Frequenzvorteiler auf 32 setzen und ADC aktivieren 
 
  ADMUX = derKanal;                      // Ÿbergebenen Kanal waehlen
//  ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen 
  ADMUX |= (1<<REFS0); // VCC als Referenzspannung nutzen 
 
  /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
  ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
  while ( ADCSRA & (1<<ADSC) ) {
     ;     // auf Abschluss der Wandlung warten 
  }
 
  // Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen
  for(i=0;i<4;i++)
  {
    ADCSRA |= (1<<ADSC);            // eine Wandlung
    while ( ADCSRA & (1<<ADSC) ) {
      ;     // auf Abschluss der Wandlung warten 
    }
    result += ADCW;            // Wandlungsergebnisse aufaddieren
  }
  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
 
  result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
 
  return result;
}



