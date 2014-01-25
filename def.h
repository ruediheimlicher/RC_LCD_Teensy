//Oszi
#define OSZIPORT           PORTA
#define OSZIPORTDDR        DDRA
#define OSZIPORTPIN        PINA
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1
#define OSZI_PULS_C        2
#define OSZI_PULS_D        3


#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)

#define OSZI_C_LO OSZIPORT &= ~(1<<OSZI_PULS_C)
#define OSZI_C_HI OSZIPORT |= (1<<OSZI_PULS_C)
#define OSZI_C_TOGG OSZIPORT ^= (1<<OSZI_PULS_C)

#define OSZI_D_LO OSZIPORT &= ~(1<<OSZI_PULS_D)
#define OSZI_D_HI OSZIPORT |= (1<<OSZI_PULS_D)
#define OSZI_D_TOGG OSZIPORT ^= (1<<OSZI_PULS_D)



#define LOOPLEDDDR          DDRD    
#define LOOPLEDPORT         PORTD   
#define LOOPLED             6       // fix verdrahtet

#define TASTENDDR           DDRE
#define TASTENPORT          PORTE
#define TASTENPIN           PINE

#define E_TASTE0				  6 // Einzeltaste
#define E_TASTE1             7


// EEPROM Speicherorte

#define TASK_OFFSET       0x2000 // Ort fuer Einstellungen

#define SETTINGBREITE      0x100; // 256 Bytes, Breite des Settingblocks fuer ein model

#define  MITTE_OFFSET      0x10 // 16
#define  LEVEL_OFFSET      0x20 // 32
#define  EXPO_OFFSET       0x30 // 48
#define  MIX_OFFSET        0x40 // 64

#define FUNKTION_OFFSET    0x60 // 96
#define DEVICE_OFFSET      0x70 // 122
#define AUSGANG_OFFSET     0x80 // 128

#define MITTE_TASK         0x01 // Mitte lesen
#define KANAL_TASK         0x02 // Level und Expo lesen
#define MIX_TASK           0x03 // Mix lesen

// Tastatur
// Atmega168
/*
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
 */
/*
// Atmega328
#define TASTE1		17
#define TASTE2		29
#define TASTE3		44
#define TASTE4		67
#define TASTE5		94
#define TASTE6		122
#define TASTE7		155
#define TASTE8		190
#define TASTE9		214
#define TASTE_L	234
#define TASTE0		252
#define TASTE_R	255
*/
// Teensy2 int ref/TL431
#define TASTE1		15
#define TASTE2		23
#define TASTE3		34
#define TASTE4		51
#define TASTE5		72
#define TASTE6		94
#define TASTE7		120
#define TASTE8		141
#define TASTE9		155
#define TASTE_L	168
#define TASTE0		178
#define TASTE_R	194

// Screen

#define DOGM_PORT	PORTD
#define DOGM_DDR	DDRD

#define DOGM_MOSI_PIN	0
#define DOGM_SCL_PIN	1
#define DOGM_CS_PIN   2
#define DOGM_CMD_PIN	3




#define MANUELLPIN		3	// Pin 6 von PORT D fuer Anzeige Manuell

#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s


// bits von programmstatus

#define MOTOR_ON        1
#define STOP_ON         2
#define EEPROM_TASK     3  // Daten in EEPROM sichern

#define MS_DIV          4	// Pin 4 von Status. Gesetzt wenn 1s abgelaufen
#define UPDATESCREEN    5 // Pin in status wird gesetzt wenn eine Taste gedrueckt ist, reset wenn update ausgefuerht

#define SETTINGWAIT     6  // Pin in status wird gesetzt bis Taste 5 3* gedrueckt ist

#define MANUELL			7	// Bit 7 von Status



#define MINWAIT         3 // Anzahl loops von loopcount1 bis einschalten


#define TASTATURPORT PORTF
#define TASTATURPIN		1


// end Screen




// SPI



#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   
#define CMD_DDR             DDRD    
#define CMD_PIN             PIND


#define KANAL_PORT            PORTB   //    PORTB
#define KANAL_DDR             DDRB    //    DDRB




// ADC
// CNC12
#define ADC_PORT            PORTF   //    PORTF
#define ADC_DDR             DDRF    //    DDRF
#define ADC_PIN             PINF    //    PINF

#define ADC_AKKUPIN         0


// Bit

// Masterstatus

#define RAM_READ                 0x01  //Bit 0
#define RAM_WRITE                0x02  //Bit 1
#define EEPROM_READ              0x04  //Bit 2
#define EEPROM_WRITE             0x08  //Bit 3

#define  POT_READ          0x10  //Bit 4
#define  HALT_BIT              7 //Bit 7

// Bits von eepromstatus
#define READ_EEPROM_START  0  // Beim Start gesetzt. Soll einmaliges Lesen der Settings beim Update des Masters ausloesen



// Bits von displaystatus

#define UHR_UPDATE         0
#define BATTERIE_UPDATE    1

#define ADC_START          0  //    Start Messung Batteriespannung mit internem ADC

#define POT_START          0  //    Start Messung Potentiometer

#define SPI_START          2  //    Start SPI auf diesem device

#define SPI_END            3  //    End SPI auf diesem device

#define POT_MITTE          7  //    Mittelwerte der Potentiometer speichern

#define ANZ_POT            6

#define POT_FAKTOR         1.20


#define EE_WREN   0
#define EE_WRITE  1
#define EE_READ   2

#define MASTER_PORT            PORTD   //    
#define MASTER_DDR             DDRD    //    
#define MASTER_PIN             PIND    //

#define INTERRUPT_PORT            PORTB   //
#define INTERRUPT_DDR             DDRB    //
#define INTERRUPT_PIN             PINB    //

#define MASTER_EN_PIN            7 // Eingang fur PinChange-Interrupt

#define SUB_BUSY_PIN             5 // Ausgang fuer busy-Meldung des Sub an Master

#define POWER_OFF_PORT            PORTD   //
#define POWER_OFF_DDR             DDRD    //
#define POWER_OFF_PIN             PIND    //

#define POWER_OFF_DETECT_PIN      3


#define SUB_READ_EEPROM_BIT      5 // Sub soll EEPROM lesen

#define SUB_TASK_BIT             4 // Sub hat Aufgaben

#define MASTER_EN_BIT            0 // Master erlaubt SPI

#define MEM_EN_PORT              PORTC// CS fuer Memory-Zugriffe des Masters
#define MEM_EN_DDR               DDRC
#define MEM_EN_PIN               7


#define TOUCH_AB_PORT            PORTB
#define TOUCH_AB_DDR             DDRB
#define TOUCH_XY_PORT            PORTF
#define TOUCH_XY_DDR             DDRF


#define EEPROM_WRITE_BYTE_TASK     1
#define EEPROM_WRITE_PAGE_TASK     2
#define EEPROM_READ_BYTE_TASK       3
#define EEPROM_READ_PAGE_TASK       4
#define EEPROM_AUSGABE_TASK         5

#define EEPROM_WRITE_START_OK    0xB0


// RAM-Tasks

#define READ_TASKADRESSE         0x1FA     // RAM_Adresse fuer  Task-Auftrag an RC_LCD
#define READ_TASKDATA            0x1FB  

#define WRITE_TASKADRESSE        0x1F0     // RAM_Adresse fuer Task-Auftrag von RC_LCD
#define WRITE_TASKDATA           0x1F1

#define RAM_SEND_PPM_TASK         2 // PPM soll Status lesen (Auftrag AN PPM)
#define RAM_RECV_PPM_TASK         1 // LCD soll Status lesen (Auftrag VON PPM)
#define RAM_TASK_OK           7 // PPM hat Task gelesen
