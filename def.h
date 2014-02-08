#define CPU_16MHz       0x00
#define CPU_8MHz        0x01

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

/*
#define E_TASTE0				  6 // Einzeltaste //
#define E_TASTE1             7 // Analog Comparator
*/

#define OFF_DDR               DDRE
#define OFF_PORT              PORTE
#define OFF_PIN               PINE

#define OFF_DETECT            7 // Analog Comparator

// EEPROM Speicherorte

#define TASK_OFFSET        0x2000 // Ort fuer Einstellungen

#define SETTINGBREITE      0x100; // 256 Bytes, Breite des Settingblocks fuer ein model

#define  MITTE_OFFSET      0x10 // 16
#define  LEVEL_OFFSET      0x20 // 32
#define  EXPO_OFFSET       0x30 // 48
#define  MIX_OFFSET        0x40 // 64

#define FUNKTION_OFFSET    0x60 // 96
#define DEVICE_OFFSET      0x70 // 122
#define AUSGANG_OFFSET     0x80 // 128


#define SAVE_LEVEL   0
#define SAVE_MIX  2
#define SAVE_EXPO 3
#define SAVE_FUNKTION 4
#define SAVE_DEVICE 5
#define SAVE_AUSGANG 6


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
 #define TASTE_L     234
 #define TASTE0		248
 #define TASTE_R     255
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





#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s


//#define MITTE_TASK         0x01 // Mitte lesen
//#define KANAL_TASK         0x02 // Level und Expo lesen
//#define MIX_TASK           0x03 // Mix lesen


#define MS_DIV          4	// Bit 4 von Status. Gesetzt wenn 1s abgelaufen
#define UPDATESCREEN    5 // Bit in status wird gesetzt wenn eine Taste gedrueckt ist, reset wenn update ausgefuehrt

#define SETTINGWAIT     6  // Bit in status wird gesetzt bis Taste 5 3* gedrueckt ist

//#define MANUELL			7	// Bit 7 von Status

#define MINWAIT         3 // Anzahl loops von loopcount1 bis einschalten


#define TASTATURPORT PORTF
#define TASTATURPIN		1


// end Screen




// SPI



#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7





// ADC
#define ADC_PORT            PORTF   //    PORTF
#define ADC_DDR             DDRF    //    DDRF
#define ADC_PIN             PINF    //    PINF

#define ADC_AKKUPIN         0


// Bit


// bits von programmstatus

#define MOTOR_ON        1
#define STOP_ON         2
#define EEPROM_TASK     3  // Daten in EEPROM sichern
#define USB_ATTACH_TASK  4  // USB initiieren



// Bits von masterstatus
#define  SUB_TASK_BIT             4 // Sub hat Aufgaben
#define  SUB_READ_EEPROM_BIT      5 // Sub soll EEPROM lesen
#define  HALT_BIT                7 //Bit 7

// Bits von eepromstatus
#define READ_EEPROM_START        0  // Beim Start gesetzt. Soll einmaliges Lesen der Settings beim Update des Masters ausloesen


// Bits von displaystatus
#define UHR_UPDATE         0
#define BATTERIE_UPDATE    1


// Task fuer substatus
#define TASTATUR_READ   0
#define TASTATUR_OK     1
#define SETTINGS_READ    2

#define SCREEN_OK       4
#define SCREEN_REFRESH  5
#define UHR_OK          6
#define UHR_REFRESH     7


// Bits fuer usbstatus
#define USB_RECV  0

#define USB_ATTACH            1 // USB_Spannung detektiert


#define ANZ_POT               6 // Anzahl zu lesender Potis

#define POT_FAKTOR            1.20 // Korrekturfaktor fuer Potentiometerstellung


#define MASTER_PORT            PORTD   //    
#define MASTER_DDR             DDRD    //    
#define MASTER_PIN             PIND    //
// PIN's
#define SUB_BUSY_PIN             5 // Ausgang fuer busy-Meldung des Sub an Master

#define INTERRUPT_PORT            PORTB   //
#define INTERRUPT_DDR             DDRB    //
#define INTERRUPT_PIN             PINB    //
// PIN's
#define MASTER_EN_PIN            7 // Eingang fur PinChange-Interrupt vom Master

#define SUB_EN_PORT              PORTE // Gate-Zugang zu EE und RAM fuer Memory-Zugriffe  des Sub
#define SUB_EN_DDR               DDRE  // mit RAM_CS_HI, EE_CS_HI
// PIN's
#define SUB_EN_PIN               0

#define USB_PORT            PORTD   //
#define USB_DDR             DDRD    //
#define USB_PIN             PIND    //
// PIN's
#define USB_DETECT_PIN      3







#define EEPROM_WRITE_BYTE_TASK     1
#define EEPROM_WRITE_PAGE_TASK     2
#define EEPROM_READ_BYTE_TASK      3
#define EEPROM_READ_PAGE_TASK      4
#define EEPROM_AUSGABE_TASK        5

#define EEPROM_WRITE_START_OK    0xB0


// RAM-Tasks

#define READ_TASKADRESSE         0x1FA     // RAM_Adresse fuer  Task-Auftrag an RC_LCD
#define READ_TASKDATA            0x1FB  

#define WRITE_TASKADRESSE        0x1F0     // RAM_Adresse fuer Task-Auftrag von RC_LCD
#define WRITE_TASKDATA           0x1F1

#define RAM_SEND_PPM_TASK         2 // PPM soll Status lesen (Auftrag AN PPM)
#define RAM_RECV_PPM_TASK         1 // LCD soll Status lesen (Auftrag VON PPM)
#define RAM_TASK_OK           7 // PPM hat Task gelesen
