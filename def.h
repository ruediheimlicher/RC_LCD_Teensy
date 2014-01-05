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

#define TASTENDDR           DDRD
#define TASTENPORT          PORTD
#define TASTENPIN           PIND

#define TASTE0				   0
#define TASTE1             1


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

#define MITTE_TASK            0x01 // Mitte lesen
#define KANAL_TASK            0x02 // Level und Expo lesen
#define MIX_TASK              0x03 // Mix lesen





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


#define KANAL_PIN          4                             // Ausgang fuer Summensignal
#define KANAL_LO           KANAL_PORT &= ~(1<<KANAL_PIN)
#define KANAL_HI           KANAL_PORT |= (1<<KANAL_PIN)


// ADC
// CNC12
#define ADC_PORT            PORTF   //    PORTF
#define ADC_DDR             DDRF    //    DDRF
#define ADC_PIN             PINF    //    PINF


// Bit

// Masterstatus

#define RAM_READ                 0x01  //Bit 0
#define RAM_WRITE                0x02  //Bit 1
#define EEPROM_READ              0x04  //Bit 2
#define EEPROM_WRITE             0x08  //Bit 3

#define  POT_READ          0x10  //Bit 4
#define  HALT_BIT              7 //Bit 7

#define ADC_START 0  //    Start Messung Batteriespannung mit internem ADC

#define POT_START 0  //    Start Messung Potentiometer

#define SPI_START 2  //    Start SPI auf diesem device

#define SPI_END   3  //    End SPI auf diesem device

#define POT_MITTE 7  //    Mittelwerte der Potentiometer speichern

#define ANZ_POT   6

#define POT_FAKTOR 1.20


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

#define SUB_READ_EEPROM_BIT      5 // Sub soll EEPROM lesen

#define SUB_TASK_BIT             4 // Sub hat Aufgaben

#define MASTER_EN_BIT            0 // Master erlaubt SPI

#define MEM_EN_PORT              PORTE // CS fuer Memory-Zugriffe des Masters
#define MEM_EN_DDR               DDRE
#define MEM_EN_PIN               0


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
