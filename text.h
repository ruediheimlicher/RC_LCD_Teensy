//
//  text.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//

#ifndef DOG_LCD_text_h
#define DOG_LCD_text_h

// Homescreen
const char titel0[] PROGMEM = "RC Home\0";
const char titel1[] PROGMEM = "ON-Zeit:\0";
const char titel2[] PROGMEM = "Stoppuhr\0";
const char titel3[] PROGMEM = "Motorzeit\0";
const char titel4[] PROGMEM = "Menu";
const char titel5[] PROGMEM = "Set";
const char titel6[] PROGMEM = "Akku\0";
const char titel7[] PROGMEM = "D\0";

PGM_P const TitelTable[] PROGMEM = {titel0, titel1, titel2, titel3, titel4, titel5, titel6, titel7};

// Modelle
const char model0[] PROGMEM = "E-Segler A  ";
const char model1[] PROGMEM = "E-Segler B  ";
const char model2[] PROGMEM = "Hangsegler A\0";
const char model3[] PROGMEM = "Motor A     \0";
const char model4[] PROGMEM = "Motor B     \0";
const char model5[] PROGMEM = "AA\0        ";
const char model6[] PROGMEM = "BB\0        ";
const char model7[] PROGMEM = "CC\0        ";

PGM_P const ModelTable[] PROGMEM = {model0, model1, model2, model3, model4, model5, model6, model7};





// Settingscreen
const char menutitel[] PROGMEM = "Settings";
const char model[] PROGMEM = "Modell";
const char setting[] PROGMEM = "Set";
const char kanal[] PROGMEM = "Kanal";

const char mix[] PROGMEM = "Mix";
const char zuteilung[] PROGMEM = "Zuteilung";
const char ausgang[] PROGMEM = "Ausgang";


PGM_P const SettingTable[] PROGMEM = {menutitel, model, setting, kanal,  mix, zuteilung,ausgang};


// Kanalscreen
const char kanaltitel[] PROGMEM = "Kan:";
const char richtung[] PROGMEM = "Ri:";
const char level[] PROGMEM = "Level";
const char expo[] PROGMEM = "Expo";

const char seitea[] PROGMEM = "A:";
const char seiteb[] PROGMEM = "B:";
const char kanaltyp[] PROGMEM = "Typ:";

PGM_P const KanalTable[] PROGMEM = {kanaltitel, richtung, level, expo, seitea, seiteb,kanaltyp};


// Kanaltyp
const char pitchtyp[] PROGMEM = "Pitch";
const char schiebertyp[] PROGMEM = "Schieber";
const char schaltertyp[] PROGMEM = "Schalter";

PGM_P const KanalTypTable[] PROGMEM = {pitchtyp,schiebertyp,schaltertyp};


// Mix
const char mixtitel[] PROGMEM = "Mix";
PGM_P const MixTable[] PROGMEM = {mixtitel};

// Mixtyp
const char nada[] PROGMEM = " -    ";
const char vmix[] PROGMEM = "V-Mix";
const char butterfly[] PROGMEM = "B-fly";
const char A[] PROGMEM = "A    ";

PGM_P const MixTypTable[] PROGMEM = {nada,vmix,butterfly,A};

// Zuteilung
const char zuteilungtitel[] PROGMEM = "Zuteilung";
PGM_P const ZuteilungTable[] PROGMEM = {zuteilungtitel};



// Funktion

const char funktion0[] PROGMEM = "Seite \0";
const char funktion1[] PROGMEM = "Hoehe \0";
const char funktion2[] PROGMEM = "Quer   \0";
const char funktion3[] PROGMEM = "Motor \0";
const char funktion4[] PROGMEM = "Quer L\0";
const char funktion5[] PROGMEM = "Quer R\0";
const char funktion6[] PROGMEM = "Lande \0";
const char funktion7[] PROGMEM = "Aux    \0";

PGM_P const FunktionTable[] PROGMEM = {funktion0, funktion1, funktion2, funktion3, funktion4, funktion5, funktion6, funktion7};

// Ausgang

const char ausgang0[] PROGMEM = "Imp";
const char ausgang1[] PROGMEM = "Kan";
const char ausgang2[] PROGMEM = "Dev";
const char ausgang3[] PROGMEM = "Funktion";
const char ausgang4[] PROGMEM = " ";
const char ausgang5[] PROGMEM = "Quer R\0";
const char ausgang6[] PROGMEM = "Lande \0";
const char ausgang7[] PROGMEM = "Aux    \0";

PGM_P const AusgangTable[] PROGMEM = {ausgang0, ausgang1, ausgang2, ausgang3, ausgang4, ausgang5, ausgang6, ausgang7};


// Zuteilung an device auf dem Sender
const char device0[] PROGMEM = "L-H\0"; // Pitch links horizontal
const char device1[] PROGMEM = "L-V\0"; // Pitch links vertikal
const char device2[] PROGMEM = "R-H\0";
const char device3[] PROGMEM = "R-V\0";
const char device4[] PROGMEM = "S-L\0"; // Schieber links
const char device5[] PROGMEM = "S-R\0"; // Schieber rechts
const char device6[] PROGMEM = "Sch\0"; // Schalter
const char device7[] PROGMEM = "Aux\0";

PGM_P const DispatchTable[] PROGMEM = {device0, device1, device2, device3, device4, device5, device6, device7};



#endif


