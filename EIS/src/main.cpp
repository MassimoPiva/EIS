#include <Arduino.h>
#include <avr/wdt.h>
#include <math.h>
#include <Button.h>
#include <Pushbutton.h>
#include <max6675.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LcdBarGraphX.h>
#include <EISLibrary.h>

// HANGAR EIS 1.1
// MODIFICHE
// Modificati valori di resistenze RB per i sensori di pressione da 44ohm a 470 ohm


// CREAZIONE OGGETTO LCD. UTILIZZO SINTASSI DELLA LIBRERIA STANDARD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// CREAZIONE OGGETTO BAR#
LcdBarGraphX bar1(&lcd, 8, 9, 0);
LcdBarGraphX bar2(&lcd, 8, 9, 1);
LcdBarGraphX bar3(&lcd, 8, 9, 2);
LcdBarGraphX bar4(&lcd, 8, 9, 3);
LcdBarGraphX bars[4] = {bar1, bar2, bar3, bar4};

/*##### COORDINATE SCHERMO 2x16 #####
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
  ___________________________________
  0 | S _ 1 : N N N O O S _ 2 : N N N |
  1 | O O O O O O O O O O O O O O O 0 |
  -----------------------------------
      * * * W A R N I N G ! * * *

  ######### COORDINATE SCHERMO 4x20 ##########
  VISUALIZZAZIONE STANDARD1
      0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9
  ___________________________________________
  0 | C H T _ 1 : N N N . . C H T _ 2 : N N N |
  1 | E G T _ 1 : N N N . . E G T _ 2 : N N N |
  2 | O I L _ P : N N N . . O I L _ T : N N N |
  3 | F L L _ P : N N N . . O A T _ _ : N N N |
  -------------------------------------------

  VISUALIZZAZIONE BARVIEW1
      0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9
  ___________________________________________
  0 | C H T _ 1 > N N N . | | | | | | | N N N |
  1 | E G T _ 1 > N N N . | | | | | | | N N N |
  2 | O I L _ P > N N N . | | | | | | | N N N |
  3 | O I L _ T > N N N . | | | | | | | N N N |
  -------------------------------------------

  VISUALIZZAZIONE BARVIEW2
      0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9
  ___________________________________________
  0 | C H T _ 2 > N N N . | | | | | | | N N N |
  1 | E G T _ 2 > N N N . | | | | | | | N N N |
  2 | F L L _ P > N N N . | | | | | | | N N N |
  3 | O A T _ T > N N N . | | | | | | | _ 5 0 |
  -------------------------------------------

*/

// GRUPPO DEFINIZIONE PINS
// PIN PULSANTI
#define pin_K1 2
#define pin_K2 3

// DEFINIZIONE OGGETTI PULSANTI UTILIZZO DELLA LIBRERIA BUTTON
Button buttonK1(pin_K1);
Button buttonK2(pin_K2);

// DEFINIZIONE OGGETTI PULSANTI UTILIZZO DELLA LIBRERIA PUSHBUTTON
// Pushbutton buttonK1(pin_K1);
// Pushbutton buttonK1(pin_K2);

// DEFINIZIONE PIN ALTOPARLANTE
#define pin_BUZ 38

// SENSORI PER INGRESSI ANALOGICI
#define pin_CHT1 A0
#define pin_CHT2 A1
#define pin_OAT A2
#define pin_OILP A3
#define pin_OILT A4
#define pin_FLLP A5

// EGT (MAX6675) / SENSORI DIGITALI
#define pin_EGT1_SCK 30
#define pin_EGT1_CS 32
#define pin_EGT1_SO 34
#define pin_EGT2_SCK 46
#define pin_EGT2_CS 48
#define pin_EGT2_SO 50

// IMPOSTAZIONE DI COSTANTI UTENTE
#define NUMBER_SENSOR 8
int frequenza_aggiornamento = 1000;
int contatore = 0;
long timer = millis();
long debounceDelay = 100;

// SENSORI VISUALIZZATI NELLE DIFFERENTI SCHERMATE A BARRE
int list_sensor1[4] = {0, 2, 4, 5};
int list_sensor2[4] = {1, 3, 6, 7};
// int list_sensor_checked[] = {0, 1, 2, 3, 4, 5, 6, 7};

// IMPOSTAZIONE INIZIALE VARIABILI PULSANTI E LORO GESTIONE
int valueK1 = LOW;
int lastValueK1 = LOW;
int actionK1 = 0;
int countK1 = 0;
long lastDebounceTimeK1 = 0;
boolean pushK1 = false;

int valueK2 = LOW;
int lastValueK2 = LOW;
int actionK2 = 0;
int countK2 = 0;
long lastDebounceTimeK2 = 0;
boolean pushK2 = false;

long millis_held;
boolean timeStart = false;

// PARAMETRI PER IL PASSAGGIO TRA LE VISUALIZZAZIONI
int switchK1 = 1;
boolean firstView1 = true;
boolean firstView2 = true;
boolean firstView3 = true;

// PARAMETRI FORMULA CALCOLO RESISTENZA
// (0.0016090854379756324, 0.0002613538714912094, -1.1734224839057563e-7} - Valori per sensore temperatura VDO P92
// (0.0009123601679945556, 0.00027559298078697655, -1.348192074016094e-7) - Valori per la sonda in rame
// (0.00008, 0.0421, -0.4068) - Valori sensore pressione VDO per Olio
// (0.000005, 0.0162, -0.1514) - Valori sensore di pressione, 3 bar, polinomio 2nd grado R2=0,99999
double param_NTC_0[3] = {0.0016090854379756324, 0.0002613538714912094, -1.1734224839057563e-7};   // CHT1
double param_NTC_1[3] = {0.0016090854379756324, 0.0002613538714912094, -1.1734224839057563e-7};   // CHT2
double param_NTC_4[3] = {0.00008, 0.0421, -0.4068};                                               // OILP
double param_NTC_5[3] = {0.0016160898596785772, 0.00025964706479783116, -1.0207064690299489e-7};  // OILT                                                            // OILT
double param_NTC_6[3] = {0.000005, 0.0162, -0.1514};                                              // FLLP
double param_NTC_7[3] = {0.0009123601679945556, 0.00027559298078697655, -1.348192074016094e-7};   // OAT


// IMPOSTAZIONE CONNESIONE MAX6675 CON PIN ARDUINO E IMPOSTAZIONE VARIABILE KTC
MAX6675 ktc_EGT1(pin_EGT1_SCK, pin_EGT1_CS, pin_EGT1_SO);
MAX6675 ktc_EGT2(pin_EGT2_SCK, pin_EGT2_CS, pin_EGT2_SO);

// STRUTTURA GESTIONE SENSORI
typedef struct {
  double value = 0;         // Tutti i sensori sono letti in double
  int coordinates[3];       // Coordinate schermo [riga, colonna testo, colonna valore]
  double limit_values[4];   // Valori limiti strumento
  int Rb;                   // Resistenza del partitore di tenzione per le sonde NTC
  String text;              // Testo fisso
  String type;              // Tipo valore [int, double]
  boolean present;          // Indica se il EIS sente il sensore
} EIS_Sensor;
EIS_Sensor sens[NUMBER_SENSOR];

// PARAMETRI LIMITI MOTORE, DA MODIFICARE IN FUNZIONE DEL MOTORE
  const double limit_values[NUMBER_SENSOR][4] = {
    {40, 60, 110, 130},        // CHT1
    {40, 60, 110, 130},        // CHT2
    {400, 700, 920, 950},      // EGT1
    {400, 700, 920, 950},      // EGT2
    {1.0, 2.2, 6.1, 8.0},      // OILP
    {40, 60, 110, 130},        // OILT
    {1.0, 2.5, 4.5, 5.5},      // FLLP
    { -20, 0, 30, 50}          // OAT
  };

  // **********************************************************************************************
  // *** FUNZIONI *** FUNZIONI *** FUNZIONI *** FUNZIONI *** FUNZIONI *** FUNZIONI *** FUNZIONI ***
  // **********************************************************************************************

  // VISUALIZZA A VIDEO QUANDO UN PARAMETRO E' IN ARCO GIALLO OPPURE NON PRESENTE
  String GestioneAttenzione(int n_sens, double value, double  Limit_Values[]) {

     if ((n_sens == 4) || (n_sens == 6)) {

         if (value < Limit_Values[0]) {
           return "LOW";
         }
         else if (value >= Limit_Values[0] && value < Limit_Values[1]) {
           return "WRN";
         }
         else if (value >= Limit_Values[1] && value < Limit_Values[2]) {
           return (String)value;
         }
         else if (value >= Limit_Values[2] && value < Limit_Values[3]) {
           return "HGH";
         }
         else if (value >= Limit_Values[3]) {
         return "!!!";
         }

      }
      else {

         if (value < Limit_Values[0]) {
           return "CLD";
         }
         else if (value >= Limit_Values[0] && value < Limit_Values[1]) {
           return "WRN";
         }
         else if (value >= Limit_Values[1] && value < Limit_Values[2]) {
           return (String)value;
         }
         else if (value >= Limit_Values[2] && value < Limit_Values[3]) {
           return "WRN";
         }
         else if (value >= Limit_Values[3]) {
         return "!!!";
         }
      }
  } //End GestioneAttenzione


// VISUALIZZAZIONE STANDARD1, 8 parametri visualizzati. Prima Pulisce lo schermo, riscrive le
// intestazioni, le scritte fisse dello schermo
void standard1ViewText() {
  lcd.clear();
  for (int i = 0; i < NUMBER_SENSOR; i++) {
    ScriviTXT(lcd, sens[i].coordinates[0], sens[i].coordinates[1], sens[i].text, "text");
  }
} //End standard1ViewText


// VISUALIZZAZIONE STANDARD1, SCRIVE I VALORI
void standard1ViewValue(int cont) {

  for (int i = 0; i < NUMBER_SENSOR; i++) {

       if (sens[i].present) {

          String temp = GestioneAttenzione(i, sens[i].value, sens[i].limit_values);

//        if ((temp == "!!!") && (cont%2==0)) {
//             lcd.clear();
//             Serial.print("ma soono qui?");
//             ScriviTXT(lcd, 0, 0, "********************", "text");
//             ScriviTXT(lcd, 1, 4, sens[i].text, "text");
//             ScriviTXT(lcd, 1, 12, (String)(sens[i].value), "text");
//             ScriviTXT(lcd, 2, 0, "***** WARNING! *****", "text");
//             ScriviTXT(lcd, 3, 0, "********************", "text");
//             delay(1000);
//             temp = "";
//             standard1ViewText();
//
////             tone (pinBuzz, map(S, 10, 130, 1400, 5000), 400);
//          }

          if (cont%7 == 0) {
              ScriviTXT(lcd, sens[i].coordinates[0], sens[i].coordinates[2], temp, sens[i].type);
              } else {
              ScriviTXT(lcd, sens[i].coordinates[0], sens[i].coordinates[2], String(sens[i].value), sens[i].type);
              }
           temp = "";
       } else {
          ScriviTXT(lcd, sens[i].coordinates[0], sens[i].coordinates[2], "---", sens[i].type);
       }
  }
 } //End standard1ViewValue


// VISUALIZZAZIONE A BARRE: VISUALIZZA 4 PARAMETRI INDICATI NEL LIST_SENSOR
void barView (int * list_sensor, int n) {
  for (int i = 0; i < n; i++) {
    int this_sens = list_sensor[i];
    if (sens[this_sens].present){
       ScriviTXT(lcd, i, 0, sens[this_sens].text, "text");
       ScriviTXT(lcd, i, 6, String(sens[this_sens].value), sens[this_sens].type);
       ScriviTXT(lcd, i, 17, String(sens[this_sens].limit_values[3]), sens[this_sens].type);
       bars[i].drawValue(sens[this_sens].value, sens[this_sens].limit_values[3]);
      } else {
       ScriviTXT(lcd, i, 0, sens[this_sens].text, "text");
       ScriviTXT(lcd, i, 6, "---", "text");
       ScriviTXT(lcd, i, 17, String(sens[this_sens].limit_values[3]), sens[this_sens].type);
      }//if
  }//for
}//barView


// VERIFICA PRESENZA SENSORI
void checkSens(){
  int wait_notok = 2000;
  int wait_ok =1500;
  Serial.println("Check Sensori in Esecuzione");

  lcd.clear();
  delay(500);
  ScriviTXT(lcd, 0, 0, "## Check  Sensori ##", "text");
  delay(1000);
  int riga = 2;

// Check CHT1, sensore [0]
  double CHT1 = analogRead(pin_CHT1); //Lettura pin A0
  if ((CHT1 > 1010.00) || (isnan(CHT1))) {
     ScriviTXT(lcd, riga, 0, "# CHT1 not present #", "text");
     sens[0].present = false;
     delay(wait_notok);
  } else {
  sens[0].present = true;
  sens[0].value = round (tempEval_NTC (sens[0].Rb, param_NTC_0, CHT1) - 273.16);
  ScriviTXT(lcd, riga, 0, "# CHT1 OK  present #", "text");
  delay(wait_ok);}


// Check CHT2, sensore [1]
  double CHT2 = analogRead(pin_CHT2);  //Lettura pin A1
  if ((CHT2 > 1010.00) || (isnan(CHT2))) {
     ScriviTXT(lcd, riga, 0, "# CHT2 not present #", "text");
     sens[1].present = false;
     delay(wait_notok);
  } else {
  sens[1].present = true;
  sens[1].value = round (tempEval_NTC (sens[1].Rb, param_NTC_1, CHT2) - 273.16);
  ScriviTXT(lcd, riga, 0, "# CHT2 OK  present #", "text");
  delay(wait_ok);}


// Check EGT1, sensore [2]
  double EGT1 = ktc_EGT1.readCelsius();
  if (isnan(EGT1)){
     ScriviTXT(lcd, riga, 0, "# EGT1 not present #", "text");
     sens[2].present = false;
     delay(wait_notok);
  } else {
  sens[2].present = true;
  sens[2].value = EGT1;
  ScriviTXT(lcd, riga, 0, "# EGT1 OK  present #", "text");
  delay(wait_ok);}


// Check EGT2, sensore [3]
  double EGT2 = ktc_EGT2.readCelsius();
  if (isnan(EGT2)){
     ScriviTXT(lcd, riga, 0, "# EGT2 not present #", "text");
     sens[3].present = false;
     delay(wait_notok);
  } else {
  sens[3].present = true;
  sens[3].value = EGT2;
  ScriviTXT(lcd, riga, 0, "# EGT2 OK  present #", "text");
  delay(wait_ok);}


// Check OILP, sensore [4]
  double OILP = analogRead(pin_OILP);
  if ((OILP > 1000.00) || (isnan(OILP))) {
     ScriviTXT(lcd, riga, 0, "# OILP not present #", "text");
     sens[4].present = false;
     delay(wait_notok);
  } else {
  sens[4].present = true;
  sens[4].value = (presEval_NTC (sens[4].Rb, param_NTC_4, OILP));
  ScriviTXT(lcd, riga, 0, "# OILP OK  present #", "text");
  delay(wait_ok);}


// Check OILT, sensore [5]
  double OILT = analogRead(pin_OILT);
  if ((OILT > 1010.00) || (isnan(OILT))) {
     ScriviTXT(lcd, riga, 0, "# OILT not present #", "text");
     sens[5].present = false;
     delay(wait_notok);
  } else {
  sens[5].present = true;
  sens[5].value = round (tempEval_NTC (sens[5].Rb, param_NTC_5, OILT) - 273.16);
  ScriviTXT(lcd, riga, 0, "# OILT OK  present #", "text");
  delay(wait_ok);}


// Check FLLP, sensore [6]
  double FLLP = analogRead(pin_FLLP);
  if ((FLLP > 1000.00) || (isnan(FLLP))) {
     ScriviTXT(lcd, riga, 0, "# FLLP not present #", "text");
     sens[6].present = false;
     delay(wait_notok);
  } else {
  sens[6].present = true;
  sens[6].value = (presEval_NTC (sens[6].Rb, param_NTC_6, FLLP))/14.503;  // conversione in psi
  ScriviTXT(lcd, riga, 0, "# FLLP OK  present #", "text");
  delay(wait_ok);}



// Check OAT, sensore [7]
  double OAT = analogRead(pin_OAT);
  if ((OAT > 1010.00) || (isnan(OAT))) {
     ScriviTXT(lcd, riga, 0, "# OAT  not present #", "text");
     sens[7].present = false;
     delay(wait_notok);
  } else {
  sens[7].present = true;
  sens[7].value = round (tempEval_NTC (sens[7].Rb, param_NTC_7, OAT) - 273.16);
  ScriviTXT(lcd, riga, 0, "# OAT  OK  present #", "text");
  delay(wait_ok);}


// Creazione vettore con lista sensori presenti e funzionanti
  int n_sens_ok = 0;
  int list_sens_checked[8];
  for (int i = 0; i < NUMBER_SENSOR; i++) {
      if (sens[i].present) {
        sens[i].value = 0.0;
        list_sens_checked[n_sens_ok] = i;
        n_sens_ok = n_sens_ok + 1;
      }
  }
} //End Check_Sens


// LEGGE I SENSORI PRESENTI CHE HANNO SUPERATO IL CHECK
void leggiSens() {

  if (sens[0].present){
  double CHT1 = analogRead(pin_CHT1); //Lettura pin A0
  sens[0].value = round (tempEval_NTC (sens[0].Rb, param_NTC_0, CHT1) - 273.16);}

  if (sens[1].present){
  double CHT2 = analogRead(pin_CHT2);  //Lettura pin A1
  sens[1].value = round (tempEval_NTC (sens[1].Rb, param_NTC_1, CHT2) - 273.16);}

  if (sens[2].present){sens[2].value = ktc_EGT1.readCelsius();}
  if (sens[3].present){sens[3].value = ktc_EGT2.readCelsius();}

  if (sens[4].present){
  double OILP = analogRead(pin_OILP);
  sens[4].value = (presEval_NTC (sens[4].Rb, param_NTC_4, OILP));}

  if (sens[5].present){
  double OILT = analogRead(pin_OILT);
  sens[5].value = round (tempEval_NTC (sens[5].Rb, param_NTC_5, OILT) - 273.16);
  Serial.print("OILT:");
  Serial.print(OILT);
  Serial.print("   OILT Value:");
  Serial.println(sens[5].value);
  }

  if (sens[6].present){
  double FLLP = analogRead(pin_FLLP);
  sens[6].value = (presEval_NTC (sens[6].Rb, param_NTC_6, FLLP))/14.503;} // conversione in psi [1 bar = 14,5038 psi]

  if (sens[7].present){
  double OAT = analogRead(pin_OAT);
  sens[7].value = round (tempEval_NTC (sens[7].Rb, param_NTC_7, OAT) - 273.16);

  }
} // End leggiSens

// SETUP PROGRAMMA, AVVIO LCD, PULSANTI, CARICAMENTO INFO SENSORE, CHECK SENSORI E PRIMA VISUALIZZAZIONE
void setup() {

// IMPOSTAZIONE SERIALE E MESSAGGIO AVVIO SCHEDA
  Serial.begin(9600);
  Serial.println("AVVIO SCHEDA!!!");


// IMPOSTAZIONE PIN INPUT PULSANTI
  pinMode (pin_K1, INPUT);
  pinMode (pin_K2, INPUT);

// INIZIALIZZAZIONE OGGETTI PULSANTI
  buttonK1.begin();
  Serial.println(buttonK1.pressed());
  buttonK2.begin();
  Serial.println(buttonK2.pressed());

// INPOSTAZIONE PIN OUTPUT ALTOPARLANTE
  pinMode (pin_BUZ, OUTPUT);

// INIZIALIZZAZIONE SCHERMO
  lcd.begin(20, 4);              // Inizializzo schermo 20 colonne x 4 righe
  lcd.backlight();               // Accensione retroilluminazione

  ScriviTXT(lcd, 0, 0, "####################", "text");
  delay(500);
  ScriviTXT(lcd, 1, 0, "# SCHEDA IN AVVIO! #", "text");
  delay(500);
  ScriviTXT(lcd, 2, 0, "#  Hangar EIS 1.0  #", "text");
  delay(500);
  ScriviTXT(lcd, 3, 0, "####################", "text");
  delay(1000);
  tone(pin_BUZ, 200, 50);
  lcd.clear();

// Disabilito la funzione "watchdog" per utilizzarla con la funzione reset
  wdt_disable();

//CARICO LA STRUTTURA EIS_SENSOR DI TUTTI I DATI COSTANTI
//Text to watch in LCD.
  sens[0].text = "CHT 1>";
  sens[1].text = "CHT 2>";
  sens[2].text = "EGT 1>" ;
  sens[3].text = "EGT 2>";
  sens[4].text = "OIL P>";
  sens[5].text = "OIL T>";
  sens[6].text = "FLL P>";
  sens[7].text = "OAT  >";

// Definisco se la variabile deve essere trattata come Int o Double
  sens[0].type = "int";
  sens[1].type = "int";
  sens[2].type = "int";
  sens[3].type = "int";
  sens[4].type = "double";
  sens[5].type = "int";
  sens[6].type = "double";
  sens[7].type = "int";

// Assegno le resitenze dei ripartitori tenzione lettura sensori analogici
  sens[0].Rb = 1000;     // CHT1
  sens[1].Rb = 999;     // CHT2
  sens[4].Rb = 470;     // OILP
  sens[5].Rb = 999;     // OILT
  sens[6].Rb = 470;     // FLLP
  sens[7].Rb = 999;     // OAT

// Carico la struttura dati con i valori limite dei sensori
  for (int i = 0; i < NUMBER_SENSOR; i++)
    for (int j = 0; j < 4; j++)
      sens[i].limit_values[j] = limit_values[i][j];

// Carico la struttura dati delle coordinate schermo per la visualizzazione dei testi e dei valori
  byte m1 = -1, m2 = 0, m3 = 6;
  for (int i = 0; i < NUMBER_SENSOR; i++) {
    if (i % 2 == 0) {
      m2 = 0;
      m3 = 6;
      m1++;
    } else {
      m2 = 11;
      m3 = 17;
    }
    sens[i].coordinates[0] = m1;     // riga
    sens[i].coordinates[1] = m2;     // col testo
    sens[i].coordinates[2] = m3;     // col valore
  }


// CHECK SENSORI ALL'ACCENSIONE
  checkSens();

// AVVIO VISUALIZAZIONE STANDARD 1
  standard1ViewText();

}

// PROGRAMMA PRINCIPALE
void loop() {

// Gestione delle 3 visualizzazioni, la pressione di K1 commuta tra una visualizzazione e la successiva
// Visualizzazione default è switchK1 = 1.

   switch (switchK1) {
        case 1:
             if (firstView1) {
                 Serial.println("sono dentro il case 1");
                 standard1ViewText();
                 firstView1 = false;
                 firstView2 = true;}
             do {
                leggiSens();
                contatore = contatore + 1;
                if (contatore == 9999) contatore = 0;
                if (buttonK1.pressed()) {
                    if (buttonK2.pressed()) Serial.println("sono dentro doppio");
                    checkSens();
                    standard1ViewText();
                   }
                if (buttonK2.pressed()) {
                    countK1 = countK1 + 1;
                    switchK1 = countK1%3;
                    if (switchK1 == 0) {switchK1 = 3;}
                }
                standard1ViewValue(contatore);
                delay(frequenza_aggiornamento);
              } while(switchK1 == 1); // Condizioni di uscita dal ciclo do: pressione del pulsante e switchK1++
             break;

        case 2:
             if (firstView2) {
                Serial.println("sono dentro il case 2");
                lcd.clear();
                firstView2 = false;
                firstView3 = true;}
             do {
                leggiSens();
                barView(list_sensor1, 4);
                if (buttonK2.pressed()) {
                    countK1 = countK1 + 1;
                    switchK1 =countK1%3;
                    if (switchK1 == 0) {switchK1 = 3;}
                }
                delay(frequenza_aggiornamento);
              } while(switchK1 == 2);
            break;

        case 3:
             if (firstView3) {
                Serial.println("sono dentro il case 3");
                lcd.clear();
                firstView3 = false;
                firstView1 = true;}
             do{
               leggiSens();
               barView(list_sensor2, 4);
               if (buttonK2.pressed()) {
                   countK1 = countK1 + 1;
                   switchK1 =countK1%3;
                   if (switchK1 == 0) {switchK1 = 3;}
               }
               delay(frequenza_aggiornamento);
             } while(switchK1 == 3);
           break;

        default:
           standard1ViewValue(contatore);
  } //End Switch

}
