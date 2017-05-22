/* Code file for EIS Library */
#include "Arduino.h"            // Basic per utilizzare le librerie
#include <avr/wdt.h>            // Necessaria per il WDT/Reset
#include <LiquidCrystal_I2C.h>  // Using version 1.2.1


// FUNZIONE RESET
void reset() {
  // ScriviTXT(lcd, 0, 0, "####################", "text");
  // delay(500);
  // ScriviTXT(lcd, 1, 0, "#  RESET IN CORSO  #", "text");
  // delay(500);
  // ScriviTXT(lcd, 2, 0, "#  Hangar EIS 1.0  #", "text");
  // delay(500);
  // ScriviTXT(lcd, 3, 0, "####################", "text");
  // delay(1000);
  // lcd.clear();
  delay(1000);
  wdt_enable(WDTO_4S);
  while (1);
}


// ScriviTXT riceve in ingresso la posizione del cursore (RIGA E COLONNA) ed il testo da visualizzare nel display.
// Se la stringa (numeri) ha meno di 3 caratteri, vengono aggiunti 1 o 2 spazi
void ScriviTXT(LiquidCrystal_I2C& lcd, int rig, int col, String testo, String tipo){
     lcd.setCursor(col,rig);

     if (tipo == "text") {
      lcd.print(testo);
      return;}

     if (tipo == "int") {
        if (testo.length() == 4) {testo = "  " + testo.substring(0,1);}
        if (testo.length() == 5) {testo = " " + testo.substring(0,2);}
        if (testo.length() == 6) {testo = testo.substring(0,3);}
        lcd.print(testo);
      return;}

     if (tipo == "double") {
        if (testo.length() == 4) {testo = testo.substring(0,3);}
        if (testo.length() == 5) {testo = " " + testo.substring(0,2);}
        lcd.print(testo);
     return;}
   return;
}


// Funzione per il calcolo della temperatura con sonda PT100 e funzione map personalizzata sulla
// tabella delle temperature in[]
float MultiMap(float val, float* _in, uint8_t size) {
  if (val < _in[0] ) return -999;         // se il valore di resistenza letto dalla sonda è fuori dal range
  if (val > _in[size-1] ) return 999;
  uint8_t pos = 0;                        // cerca la posizione nel vettore delle resistenze il valore più vicino
  while(val > _in[pos]) pos++;
  if (val == _in[pos]) return pos;        // gestione del raro caso che il valore letto corrisponde ad un valore preciso del vettore
  float r1 = _in[pos-1];                  // calcolo della del valore come interpolazione
  float r2 = _in[pos];
  int c1 = pos-1;
  int c2 = pos;
  return (c1 + (val - r1) / (r2-r1) * (c2-c1));
}


// FUNZIONE PER IL CALCOLO DELLA TEMPERATURA IN °C CON ELABORAZIONE NTC CON FORMULA DI Steinhart-Hart
double tempEval_NTC (int R, double param[], double aRead) {
  double Vout = aRead * (5.0 / 1023.0);
  double R2 = R * 1/(5.0/Vout - 1);
  double LOG = log(R2);
  return 1/( param[0] + param[1] * LOG + param[2] * LOG * LOG * LOG);
}

// FUNZIONE PER IL CALCOLO DELLA PRESSIONE RESTITUISCE BAR
double presEval_NTC (int R, double param[], double aRead) {
  double Vout = aRead * (5.0 / 1023.0);
  double R2 = R * 1/(5.0/Vout - 1);
  return ((param[0] * R2 * R2 + param[1] * R2 + param[2])-0.4);
}

/* Stesura del codice delle funzioni */
