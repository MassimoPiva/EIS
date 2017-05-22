/* Header file for EIS Library */

#ifndef EISLIBRARY_H

#define EISLIBRARY_H

void reset();
void ScriviTXT(LiquidCrystal_I2C& lcd, int rig, int col, String testo, String tipo);
float MultiMap(float val, float* _in, uint8_t size);
double tempEval_NTC (int R, double param[], double aRead);
double presEval_NTC (int R, double param[], double aRead);

#endif
