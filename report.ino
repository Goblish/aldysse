/*
#include <Wire.h>
#include <SFE_ISL29125.h>
#include <math.h>

#define REDONE 0
#define YELLOW 1
#define GREEN 2
#define CYAN 3
#define BLUE 4 
#define PURPLE 5
#define REDTWO 6 

const double R_PERCENT_CORRECT = 12.00;
const double G_PERCENT_CORRECT = -13.00;
const double B_PERCENT_CORRECT = 2.00;

SFE_ISL29125 RGB_sensor;

void rectify(double *R, double *G, double *B) {
  if (fabs(*R - *G) <= 0.1) {
    *R = *R;
  }

  if (fabs(*R - *B) <= 0.1) {
    *B = *R;
  }

  if (fabs(*B - *G) <= 0.1) {
    *G = *B;
  }
}

double doubleMax(double r, double g, double b) {
  double max = r;

  if (max < g) {
    max = g;
  }
  
  if (max < b) {
    max = b;
  }
  
  return max;
}

double doubleMin(double r, double g, double b) {
  double min = r;

  if (min > g) {
    min = g;
  }
  
  if (min > b) {
    min = b;
  }
  
  return min;
}

double doubleMap(uint16_t val, float in_min, float in_max, float out_min, float out_max)
{
  double x = (double)val;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);
  
  if (RGB_sensor.init()) {
    Serial.println("Sensor init success");
    RGB_sensor.config(CFG1_MODE_RGB | CFG1_375LUX | CFG1_16BIT, CFG2_IR_OFFSET_OFF | CFG_DEFAULT, CFG3_RGB_CONV_TO_INT_DISABLE);
    Serial.println("# Data Report");
    Serial.println("");
  }
}

void loop() {
  uint16_t red = RGB_sensor.readRed();
  uint16_t green = RGB_sensor.readGreen();
  uint16_t blue = RGB_sensor.readBlue();

  double R = doubleMap(red, 0.0, 65535.0, 0.0, 1.0);
  double G = doubleMap(green, 0.0, 65535.0, 0.0, 1.0);
  double B = doubleMap(blue, 0.0, 65535.0, 0.0, 1.0);

  double total = R + G + B;

  double pRed = R / total * 100;
  double pGreen = G / total * 100;
  double pBlue = B / total * 100;

  Serial.println("");
  Serial.println(pRed, DEC);
  Serial.println(pGreen, DEC);
  Serial.println(pBlue, DEC);
  Serial.println("");

  double R2 = (pRed + R_PERCENT_CORRECT) * total / 100;
  double G2 = (pGreen + G_PERCENT_CORRECT) * total / 100;
  double B2 = (pBlue + B_PERCENT_CORRECT) * total / 100;
/*
double R3 = R;
double G3 = G;
double B3 = B;
*//*
double R3 = floor(R2 * 10.00) / 10.00;
double G3 = floor(G2 * 10.00) / 10.00;
double B3 = floor(B2 * 10.00) / 10.00;

rectify(&R3, &G3, &B3);

  Serial.println("---");
  Serial.println(R3, DEC);
  Serial.println(G3, DEC);
  Serial.println(B3, DEC);
  Serial.println("---");

/*
  int R = map(red, 0, 65535, 0, 255);
  int G = map(green, 0, 65535, 0, 255);
  int B = map(blue, 0, 65535, 0, 255);
*/
/*
  double R = doubleMap(red, 0.0, 65535.0, 0.0, 1.0);
  double G = doubleMap(green, 0.0, 65535.0, 0.0, 1.0);
  double B = doubleMap(blue, 0.0, 65535.0, 0.0, 1.0);
*/
/*
  Serial.println("");
  Serial.println(R, DEC);
  Serial.println(G, DEC);
  Serial.println(B, DEC);
  Serial.println("");
*//*

  double max = doubleMax(R3, G3, B3);
  double min = doubleMin(R3, G3, B3);
  double C = max - min;
  double H = 7;

  if (max == min) {
    Serial.println("BLANK");
  }
  
  if (max == R3) {
    H = fmod((G3 - B3) / C, 6.00);
  }
  
  if (max == G3) {
    H = (B3 - R3) / C + 2.00;
  }
  
  if (max == B3) {
    H = (R3 - G3) / C + 4.00;
  }

  Serial.println("H --- H");
  Serial.println(H * 60, DEC);
  Serial.println("---");

  if (0.00 <= H && H < 1.00) {
    Serial.println("RED ONE");
  }
  if (1.00 <= H && H  < 2.00) {
    Serial.println("YELLOW");
  }
  if (2.00 <= H && H  < 3.00) {
    Serial.println("GREEN");
  }
  if (3.00 <= H && H  < 4.00) {
    Serial.println("CYAN");
  }
  if (4.00 <= H && H  < 5.00) {
    Serial.println("BLUE");
  }
  if (5.00 <= H && H  < 6.00) {
    Serial.println("PURPLE");
  }

  delay(3500);
}
*/

