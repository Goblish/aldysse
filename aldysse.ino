#include <Wire.h>
#include <SFE_ISL29125.h>

#define RED 0
#define GREEN 1
#define BLUE 2

SFE_ISL29125 RGB_sensor;

const double R_PERCENT_CORRECT = 12.00;
const double G_PERCENT_CORRECT = -13.00;
const double B_PERCENT_CORRECT = 2.00;

boolean incoming_read = false;

void incoming_raw_value() {
  incoming_read = true;
}

void setup() {
  Serial.begin(9600);
  
  if (RGB_sensor.init()) {
    Serial.println("Sensor init success");
    RGB_sensor.config(CFG1_MODE_RGB | CFG1_375LUX | CFG1_12BIT, CFG2_IR_OFFSET_OFF | CFG_DEFAULT, CFG3_RGB_CONV_TO_INT_ENABLE);
    attachInterrupt(0, incoming_raw_value, FALLING);
  }
}

void loop() {
  unsigned int RGB_raw[3];
  static uint8_t adjust_factorRGB[3];
  unsigned int RGB[3];
  
  if (incoming_read) {
    RGB_raw[RED] = RGB_sensor.readRed();
    RGB_raw[GREEN] = RGB_sensor.readGreen();
    RGB_raw[BLUE] = RGB_sensor.readBlue();

    Serial.print("RED: ");
    Serial.println(RGB_raw[RED]);
    Serial.print("GREEN: ");
    Serial.println(RGB_raw[GREEN]);
    Serial.print("BLUE: ");
    Serial.println(RGB_raw[BLUE]);
 
    incoming_read = false;
  }

  //lcd_adjust(0, 0, 0);

  RGB_raw[RED] = RGB_sensor.readRed();
  RGB_raw[GREEN] = RGB_sensor.readGreen();
  RGB_raw[BLUE] = RGB_sensor.readBlue();
/*
  double R = map_double(R_sensor, 0.0, 65535.0, 0.0, 1.0);
  double G = map_double(G_sensor, 0.0, 65535.0, 0.0, 1.0);
  double B = map_double(B_sensor, 0.0, 65535.0, 0.0, 1.0);

  double total = R + G + B;

  double pRed = R / total * 100;
  double pGreen = G / total * 100;
  double pBlue = B / total * 100;

  R = (pRed + R_PERCENT_CORRECT) * total / 100;
  G = (pGreen + G_PERCENT_CORRECT) * total / 100;
  B = (pBlue + B_PERCENT_CORRECT) * total / 100;

  double I = (R + G + B) / 3.00;
  double M = max_double(R, G, B);
  double m = min_double(R, G, B);
  double L = (M + m) / 2.00;
  double C = M - m;
  double H;

  double alpha = R - (G + B) / 2.00;
  double beta = sqrt(3.00) / 2.00 * (G - B);
  double H2 = atan2(beta, alpha);
  H2 = (H2 * 60.00) + 360;


  if (M == R) {
    H = fmod((G - B) / C, 6.00);
  }
  
  if (M == G) {
    H = (B - R) / C + 2.00;
  }
  
  if (M == B) {
    H = (R - G) / C + 4.00;
  }
  
  H = H * 60.00;

  Serial.println("----");  
  Serial.println(R);
  Serial.println(G);
  Serial.println(B);
  Serial.print("I : ");
  Serial.println(I);
  Serial.print("V : ");
  Serial.println(M);
  Serial.print("L : ");
  Serial.println(L);
  Serial.print("H : ");
  Serial.println(H);
  Serial.print("H2 : ");
  Serial.println(H2);
  Serial.println("----");

  delay(5000);
*/
}

