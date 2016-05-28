/*
#include <Wire.h>
#include <SFE_ISL29125.h>

SFE_ISL29125 RGB_sensor;

void setup() {
  Serial.begin(9600);
  
  if (RGB_sensor.init()) {
    Serial.println("Sensor init success");
    RGB_sensor.config(CFG1_MODE_RGB | CFG1_375LUX | CFG1_16BIT, CFG2_IR_OFFSET_OFF | CFG_DEFAULT, CFG3_RGB_CONV_TO_INT_DISABLE);
    Serial.println("# LCD ADJUST");
    Serial.println("");
  }
}

void loop() {
  unsigned int red = RGB_sensor.readRed();
  unsigned int green = RGB_sensor.readGreen();
  unsigned int blue = RGB_sensor.readBlue();

  unsigned int R = map(red, 0, 65535, 0, 255);
  unsigned int G = map(green, 0, 65535, 0, 255);
  unsigned int B = map(blue, 0, 65535, 0, 255);

  Serial.println("---- Mapped values");
  Serial.println(R, DEC);
  Serial.println(G, DEC);
  Serial.println(B, DEC);
  Serial.println("----");
  Serial.println("");

  unsigned int total = R + G + B;
  
  unsigned int pRed = R * 100 / total;
  unsigned int pGreen = G * 100 / total;
  unsigned int pBlue = B * 100 / total;

  Serial.println("---- Percent");
  Serial.println(pRed, DEC);
  Serial.println(pGreen, DEC);
  Serial.println(pBlue, DEC);
  Serial.println("----");
  Serial.println("");

  int cRed = 33 - pRed;
  int cGreen = 33 - pGreen;
  int cBlue = 33 - pBlue;

  Serial.println("---- Correct");
  Serial.println(cRed, DEC);
  Serial.println(cGreen, DEC);
  Serial.println(cBlue, DEC);
  Serial.println("----");
  Serial.println("");

  int cR = (pRed + cRed) * total / 100;
  int cG = (pGreen + cGreen) * total / 100;
  int cB = (pBlue + cBlue) * total / 100;

  Serial.println("---- Mapped values corrected");
  Serial.println(cR, DEC);
  Serial.println(cG, DEC);
  Serial.println(cB, DEC);
  Serial.println("----");
  Serial.println("");

  delay(5000);
}
*/
