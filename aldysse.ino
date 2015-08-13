/*
#include <Wire.h>
#include <SFE_ISL29125.h>

#define RED 0x01
#define GREEN 0x02
#define BLUE 0x03
#define BLANK 0x04

SFE_ISL29125 RGB_sensor;

unsigned int i = 0;
uint8_t lastdata = 0x00;
unsigned int nbRead = 0;

void setup() {
  Serial.begin(9600);
  
  if (RGB_sensor.init()) {
    Serial.println("Sensor init success");
    RGB_sensor.config(CFG1_MODE_RGB | CFG1_375LUX | CFG1_12BIT, CFG2_IR_OFFSET_OFF | CFG_DEFAULT, CFG3_RGB_CONV_TO_INT_ENABLE);
    attachInterrupt(0, incoming, FALLING);
  }
}

void incoming() {
  i++;
}

void loop() {
  static unsigned long ms = millis();
  static unsigned int lasti = 0;
  double red = 0;
  double green = 0;
  double blue = 0;
  double total = 0;
  double pRed = 0;
  double pGreen = 0;
  double pBlue = 0;
  uint8_t data = BLANK;
  boolean canPrint;

  if (i > lasti) {
    red = RGB_sensor.readRed();
    green = RGB_sensor.readGreen();
    blue = RGB_sensor.readBlue();
    total = red + green + blue;
 
    pRed = red / total * 100;
    pGreen = green / total * 100;
    pBlue = blue / total * 100;

    if (pRed > 50.00) {
      data = RED;
    } else if (pGreen > 50.00) {
      data = GREEN;
    } else if (pBlue > 50.00) {
      data = BLUE;
    } else {
      data = BLANK;
    }

    if (lastdata == data) {
      nbRead++;
    } else {
      nbRead = 0;
    }

    if (nbRead >= 4) {
      //if (data != BLANK && canPrint == true) {
          switch (data) {
            case RED:
              Serial.println("0");
              break;
            case GREEN:
              Serial.println("1");
              break;
            case BLUE:
              Serial.println("2");
              break;
            case BLANK:
              Serial.println("BLANK");
              break;
          }
          canPrint = false;
      //} else if (data == BLANK) {
      //  canPrint = true;
      //}
      //Serial.println(millis() - ms);
      ms = millis();
      nbRead = 0;
    }

    lastdata = data;
 
    lasti = i;
    RGB_sensor.readStatus();
  }
}
*/

