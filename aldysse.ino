#include <Wire.h>
#include <SFE_ISL29125.h>

#define RED 0
#define GREEN 1
#define BLUE 2

#define RGB_MAP_FROM_MIN 0.0
#define RGB_MAP_FROM_MAX 4095.00
#define RGB_MAP_TO_MIN 0.0
#define RGB_MAP_TO_MAX 1.0

#define WHITE_CHECK_MIN 100

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
    RGB_sensor.config(CFG1_MODE_RGB | CFG1_375LUX | CFG1_12BIT, CFG2_IR_OFFSET_OFF, CFG3_RGB_CONV_TO_INT_ENABLE);
    attachInterrupt(0, incoming_raw_value, FALLING);
  }
}

void loop() {
  unsigned int RGB_raw[3];
  double RGB_map[3];
  static unsigned int is_white_count = 0;
  static double RGB_adjust_factor[3];
  double RGB[3];
  double M;
  double m;
  double C;
  double H;

  if (incoming_read) {
    incoming_read = false;
 
    RGB_raw[RED] = RGB_sensor.readRed();
    RGB_raw[GREEN] = RGB_sensor.readGreen();
    RGB_raw[BLUE] = RGB_sensor.readBlue();

    for (byte i = 0; i < 3; ++i) {
      RGB_map[i] = map_double(RGB_raw[i], RGB_MAP_FROM_MIN, RGB_MAP_FROM_MAX, RGB_MAP_TO_MIN, RGB_MAP_TO_MAX);
    }

    if (RGB_is_same_that_last(RGB_map) && lcd_is_white(RGB_raw)) {
      is_white_count++;
    } else {
      is_white_count = 0;
    }

    if (is_white_count >= WHITE_CHECK_MIN) {
      lcd_adjust_factor(RGB_map, RGB_adjust_factor);
      //lcd_adjust_factor(RGB_raw, RGB_adjust_factor);
      is_white_count = 0;
    } else {
      for (byte i = 0; i < 3; ++i) {
        Serial.println(RGB_adjust_factor[i]);
        RGB[i] = RGB_map[i] + (RGB_map[i] * (RGB_adjust_factor[i] / 100.00));
        Serial.println(RGB[i]);
      }

      H = 0;
      M = max_double(RGB[RED], RGB[GREEN], RGB[BLUE]);
      m = min_double(RGB[RED], RGB[GREEN], RGB[BLUE]);
      C = M - m;

      if (M == RGB[RED]) {
        H = fmod((RGB[GREEN] - RGB[BLUE]) / C, 6.00);
      }

      if (M == RGB[GREEN]) {
        H = (RGB[BLUE] - RGB[RED]) / C + 2.00;
      }

      if (M == RGB[BLUE]) {
        H = (RGB[RED] - RGB[GREEN]) / C + 4.00;
      }

      //H = H * 60.00;
      /*
      Serial.println(H);

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
      }*/

      delay(2000);
    }

/*
    Serial.print("RED: ");
    Serial.println(RGB_raw[RED]);
    Serial.print("GREEN: ");
    Serial.println(RGB_raw[GREEN]);
    Serial.print("BLUE: ");
    Serial.println(RGB_raw[BLUE]);
*/
/*
    double R = map_double(RGB_raw[RED], 0.0, 4096.0, 0.0, 1.0);
    double G = map_double(RGB_raw[GREEN], 0.0, 4096.0, 0.0, 1.0);
    double B = map_double(RGB_raw[BLUE], 0.0, 4096.0, 0.0, 1.0);
  
    double M = max_double(R, G, B);
    double m = min_double(R, G, B);
    double C = M - m;
    
    if (C < 0.228) {
      Serial.println(C);
      Serial.println("WHITE");
    } else {
      Serial.println(C);
      Serial.println("OTHER");
    }
*/
    incoming_read = false;
    RGB_sensor.readStatus();
  }

  //lcd_adjust(0, 0, 0);

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

