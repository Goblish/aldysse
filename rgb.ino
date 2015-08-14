#include <math.h>

boolean RGB_is_same_that_last(double RGB[3]) {
  double RGB_round[3];
  static double RGB_last_value[3];
  byte same_count;

  same_count = 0;
  for (byte i = 0; i < 3; ++i) {
    RGB_round[i] = round(RGB[i] * 10.00) / 10.00;

    if (RGB_last_value[i] && (RGB_round[i] - RGB_last_value[i] <= 0.1)) {
      ++same_count;
    }

    RGB_last_value[i] = RGB_round[i];
  }

  if (same_count == 3) {
    return true;
  }
  
  return false;
}

void RGB_calc_percent(unsigned int RGB[3], double RGB_percent[3]) {
  double total;

  total = RGB[RED] + RGB[BLUE] + RGB[GREEN];
  for (byte i = 0; i < 3; ++i) {
    RGB_percent[i] = (double)RGB[i] / total * 100.00;
  }
}

void RGB_calc_percent_double(double RGB[3], double RGB_percent[3]) {
  double total;

  total = RGB[RED] + RGB[BLUE] + RGB[GREEN];
  for (byte i = 0; i < 3; ++i) {
    RGB_percent[i] = RGB[i] / total * 100.00;
  }
}

void RGB_to_HSV() {
}

