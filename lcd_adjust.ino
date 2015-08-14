#define COLOR_BASE 0xFFFFFF

boolean lcd_is_white(unsigned int RGB_raw[3]) {
  double RGB_percent[3];
  
  RGB_calc_percent(RGB_raw, RGB_percent);
  
  // maybe with distance ? écart ?
  if (RGB_percent[RED] <= 50.00 && 
      RGB_percent[GREEN] <= 50.00 && 
      RGB_percent[BLUE] <= 50.00) {
    return true;
  }
  
  return false;
}

void lcd_adjust_factor(unsigned int RGB[3], double RGB_adjust_factor[3]) {
  unsigned int RGB_base[3];
  double RGB_base_percent[3];
  double RGB_percent[3];

  for (byte i = 0; i < 3; ++i) {
    RGB_base[i] = (unsigned int)byte(COLOR_BASE >> (16 - i * 8));
  }

  RGB_calc_percent(RGB_base, RGB_base_percent);
  RGB_calc_percent(RGB, RGB_percent);

  for (byte i = 0; i < 3; ++i) {
    RGB_adjust_factor[i] = RGB_base_percent[i] - RGB_percent[i];
  }
}

