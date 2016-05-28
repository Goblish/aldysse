#define COLOR_BASE 0xFFFFFF

void lcd_is_white(uint16_t *R, uint16_t *G, uint16_t *B) {

}

void lcd_adjust(uint16_t *R, uint16_t *G, uint16_t *B) {
  byte baseR = COLOR_BASE >> 16;
  byte baseG = COLOR_BASE >> 8;
  byte baseB = COLOR_BASE;

  Serial.println(baseR, DEC);
  Serial.println(baseG, DEC);
  Serial.println(baseB, DEC);
}

void lcd_adjust_reset() {

}

