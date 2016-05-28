
double map_double(uint16_t val, float in_min, float in_max, float out_min, float out_max)
{
  double x = (double)val;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double max_double(double a, double b, double c) {
  double max = a;

  if (max < b) {
    max = b;
  }
  
  if (max < c) {
    max = c;
  }
  
  return max;
}

double min_double(double a, double b, double c) {
  double min = a;

  if (min > b) {
    min = b;
  }
  
  if (min > c) {
    min = c;
  }
  
  return min;
}

