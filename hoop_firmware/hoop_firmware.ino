#include <Arduino.h>
#include <Wire.h>
#include "led_driver.h"
#include <math.h>

void setup() {
  init_leds();
  Wire.begin();
}

const float LAMBDA = 0.7;
float accel_x = 0;
float accel_y = 0;
float angle = 0;

static void avg_accel(byte reading, float* avg) {
  *avg = (1 - LAMBDA)*((float)reading) + LAMBDA*(*avg);
}

#define ACCEL_I2C_ADDR 0x15
void read_accel() {
  Wire.beginTransmission(ACCEL_I2C_ADDR);
  Wire.write(0x00); // Register 0 is x-axis acceleration
  Wire.endTransmission();
  
  Wire.requestFrom(ACCEL_I2C_ADDR, 3);
  avg_accel(Wire.read(), &accel_x);
  avg_accel(Wire.read(), &accel_y);
  angle = atan2f(accel_y, accel_x);
  byte status = Wire.read();
}

void loop() {
  read_accel();
  update_leds();
}
