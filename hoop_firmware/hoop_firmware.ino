#include <Arduino.h>

#include "led_driver.h"
#include "accel.h"
#include "physics.h"

void setup() {
  init_leds();
  init_accel();
  init_physics();
}

void loop() {
  update_accel();
  clear_led_data();
  if (!(tilt_count & 0x80)) {
    update_physics();
  }
  update_leds();
}
