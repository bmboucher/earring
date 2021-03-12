#include <Arduino.h>

#include <avr/sleep.h>

#include "led_driver.h"
#include "accel.h"
#include "physics.h"
#include "pins.h"

#define MAX_SLEEP_COUNT 1000
uint16_t sleep_count;
void setup() {
  init_leds();
  init_accel();
  init_physics();

  sleep_count = 0;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}

// Wake-up interrupt
ISR(PORTA_PORT) {
  PORTA.PIN3CTRL = 0x00; // Disable wake-up interrupt
  setup();
}

void go_to_sleep() {
  PORTA.DIR = 0x00;      // Set all pins to input
  PORTA.PIN3CTRL = 0x01; // Enable wake-up interrupt
  sleep_cpu();
}

void loop() {
  update_accel();
  clear_led_data();
  if (!(tilt_count & 0x80)) {
    update_physics();
  }
  update_leds();

  if (tilt_count == 0xFF) {
    sleep_count++;
  } else {
    sleep_count = 0;
  }

  if (sleep_count >= MAX_SLEEP_COUNT) {
    go_to_sleep();
  }
}
