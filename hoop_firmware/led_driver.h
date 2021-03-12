#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include <stdint.h>

#define N_LEDS 60
#define N_LED_BYTES (N_LEDS/8)

extern uint8_t active_color[3];
extern uint8_t active_flags[N_LED_BYTES];

void init_leds(void);
void clear_led_data(void);
void set_led_active(uint8_t n);
void update_leds(void);

#endif