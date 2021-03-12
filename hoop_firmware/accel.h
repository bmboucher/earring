#ifndef ACCEL_H
#define ACCEL_H

#include <stdint.h>

extern uint16_t down_angle;
extern uint8_t tilt_count;

void init_accel(void);
void update_accel(void);

#endif