#ifndef ATAN2_H
#define ATAN2_H

#include <stdint.h>

/**
 * 16-bit fixed point four-quadrant arctangent. Given some Cartesian vector
 * (x, y), find the angle subtended by the vector and the positive x-axis.
 *
 * The value returned is in units of 1/65536ths of one turn. This allows the use
 * of the full 16-bit unsigned range to represent a turn. e.g. 0x0000 is 0
 * radians, 0x8000 is pi radians, and 0xFFFF is (65535 / 32768) * pi radians.
 *
 * Because the magnitude of the input vector does not change the angle it
 * represents, the inputs can be in any signed 16-bit fixed-point format.
 *
 * @param y y-coordinate in signed 16-bit
 * @param x x-coordinate in signed 16-bit
 * @return angle in (val / 32768) * pi radian increments from 0x0000 to 0xFFFF
 */
uint16_t atan2(const int16_t x, const int16_t y);

#endif