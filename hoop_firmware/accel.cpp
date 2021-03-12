#include <stdint.h>
#include <Wire.h>

void init_accel() {
    Wire.begin();
}

// Use the fixed-point approximation
//       atan(x) ~= x(0.25 + 0.273/pi - 0.273/pi * |x|)
//                = x(             K2 +    K1 * (-|x|))
#define K1 2847
//   = 0.273/pi ~= 0.087 mapped from [-1,1) to [-32768,32768)
#define K2 (K1+8192)
//   = 0.25 + 0.273/pi ~= 0.337
static int16_t s16_nabs(int16_t x) {
    return x < 0 ? x : -x;
}
static int16_t q15_div(int16_t num, int16_t den) {
    int32_t result = (((int32_t)num) << 16) / den;
    return *((int16_t*)&result);
}
static int16_t q15_mul(int16_t x, int16_t y) {
    int32_t result = (int32_t)x * (int32_t)y;
    return *((int16_t*)&result);
}

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
static uint16_t atan2(const int16_t x, const int16_t y) {
    if (x == y) { // x/y or y/x would return -1 since 1 isn't representable
        if (y > 0) { // 1/8
            return 8192;
        } else if (y < 0) { // 5/8
            return 40960;
        } else { // x = y = 0
            return 0;
        }
    }
    const int16_t nabs_y = s16_nabs(y), nabs_x = s16_nabs(x);
    if (nabs_x < nabs_y) { // octants 1, 4, 5, 8
        const int16_t y_over_x = q15_div(y, x);
        const int16_t correction = q15_mul(K1, s16_nabs(y_over_x));
        const int16_t unrotated = q15_mul(K2 + correction, y_over_x);
        if (x < 0) { // octants 1, 8
            return unrotated;
        } else { // octants 4, 5
            return 32768 + unrotated;
        }
    } else { // octants 2, 3, 6, 7
        const int16_t x_over_y = q15_div(x, y);
        const int16_t correction = q15_mul(K1, s16_nabs(x_over_y));
        const int16_t unrotated = q15_mul((K2 + correction), x_over_y);
        if (y > 0) { // octants 2, 3
            return 16384 - unrotated;
        } else { // octants 6, 7
            return 49152 - unrotated;
        }
    }
}

static int16_t accel_x;
static int16_t accel_y;
uint16_t down_angle;
uint8_t tilt_count;

#define MAX_TILT_COUNT 10

void update_avg(int8_t reading, int16_t* avg) {
    int16_t scaled = 3 * (*avg / 4);
    int16_t addl = ((int16_t)reading) * 64; // Scale-up 8 bits, divide by 4
    *avg = scaled + addl;
}

#define ACCEL_I2C_ADDR 0x15
void update_accel(void) {
  Wire.beginTransmission(ACCEL_I2C_ADDR);
  Wire.write(0x00); // Register 0 is x-axis acceleration
  Wire.endTransmission();
  
  Wire.requestFrom(ACCEL_I2C_ADDR, 3);
  int8_t reading_x = Wire.read();
  int8_t reading_y = Wire.read();
  uint8_t status = Wire.read();
  // Status bit 4 is a tilt indicator
  if (status & 0x10) {
    if (tilt_count < MAX_TILT_COUNT) {
      tilt_count++;
    } else {
      tilt_count = 0xFF;
    }
  } else {
    tilt_count = 0;
    update_avg(reading_x, &accel_x);
    update_avg(reading_y, &accel_y);
    down_angle = atan2(accel_x, accel_y);
  }
}