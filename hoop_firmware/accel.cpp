#include <stdint.h>
#include <Wire.h>
#include "atan2.h"

// MXC6255XU 2-axis accelerometer, communicates over I2C
#define ACCEL_I2C_ADDR 0x15

// Write-only register to configure shake interrupt
#define DETECTION  0x04
// Shake threshold = 1.5g
#define SHTH_1_5G  0x20
// Shake events time window = 160 ms
#define SHC_160_MS 0x04
// Orientation hysteresis period = 640 ms
#define ORC_640_MS 0x02

// Initialize I2C
void init_accel() {
    Wire.begin();

    Wire.beginTransmission(ACCEL_I2C_ADDR);
    Wire.write(DETECTION);
    Wire.write(SHTH_1_5G | SHC_160_MS | ORC_640_MS);
    Wire.endTransmission();
}

// ===== GLOBAL VARIABLES =====
// Moving average accelerations in X and Y
static int16_t accel_x;
static int16_t accel_y;
// Calculated location of "down" using atan2(y/x)
uint16_t down_angle;
// Counts up observations of the tilt flag in the status register
uint8_t tilt_count;
// Number of simultaneous events required to change tilt state
#define TILT_COUNT_THRESH 10

// Averages a single 8-bit value into a running 16-bit average (with scaling)
void update_avg(int8_t reading, int16_t* avg) {
    int16_t scaled = 3 * (*avg / 4);
    int16_t addl = ((int16_t)reading) * 64; // Scale-up 8 bits, divide by 4
    *avg = scaled + addl;
}

void update_accel(void) {
  // Request a read
  Wire.beginTransmission(ACCEL_I2C_ADDR);
  Wire.write(0x00); // Register 0 is x-axis acceleration
  Wire.endTransmission();
  
  // Read the X, Y and STATUS bytes
  Wire.requestFrom(ACCEL_I2C_ADDR, 3);
  int8_t reading_x = Wire.read();
  int8_t reading_y = Wire.read();
  uint8_t status = Wire.read();

  // Check the tilt flag (bit 4 in status byte)
  if (status & 0x10) {
    // If we have been off-tilt for a while, tilt_count will be 0 and we count
    // tilt events up to the threshold.
    if (tilt_count < TILT_COUNT_THRESH) {
      tilt_count++;
    } else {
      tilt_count = 0xFF; // Flip the MSB (i.e. move to "fully tilted")
    }
  } else {
    if (tilt_count == 0xFF) {
      // When we first come off tilt, leave the MSB set
      tilt_count = 0x80;
    } else if (tilt_count & 0x80) {
      // When the MSB is set, we count the number of off-tilt events
      tilt_count++;
      if ((tilt_count & 0x7F) >= TILT_COUNT_THRESH) {
        tilt_count = 0; // Clear the MSB (i.e. move to "fully not tilted")
      }
    }

    if (tilt_count == 0) {
      // Update gravity angle
      update_avg(reading_x, &accel_x);
      update_avg(reading_y, &accel_y);
      down_angle = atan2(accel_x, accel_y);
    }
  }
}