/*--------------------------------------------------------------------
  This file is part of the tinyNeoPixel library, derived from
  Adafruit_NeoPixel.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  --------------------------------------------------------------------*/
// *INDENT-OFF* astyle hates this file
// *PAD-OFF* and destroys the lookup tables!

#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include <stdint.h>

#define N_LEDS 60
#define N_LED_BYTES (N_LEDS/8)

extern uint8_t active_color[3];
extern uint8_t active_flags[N_LED_BYTES];

void init_leds(void);
void update_leds(void);

#endif