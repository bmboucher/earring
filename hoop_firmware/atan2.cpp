#include "atan2.h"

// ===== ATAN2 FIXED-POINT APPROXIMATION =====
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
