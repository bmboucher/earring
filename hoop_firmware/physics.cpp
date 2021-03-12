#include "physics.h"
#include "accel.h"
#include "led_driver.h"

#include <Arduino.h>

#define N_PTS 5
#define INIT_POS (65536 / 5)
uint16_t position[N_PTS];
int16_t velocity[N_PTS];

void init_physics() {
    memset(velocity, 0, sizeof(uint16_t) * N_PTS);
    for (uint8_t i = 0; i < N_PTS; i++) {
        position[i] = i * INIT_POS;
    }
}

static void set_led_pt(uint16_t angle) {
    uint16_t scaled = angle >> 6; // Scale [0,1) to [0,1/64) to avoid overflow
    uint16_t segment = scaled * N_PTS; // Multiply by 60 - range is [0,60/64)
    // Round to nearest 1/64 by adding 1/128 and taking 6 MSB 
    uint8_t rounded = (uint8_t)((segment + 512) >> 10);
    while (rounded >= N_LEDS) { rounded -= N_LEDS; }
    set_led_active(rounded);
}

static const int16_t MAX_VELOCITY = 10000;

#define REV_PER_LED (65536 / N_LEDS)

static uint16_t abs(int16_t n) { return (uint16_t)(n >= 0 ? n : -n); }

static const int16_t STICKY_COLL_THRESHOLD = 1000;

static inline void flip_words(int16_t* w1, int16_t* w2) {
                     //   w1      w2
    *w1 = *w1 + *w2; // w1 + w2   w2
    *w2 = *w1 - *w2; // w1 + w2   w1
    *w1 = *w1 - *w2; //   w2      w1
}

const void scale_velocity(uint8_t i) {
    const int16_t V_SCALE = 26214; // 0.8 scaled from [-1,1) to [-32768,32768)
    int32_t v = ((int32_t)velocity[i]) * V_SCALE;
    velocity[i] = *((int16_t*)&v);
}

static void fix_collisions_ord(uint8_t left, uint8_t right) {
    //   <--L    <-R    or   <--L    R->
    if (velocity[left] <= 0 && velocity[right] > velocity[left]) return;
    //    <-L   R-->    or    L->   R-->
    if (velocity[right] >= 0 && velocity[left] < velocity[right]) return;

    // We are now left with "true" collisions
    //     L--> <--R        =>       <--L     R-->     (bounce)
    //     L-->    R->      =>          L->   R-->     (bump)
    //   <-L    <--R        =>       <--L   <-R        (bump)
    // In each case, we simply reverse the velocities and then reduce them
    // by a fixed scale factor for "lost" energy
    flip_words(&velocity[left], &velocity[right]);
    scale_velocity(left);
    scale_velocity(right);
}

static void fix_collisions(uint8_t i, uint8_t j) {
    if (position[j] - position[i] < REV_PER_LED) {
        // i (fixed) <-- too close --> j
        position[j] = position[i] + REV_PER_LED;
        fix_collisions_ord(i, j);
    } else if (position[i] - position[j] < REV_PER_LED) {
        // j <-- too close --> i (fixed)
        position[j] = position[i] - REV_PER_LED;
        fix_collisions_ord(j, i);
    }
}

static void update_pt(uint8_t i) {
    uint16_t raw = down_angle - position[i];
    bool flip = raw > 32768; // i.e. positive angle is > 1/2 turn
    if (raw == 32768) raw = 0; // for exactly 1/2 turn, ignore gravity
    if (flip) raw = ((65535 - raw) + 1);
    raw >>= 8; // Divide by 256 before adding to velocity
    int16_t gravity = *((int16_t*)&raw);
    if (flip) gravity = -gravity;

    velocity[i] += gravity;
    if (velocity[i] > MAX_VELOCITY) { velocity[i] = MAX_VELOCITY; }
    if (velocity[i] < -MAX_VELOCITY) { velocity[i] = -MAX_VELOCITY; }
    if (velocity[i] > 0) {
        position[i] += ((uint16_t)velocity[i]) >> 4;
    } else {
        position[i] -= ((uint16_t)(-velocity[i])) >> 4;
    }
}

void update_physics() {
    for (uint8_t i = 0; i < N_PTS; i++) {
        update_pt(i);
    }

    // N-1 <--> 0
    fix_collisions(0, N_PTS - 1);
    // 0 <--> 1 <--> ... <--> N-1
    for (uint8_t i = 0; i < N_PTS; i++) {
        fix_collisions(i, i + 1);
    }

    for (uint8_t i = 0; i < N_PTS; i++) {
        set_led_pt(position[i]);
    }
}