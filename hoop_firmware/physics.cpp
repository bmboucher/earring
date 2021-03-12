#include <Arduino.h>

#include "physics.h"
#include "accel.h"
#include "led_driver.h"

// Useful constants in "brads"
#define FULL_REV 65536
#define INIT_POS (FULL_REV / N_PTS)
#define REV_PER_LED (FULL_REV / N_LEDS)

// When accumulating acceleration -> velocity -> position, we apply bit shifts
// at each stage to prevent overly rapid movement.
#define GRAVITY_SHIFT_BITS 4
#define VELOCITY_SHIFT_BITS 4

// Limit velocity to the range of [-0.5, 0.5]
#define MAX_VELOCITY 16384
// Factor of 0.95 scaled from [-1,1) to [-32768,32768)
#define COLLISION_FACTOR 31130

// ===== GLOBAL STATE VARIABLES =====
// Angular position of each point, [0,65536) = 1 full revolution
uint16_t position[N_PTS];
// Angular velocity of each point, limited to [-MAX_VELOCITY, MAX_VELOCITY]
int16_t velocity[N_PTS];

// Initialize state variables
// All particles start motionless and equally distributed around the circle
void init_physics() {
    memset(velocity, 0, sizeof(uint16_t) * N_PTS);
    for (uint8_t i = 0; i < N_PTS; i++) {
        position[i] = i * INIT_POS;
    }
}

// Convert an angle position to an LED number and light it
static void set_led_pt(uint16_t angle) {
    uint16_t scaled = angle >> 6; // Scale [0,1) to [0,1/64) to avoid overflow
    uint16_t segment = scaled * N_LEDS; // Multiply by 60 - range is [0,60/64)
    // Round to nearest 1/64 by adding 1/128 and taking 6 MSB 
    uint8_t led_num = (uint8_t)((segment + 512) >> 10);
    // May round to LED 60 -> need to reset to LED 0
    while (led_num >= N_LEDS) { led_num -= N_LEDS; }

    // Turn on the LED (in data)
    set_led_active(led_num);
}

// Flip two words in-place (used to interchange velocities during collisions)
static inline void flip_words(int16_t* w1, int16_t* w2) {
                     //   w1      w2
    *w1 = *w1 + *w2; // w1 + w2   w2
    *w2 = *w1 - *w2; // w1 + w2   w1
    *w1 = *w1 - *w2; //   w2      w1
}

// Adjust down a velocity after a collision
const void scale_velocity(uint8_t i) {
    int32_t v = ((int32_t)velocity[i]) * COLLISION_FACTOR;
    velocity[i] = *((int16_t*)&v);
}

// Adjust velocities of two particles after collision; assumes positions
// have been fixed and left/right have been sorted already
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

// Check for and fix collisions between two particles
// Will keep the position of particle i fixed and adjust particle j's position
// so the distance between the two is at least 1 LED
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

// Perform the basic Euler's method update of a particle's state variables
static void update_pt(uint8_t i) {
    // Determine the magnitude and direction of "gravity" (i.e. the minimum
    // distance to the down angle)
    uint16_t gravityR = down_angle - position[i];
    uint16_t gravityL = position[i] - down_angle;
    int16_t gravity = (gravityR < gravityL) 
        ? (int16_t)(gravityR >> GRAVITY_SHIFT_BITS) 
        : -(int16_t)(gravityL >> GRAVITY_SHIFT_BITS);

    // Update the velocity from acceleration and apply limits
    velocity[i] += gravity;
    if (velocity[i] > MAX_VELOCITY) { velocity[i] = MAX_VELOCITY; }
    if (velocity[i] < -MAX_VELOCITY) { velocity[i] = -MAX_VELOCITY; }

    // Update the position from velocity
    if (velocity[i] > 0) {
        position[i] += ((uint16_t)velocity[i]) >> VELOCITY_SHIFT_BITS;
    } else {
        position[i] -= ((uint16_t)(-velocity[i])) >> VELOCITY_SHIFT_BITS;
    }
}

// Physics update loop
void update_physics() {
    // Per-particle kinematics
    for (uint8_t i = 0; i < N_PTS; i++) {
        update_pt(i);
    }

    // Particle collisions
    // N-1 <--> 0
    fix_collisions(0, N_PTS - 1);
    // 0 <--> 1 <--> ... <--> N-1
    for (uint8_t i = 0; i < N_PTS; i++) {
        fix_collisions(i, i + 1);
    }

    // Output results
    for (uint8_t i = 0; i < N_PTS; i++) {
        set_led_pt(position[i]);
    }
}