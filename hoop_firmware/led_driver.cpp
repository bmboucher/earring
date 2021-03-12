#include <Arduino.h>
#include "led_driver.h"
#include "pins.h"

// This is a highly modified addressable RGB led driver; inspiration was taken
// from the tinyNeoPixel library in Spence Konde's megaTinyCore project 
// (https://github.com/SpenceKonde/megaTinyCore) which in turn borrows from the
// Adafruit library (https://github.com/adafruit/Adafruit_NeoPixel). Both
// require 3 bytes per LED, which would be 180 bytes for our 60 LED earring; the
// ATtiny402 only has 256 bytes of RAM to begin with, so we need to reduce this.
//
// We don't need to set colors for individual LEDs, just turn individual LEDs
// "on" and "off" with a single RGB color; with 60 LEDs that requires just
// 8 bytes for the bit flags (8*8=64>60) + 3 bytes for color = 11 bytes TOTAL.
//
// This is accomplished by replacing the "pointer to color byte" in the original
// assembly with a "pointer to flag byte" - we can't have more than 1 of these
// pointers apparently (no Y register?) so the color bytes are loaded into
// registers instead. To get the timing right required breaking into smaller
// subroutines.

// RGB value to be sent to all active LEDs
uint8_t active_color[3] = {0x10, 0x10, 0x10}; // Very low white
// Bitwise flags indicating which LEDs are active
static uint8_t active_flags[N_LED_BYTES];

// On initialization, obtain the port register and pin mask
// We do fast I/O on the pin by:
//     READ   *port & pinMask
//     SET    *port = *port | pinMask
//     CLEAR  *port = *port & ~pinMask
static volatile uint8_t *port;
static uint8_t pinMask;
void init_leds(void) {
  pinMode(LED_PIN, OUTPUT);
  port    = portOutputRegister(digitalPinToPort(LED_PIN));
  pinMask = digitalPinToBitMask(LED_PIN);
  clear_led_data();
}

// Running count of LEDs that have been set; used for safety purposes so that
// no code can accidentally turn on too many LEDs and burn out the voltage
// regulator.
uint8_t active_led_count = 0xFF;

// Turns all LEDs off; this happens every update cycle by default
void clear_led_data(void) {
  memset(active_flags, 0, N_LED_BYTES);
  active_led_count = 0;
}

// Enables a particular LED by setting its bit flag
void set_led_active(uint8_t n) {
  // Safety mechanism, only turn on the first MAX_LEDS_ON LEDs requested each cycle
  if (active_led_count >= MAX_LEDS_ON) { return; }
  active_led_count++;
  uint8_t byte = n / 8;
  uint8_t bit = 1 << (n & 0x07);
  active_flags[byte] |= bit;
}

// Sends LED RGB values based on active_color and active_flags
void update_leds(void) {
  // Safety mechanism; in theory, this condition should never occur
  if (active_led_count > MAX_LEDS_ON) { return; }

  noInterrupts(); // Need 100% focus on instruction timing

  // Register definitions
  volatile uint8_t
    led = N_LEDS,            // Counts down from N_LEDS to 0
    rgb_flag = 1,            // 1 = red, 2 = blue, 4 = green (i.e. << to inc)
    red = active_color[0],   // R byte
    green = active_color[1], // G byte
    blue = active_color[2],  // B byte
    rgb_byte = red,          // Next color byte to send
    *ptr = active_flags,     // Next pointer location in flag buffer
    active = *ptr++,         // Currently loaded flag byte
    hi = *port |  pinMask,   // Port value to set pin HI
    lo = *port & ~pinMask,   // Port value to set pin LO
    next = lo,               // Next port value (HI if color bit sed and LED active)
    bit = 8,                 // Current color bit (8..0 -> next color or next LED)
    flag_bit = 8;            // Current flag bit (8..0 -> advance flag pointer)

  // Assembly
  asm volatile (
      // Main loop, repeats every 25 cycles for each bit to send
      "head:"                     "\n\t" // Clk  Pseudocode             (T =  0)
        "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi              (T =  1)
        "sbrc %[rgb_byte], 7"     "\n\t" // 1-2  if (rgb_byte & 0x80)
        "mov  %[next], %[hi]"     "\n\t" // 0-1  next = hi              (T =  3)
        "sbrs %[active], 7"       "\n\t" // 1-2  if (!active)
        "mov  %[next], %[lo]"     "\n\t" // 0-1  next = lo              (T =  5)
        "dec  %[bit]"             "\n\t" // 1    bit--                  (T =  6)
        "nop"                     "\n\t" // 1                           (T =  7)
        "st   %a[port],  %[next]" "\n\t" // 1    PORT = next            (T =  8)
        "breq nextbyte"           "\n\t" // 1-2  if (bit == 0) -> nextbyte
        "rol  %[rgb_byte]"        "\n\t" // 1    else rgb_byte <<= 1    (T = 10)
        "cp   %[bit], 1"          "\n\t" // 1                           (T = 11)
        "breq incrgb"             "\n\t" // 1-2  if (bit == 1) -> incrgb
        "nop"                     "\n\t" // 1                           (T = 13)
        "rjmp .+0"                "\n\t" // 2                           (T = 15)
        "st   %a[port],  %[lo]"   "\n\t" // 1    PORT = lo              (T = 16)
        "rjmp .+0"                "\n\t" // 2                           (T = 18)
      "rejoin1:"                  "\n\t" //                             (T = 18)
        "rjmp .+0"                "\n\t" // 2                           (T = 20)
      "rejoin2:"                  "\n\t" //                             (T = 20)
        "nop"                     "\n\t" // 1                           (T = 21)
      "rejoin3:"                  "\n\t" //                             (T = 21)
        "rjmp .+0"                "\n\t" // 2                           (T = 23)
        "rjmp head"               "\n\t" // 2    -> head (next bit out)

      // On the second to last bit of each byte, update R->G->B->R index
      "incrgb:"                   "\n\t" //      if (bit == 1)          (T = 13)
        "clc"                     "\n\t" // 1    clear carry            (T = 14)
        "rol  %[rgb_flag]"        "\n\t" // 1    rgb_flag <<= 1         (T = 15)
        "st   %a[port],  %[lo]"   "\n\t" // 1    PORT = lo              (T = 16)
        "sbrc %[rgb_flag], 3"     "\n\t" // 1-2  if (rgb_flag & 0x08)
        "ldi  %[rgb_flag], 1"     "\n\t" // 0-1    rgb_flag = 1         (T = 18)
        "rjmp rejoin2"            "\n\t" // 2    -> rejoin2 (next bit out)

      // On the last bit of each byte, set the color
      "nextbyte:"                 "\n\t" //                             (T = 10)
        "cp   %[rgb_flag], 1"     "\n\t" // 1    if (rgb_flag == 1)     (T = 11)
        "breq nextled"           "\n\t" // 1-2     -> nextled
        "mov  %[rgb_byte], %[green]\n\t" // 1    rgb_byte = green       (T = 13)
        "sbrc %[rgb_flag], 2"     "\n\t" // 1-2  if (rgb_flag & 0x04)
        "mov  %[rgb_byte], %[blue] \n\t" // 0-1    rgb_byte = blue      (T = 15)
        "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo              (T = 16)
        "rjmp rejoin1"            "\n\t" // 2    -> rejoin1 (next bit out)

      // When the RGB cycle completes, advance to the next led
      "nextled:"                  "\n\t" //                             (T = 13)
        "rol %[active]"           "\n\t" // 1    active <<= 1           (T = 14)
        "mov %[rgb_byte], %[red]   \n\t" // 1    rgb_byte = red         (T = 15)
        "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo              (T = 16)
        "dec %[led]"              "\n\t" // 1    led--                  (T = 17)
        "breq breakloop"          "\n\t" // 1-2  if (led == 0) -> exit
        "dec %[flag_bit]"         "\n\t" // 1    flag_bit--             (T = 19)
        "brne rejoin3"            "\n\t" // 1-2  if (flag_bit != 0) -> rejoin3 (next bit out)
        "ld   %[active], %a[ptr]+  \n\t" // 2    active = *ptr++        (T = 23)
        "rjmp head"               "\n\t" // 2    -> head (next bit out)

      "breakloop:"
  : [port] "+e" (port),
    [led] "+r" (led),
    [rgb_flag] "+r" (rgb_flag),
    [rgb_byte] "+r" (rgb_byte),
    [active] "+r" (active),
    [next] "+r" (next),
    [bit] "+r" (bit),
    [flag_bit] "+r" (flag_bit)
  : [ptr] "e" (ptr),
    [red] "r" (red),
    [green] "r" (green),
    [blue] "r" (blue),
    [hi] "r" (hi),
    [lo] "r" (lo));

  interrupts();
}
