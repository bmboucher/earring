#include <Arduino.h>
#include "led_driver.h"
#include "pins.h"

uint8_t active_color[3] = {0x00, 0x00, 0x00};
uint8_t active_flags[N_LED_BYTES];

static volatile uint8_t *port;
static uint8_t pinMask;

void init_leds(void) {
  port    = portOutputRegister(digitalPinToPort(LED_PIN));
  pinMask = digitalPinToBitMask(LED_PIN);
}

void clear_led_data(void) {
  memset(active_flags, 0, N_LED_BYTES);
}

void set_led_active(uint8_t n) {
  uint8_t byte = n / 8;
  uint8_t bit = 1 << (n & 0x07);
  active_flags[byte] |= bit;
}

// Safety feature - always check that at most this many LEDs are active
#define MAX_LEDS_ON 8

// *INDENT-OFF*   astyle don't like assembly
void update_leds(void) {
  uint8_t leds_on = 0;
  for (uint8_t i = 0; i < N_LED_BYTES; i++) {
    byte curr_flag = active_flags[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (curr_flag & 0x80) leds_on++;
      curr_flag <<= 1;
    }
  }
  if (leds_on > MAX_LEDS_ON) return;

  noInterrupts(); // Need 100% focus on instruction timing

  volatile uint8_t
    led = N_LEDS,
    rgb_flag = 1,
    red = active_color[0],
    green = active_color[1],
    blue = active_color[2],
    rgb_byte = red,
    *ptr = active_flags,
    active = *ptr++,
    hi = *port |  pinMask,
    lo = *port & ~pinMask,
    next = lo,
    bit = 8,
    flag_bit = 8;

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
      "rejoin1:"                  "\n\t" //
        "rjmp .+0"                "\n\t" // 2                           (T = 20)
      "rejoin2:"                  "\n\t"
        "nop"                     "\n\t" // 1                           (T = 21)
      "rejoin3:"                  "\n\t"
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
