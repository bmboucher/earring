#include <Arduino.h>
#include "led_driver.h"
#include "pins.h"

uint8_t active_color[3] = {0x00, 0x00, 0x00};
uint8_t active_flags[N_LED_BYTES];

static long last_update = 0;
static volatile uint8_t *port;
static uint8_t pinMask;

void init_leds(void) {
  port    = portOutputRegister(digitalPinToPort(LED_PIN));
  pinMask = digitalPinToBitMask(LED_PIN);
}

// *INDENT-OFF*   astyle don't like assembly
void update_leds(void) {

  // Data latch = 50+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while (micros() - last_update < 50L);
  // endTime is a private member (rather than global var) so that multiple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions.  It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT.  The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  noInterrupts(); // Need 100% focus on instruction timing

  // AVRxt MCUs --  tinyAVR 0/1/2, megaAVR 0, AVR Dx ----------------------
  // with extended maximum speeds to support vigorously overclocked
  // Dx-series parts. This is by no means intended to imply that they will
  // run at those speeds, only that - if they do - you can control WS2812s
  // with them.

  volatile uint8_t
    led = N_LEDS,
    rgb_flag = 1,
    red = active_color[0],
    green = active_color[1],
    blue = active_color[2],
    rgb_byte = red,
    *flag_ptr = active_flags,
    active = *flag_ptr++,
    hi = *port |  pinMask,
    lo = *port & ~pinMask,
    next = lo,
    bit = 8,
    flag_bit = 8;

  asm volatile (
      // Main loop, repeats every 25 cycles for each bit to send
      "head:"                     "\n\t" // Clk  Pseudocode    (T =  0)
        "st   %a[port], %[hi]"    "\n\t" // 1    PORT = hi     (T =  1)
        "sbrc %[rgb_byte], 7"     "\n\t" // 1-2  if (rgb_byte & 0x80)
        "mov  %[next], %[hi]"     "\n\t" // 0-1  next = hi     (T =  3)
        "sbrs %[active], 7"       "\n\t" // 1-2  if (!active)
        "mov  %[next], %[lo]"     "\n\t" // 0-1  next = lo     (T =  5)
        "dec  %[bit]"             "\n\t" // 1    bit--         (T =  6)
        "nop"                     "\n\t" // 1                  (T =  7)
        "st   %a[port],  %[next]" "\n\t" // 1    PORT = next   (T =  8)
        "breq nextbyte"           "\n\t" // 1-2  if (bit == 0) -> nextbyte
        "rol  %[rgb_byte]"        "\n\t" // 1    else rgb_byte <<= 1 (T = 10)
        "cp   %[bit], 1"          "\n\t" // 1                  (T = 11)
        "breq incrgb"             "\n\t" // 1-2  if (bit == 1) -> incrgb
        "nop"                     "\n\t" // 1                  (T = 13)
        "rjmp .+0"                "\n\t" // 2                  (T = 15)
        "st   %a[port],  %[lo]"   "\n\t" // 1    PORT = lo     (T = 16)
        "rjmp .+0"                "\n\t" // 2                  (T = 18)
      "endbit:"                   "\n\t"
        "nop"                     "\n\t" // 1                  (T = 19)
        "rjmp .+0"                "\n\t" // 2                  (T = 21)
        "rjmp .+0"                "\n\t" // 2                  (T = 23)
        "rjmp head"               "\n\t" // 2    -> head (next bit out)

      // On the second to last bit of each byte, update R->G->B->R index
      "incrgb:"                   "\n\t" //      if (bit == 1)  (T = 13)
        "clc"                     "\n\t" // 1    clear carry    (T = 14)
        "rol  %[rgb_flag]"        "\n\t" // 1    rgb_flag <<= 1 (T = 15)
        "st   %a[port],  %[lo]"   "\n\t" // 1    PORT = lo      (T = 16)
        "sbrc %[rgb_flag], 3"     "\n\t" // 1-2  if (rgb_flag & 0x08)
        "ldi  %[rgb_flag], 1"     "\n\t" // 0-1    rgb_flag = 1 (T = 18)
        "nop"                     "\n\t" // 2                   (T = 19)
        "rjmp .+0"                "\n\t" // 2                   (T = 21)
        "rjmp .+0"                "\n\t" // 2                   (T = 23)
        "rjmp head"               "\n\t" // 2    -> head (next bit out)

      // On the last bit of each byte, set the color
      "nextbyte:"                 "\n\t" //                             (T = 10)
        "cp   %[rgb_flag], 1"     "\n\t" // 1    if (rgb_flag == 1)     (T = 11)
        "breq nextled"           "\n\t" // 1-2     -> nextled
        "mov  %[rgb_byte], %[green]\n\t" // 1    rgb_byte = green       (T = 13)
        "sbrc %[rgb_flag], 2"     "\n\t" // 1-2  if (rgb_flag & 0x04)
        "mov  %[rgb_byte], %[blue] \n\t" // 0-1    rgb_byte = blue      (T = 15)
        "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo              (T = 16)
        "rjmp endbit"             "\n\t" // 2    -> endbit              (T = 18)

      // When the RGB cycle completes, advance to the next led
      "nextled:"                 "\n\t"  //                             (T = 13)
        "rol %[active]"           "\n\t" // 1    active <<= 1           (T = 14)
        "mov %[rgb_byte], %[red]   \n\t" // 1    rgb_byte = red         (T = 15)
        "st   %a[port], %[lo]"    "\n\t" // 1    PORT = lo              (T = 16)
        "dec %[led]"              "\n\t" // 1    led--                  (T = 17)
        "breq breakloop"          "\n\t" // 1-2  if (led == 0) -> exit
        "dec %[flag_bit]"         "\n\t" // 1    flag_bit--             (T = 19)
        "breq nextflagbyte"       "\n\t" // 1-2  if (flag_bit == 0) -> nextflagbyte
        "nop"                     "\n\t" // 1                           (T = 21)
        "rjmp .+0"                "\n\t" // 2                           (T = 23)
        "rjmp head"               "\n\t" // 2    -> head (next bit out)

      // Every 8 LEDs, we need to advance the flag pointer to the next byte
      "nextflagbyte:"             "\n\t" //      if (flag_bit == 0) (T = 21)
        "ld   %[active], %a[flag_ptr]+  \n\t" // 2    active = *flag_ptr++ (T = 23)
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
  : [flag_ptr] "e" (flag_ptr),
    [red] "r" (red),
    [green] "r" (green),
    [blue] "r" (blue),
    [hi] "r" (hi),
    [lo] "r" (lo));

  interrupts();
  last_update = micros();
}
