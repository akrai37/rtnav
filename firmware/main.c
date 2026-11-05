// rtnav firmware entry point (ATmega328P, 16 MHz).
//
// Timer1 raises a compare-match interrupt every 10 ms. The interrupt only
// marks a tick as due; the main loop runs the control step and then sleeps
// until the next interrupt, so the CPU idles between ticks and other
// interrupts (the ADC) are never held up by control work.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>

#define CONTROL_HZ 100u
#define TIMER1_PRESCALER 64u

static volatile uint8_t tick_due;

ISR(TIMER1_COMPA_vect) {
  tick_due = 1;
}

// CTC mode on Timer1: count clk/64 up to OCR1A, then reset and interrupt.
// 16 MHz / 64 = 250 kHz, so OCR1A = 2499 gives exactly 100 Hz.
static void timer1_start(void) {
  TCCR1A = 0;
  OCR1A = (uint16_t)(F_CPU / TIMER1_PRESCALER / CONTROL_HZ - 1u);
  TIMSK1 = (uint8_t)(1u << OCIE1A);
  TCCR1B = (uint8_t)((1u << WGM12) | (1u << CS11) | (1u << CS10));
}

int main(void) {
  timer1_start();
  set_sleep_mode(SLEEP_MODE_IDLE);
  sei();
  for (;;) {
    // Sleep only if no tick is pending. Interrupts are disabled while
    // checking and `sei` takes effect after the next instruction, so a tick
    // arriving between the check and `sleep` still wakes the CPU.
    cli();
    if (!tick_due) {
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
    }
    sei();
    if (tick_due) {
      tick_due = 0;
      // The control step runs here (sensing, navigation, motor output).
    }
  }
}
