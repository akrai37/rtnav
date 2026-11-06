// rtnav firmware: a 100 Hz control loop on Timer1. The interrupt only flags
// that a tick is due; the control step (sensor snapshot, navigation, motor
// update) runs in the main loop, and the CPU sleeps in between.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>

#include "adc.h"
#include "board.h"
#include "motors.h"
#include "rtnav/nav.h"

#define CONTROL_HZ 100u
#define TIMER1_PRESCALER 64u
#define TIMER1_TOP (F_CPU / TIMER1_PRESCALER / CONTROL_HZ - 1u)  // 2499: 4 us per count

// Live diagnostics, readable with a debugger or by the firmware-in-the-loop
// harness (which looks the symbol up in the ELF).
typedef struct {
  uint32_t steps;             // control steps run
  uint16_t missed;            // ticks that arrived while the previous one was still pending
  uint16_t worst_end;         // worst Timer1 count at the end of a step (4 us units)
  uint8_t reset_cause;        // MCUSR at boot: bit 3 = watchdog reset
  uint8_t state;              // nav_state
  uint8_t masked;             // line sensors judged faulty
  uint16_t in[ADC_CHANNELS];  // the last sensor snapshot
} rtnav_debug_t;

volatile rtnav_debug_t rtnav_debug;

static volatile uint8_t tick_due;

ISR(TIMER1_COMPA_vect) {
  if (tick_due) {
    ++rtnav_debug.missed;
  }
  tick_due = 1;
}

static void timer1_start(void) {
  TCCR1A = 0;
  OCR1A = (uint16_t)TIMER1_TOP;
  TIMSK1 = (uint8_t)(1u << OCIE1A);
  TCCR1B = (uint8_t)((1u << WGM12) | (1u << CS11) | (1u << CS10));  // CTC, /64
}

// Sleeps until the tick interrupt. Interrupts are off between the check and
// the sleep (sei takes effect after the next instruction), so a tick cannot
// slip in between and leave the loop asleep for a whole period.
static void wait_for_tick(void) {
  for (;;) {
    cli();
    if (tick_due) {
      tick_due = 0;
      sei();
      return;
    }
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
}

int main(void) {
  rtnav_debug.reset_cause = MCUSR;
  MCUSR = 0;
  wdt_enable(WDTO_60MS);  // a hung loop resets the chip; at reset the motor pins go low

  DEBUG_DDR |= (uint8_t)(1u << DEBUG_BIT);
  motors_init();
  adc_init();

  nav n;
  const nav_config cfg = nav_default_config();
  nav_init(&n, &cfg);

  set_sleep_mode(SLEEP_MODE_IDLE);
  timer1_start();
  sei();

  for (;;) {
    wait_for_tick();
    DEBUG_PORT |= (uint8_t)(1u << DEBUG_BIT);

    uint16_t raw[ADC_CHANNELS];
    adc_snapshot(raw);
    nav_inputs in;
    for (uint8_t i = 0; i < NAV_LINE_SENSORS; ++i) {
      in.line[i] = raw[i];
    }
    in.distance = raw[ADC_DISTANCE_CHANNEL];
    motors_set(nav_step(&n, &in));
    wdt_reset();

    const uint16_t end = TCNT1;
    if (end > rtnav_debug.worst_end) {
      rtnav_debug.worst_end = end;
    }
    ++rtnav_debug.steps;
    rtnav_debug.state = (uint8_t)n.state;
    rtnav_debug.masked = n.masked;
    for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
      rtnav_debug.in[i] = raw[i];
    }
    DEBUG_PORT &= (uint8_t)~(1u << DEBUG_BIT);
  }
}
