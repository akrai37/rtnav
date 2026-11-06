#include "motors.h"

#include <avr/io.h>

static uint8_t magnitude(int16_t duty) {
  if (duty < 0) {
    duty = (int16_t)-duty;
  }
  return duty > 255 ? 255 : (uint8_t)duty;
}

void motors_init(void) {
  PORTD &= (uint8_t)~((1u << PD6) | (1u << PD7) | (1u << PD5) | (1u << PD4) | (1u << PD2));
  PORTB &= (uint8_t)~(1u << PB0);
  DDRD |= (uint8_t)((1u << PD6) | (1u << PD7) | (1u << PD5) | (1u << PD4) | (1u << PD2));
  DDRB |= (uint8_t)(1u << PB0);
  // Fast PWM, TOP 0xFF; outputs stay disconnected until a duty is set.
  TCCR0A = (uint8_t)((1u << WGM01) | (1u << WGM00));
  TCCR0B = (uint8_t)((1u << CS01) | (1u << CS00));  // 16 MHz / 64 / 256 = 977 Hz
}

// In fast PWM a compare value of 0 still gives a one-count pulse every
// period, so zero duty disconnects the output instead.
void motors_set(nav_motors m) {
  const uint8_t left = magnitude(m.left);
  const uint8_t right = magnitude(m.right);

  if (left) {
    OCR0A = left;
    TCCR0A |= (uint8_t)(1u << COM0A1);
  } else {
    TCCR0A &= (uint8_t)~(1u << COM0A1);
  }
  if (right) {
    OCR0B = right;
    TCCR0A |= (uint8_t)(1u << COM0B1);
  } else {
    TCCR0A &= (uint8_t)~(1u << COM0B1);
  }

  // Direction: forward IN1 high, reverse IN2 high, coast both low.
  uint8_t d = PORTD & (uint8_t)~((1u << PD7) | (1u << PD4) | (1u << PD2));
  uint8_t b = PORTB & (uint8_t)~(1u << PB0);
  if (m.left > 0) {
    d |= (uint8_t)(1u << PD7);
  } else if (m.left < 0) {
    b |= (uint8_t)(1u << PB0);
  }
  if (m.right > 0) {
    d |= (uint8_t)(1u << PD4);
  } else if (m.right < 0) {
    d |= (uint8_t)(1u << PD2);
  }
  PORTD = d;
  PORTB = b;
}
