#include "adc.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

static volatile uint16_t samples[ADC_CHANNELS];
static volatile uint8_t channel;

// Single conversions started from the interrupt, not free-running mode: in
// free-running mode a new MUX setting applies only from the conversion
// after the one already under way, which misattributes samples.
ISR(ADC_vect) {
  uint8_t ch = channel;
  samples[ch] = ADC;
  ch = (uint8_t)(ch + 1u == ADC_CHANNELS ? 0u : ch + 1u);
  channel = ch;
  ADMUX = (uint8_t)((1u << REFS0) | ch);
  ADCSRA |= (uint8_t)(1u << ADSC);
}

void adc_init(void) {
  DIDR0 = 0x3F;  // ADC0..5 are analog only: disable their digital input buffers
  channel = 0;
  ADMUX = (uint8_t)(1u << REFS0);  // AVcc reference, channel 0
  // Enable, interrupt on completion, clock 16 MHz / 128 = 125 kHz.
  ADCSRA = (uint8_t)((1u << ADEN) | (1u << ADIE) | (1u << ADPS2) | (1u << ADPS1) | (1u << ADPS0));
  ADCSRA |= (uint8_t)(1u << ADSC);
}

void adc_snapshot(uint16_t out[ADC_CHANNELS]) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
      out[i] = samples[i];
    }
  }
}
