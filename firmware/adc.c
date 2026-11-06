#include "adc.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#define ADC_PRESCALE ((1u << ADPS2) | (1u << ADPS1) | (1u << ADPS0))  // 16 MHz / 128

#ifdef RTNAV_VARIANT_BLOCKING_ADC

// Negative control: 32x oversampling by polling inside the control step.
// 6 channels x 32 conversions x 104 us is about 20 ms, twice the tick.
#define OVERSAMPLE 32u

void adc_init(void) {
  DIDR0 = 0x3F;
  ADMUX = (uint8_t)(1u << REFS0);
  ADCSRA = (uint8_t)((1u << ADEN) | ADC_PRESCALE);
}

void adc_snapshot(uint16_t out[ADC_CHANNELS]) {
  for (uint8_t ch = 0; ch < ADC_CHANNELS; ++ch) {
    ADMUX = (uint8_t)((1u << REFS0) | ch);
    uint32_t sum = 0;
    for (uint8_t k = 0; k < OVERSAMPLE; ++k) {
      ADCSRA |= (uint8_t)(1u << ADSC);
      while (ADCSRA & (1u << ADSC)) {
      }
      sum += ADC;
    }
    out[ch] = (uint16_t)(sum / OVERSAMPLE);
  }
}

#else

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
  ADCSRA = (uint8_t)((1u << ADEN) | (1u << ADIE) | ADC_PRESCALE);
  ADCSRA |= (uint8_t)(1u << ADSC);
}

void adc_snapshot(uint16_t out[ADC_CHANNELS]) {
#ifdef RTNAV_VARIANT_TORN_READ
  // Negative control: the same copy without the interrupt lock.
  for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
    out[i] = samples[i];
  }
#else
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < ADC_CHANNELS; ++i) {
      out[i] = samples[i];
    }
  }
#endif
}

#endif
