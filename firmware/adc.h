#ifndef RTNAV_ADC_H
#define RTNAV_ADC_H

#include <stdint.h>

#include "board.h"

// Interrupt-driven round robin over ADC0..ADC5: each conversion-complete
// interrupt stores its result, selects the next channel and starts the next
// conversion. At 125 kHz ADC clock a conversion takes 104 us, so every
// channel is refreshed about every 0.6 ms, ~16 times per control tick.
void adc_init(void);

// Copies the latest sample of every channel. The copy runs with interrupts
// off: each sample is two bytes and the CPU reads them one at a time, so
// without it an update landing between the two reads tears the value.
void adc_snapshot(uint16_t out[ADC_CHANNELS]);

#endif
