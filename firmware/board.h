#ifndef RTNAV_BOARD_H
#define RTNAV_BOARD_H

// Pin map for an Arduino Uno (ATmega328P) with an H-bridge motor driver
// (L298N / TB6612-style: a PWM input and two direction inputs per motor).
//
//   A0..A4  PC0..PC4  line sensors, left to right (ADC0..ADC4)
//   A5      PC5       front IR rangefinder (ADC5)
//   D6      PD6/OC0A  left motor PWM
//   D7, D8  PD7, PB0  left motor direction
//   D5      PD5/OC0B  right motor PWM
//   D4, D2  PD4, PD2  right motor direction
//   D13     PB5       debug: high while the control step runs

#define ADC_CHANNELS 6
#define ADC_DISTANCE_CHANNEL 5

#define DEBUG_DDR DDRB
#define DEBUG_PORT PORTB
#define DEBUG_BIT PB5

#endif
