#ifndef RTNAV_NAV_H
#define RTNAV_NAV_H

// Navigation logic, independent of the hardware: the same code runs in the
// AVR firmware, the host simulation and the tests. Called once per control
// tick (10 ms) with the latest raw sensor readings, it returns the motor
// commands for the next tick.
//
// Everything is integer arithmetic. The ATmega328P has no floating-point
// unit, and on AVR `int` is 16 bits, so intermediate results are computed
// in explicitly sized int32_t.
//
// States:
//   FOLLOW  PD control on the line position from five reflectance sensors
//   SEARCH  the line was lost: turn toward the side it was last seen on
//   AVOID   an obstacle ahead: turn away, arc around it, rejoin the line
//   STOP    safe state after a search or avoidance gives up: motors off

#include <stdbool.h>
#include <stdint.h>

#define NAV_LINE_SENSORS 5
#define NAV_ADC_MAX 1023
#define NAV_MOTOR_MAX 255
// Line position runs from -2000 (line under the leftmost sensor) to +2000.
#define NAV_POSITION_MAX 2000

typedef enum { NAV_FOLLOW, NAV_SEARCH, NAV_AVOID, NAV_STOP } nav_state;

// Raw 10-bit ADC readings for one tick.
typedef struct {
  uint16_t line[NAV_LINE_SENSORS];  // left to right; higher = darker (on the line)
  uint16_t distance;                // front IR rangefinder; higher = closer
} nav_inputs;

// Signed PWM duty per motor: -255 full reverse .. +255 full forward.
typedef struct {
  int16_t left;
  int16_t right;
} nav_motors;

typedef struct {
  // Per-sensor ADC readings over white floor and over the black line.
  uint16_t white[NAV_LINE_SENSORS];
  uint16_t black[NAV_LINE_SENSORS];
  int16_t base_speed;  // duty when centred on the line
  int16_t kp_q8;       // proportional gain, 8 fractional bits (256 = 1.0 per position unit / 1000)
  int16_t kd_q8;       // derivative gain, same scale
  uint16_t line_lost;  // total normalised reflectance below this means no line
  uint16_t obstacle_adc;      // distance reading at or above this means an obstacle
  uint16_t search_ticks;      // give up searching after this long
  uint16_t avoid_turn_ticks;  // phase 1 of avoidance: turn away
  uint16_t avoid_arc_ticks;   // phase 2: arc around before looking for the line
  uint16_t avoid_max_ticks;   // give up avoiding after this long
} nav_config;

typedef struct {
  nav_config cfg;
  nav_state state;
  int32_t last_position;  // position on the previous tick, for the derivative
  int8_t last_side;       // -1 line last seen to the left, +1 to the right
  uint16_t ticks_in_state;
  uint32_t ticks;
} nav;

// Reasonable defaults for the simulated robot (and a typical 5-sensor
// array with a Sharp-style rangefinder).
nav_config nav_default_config(void);

void nav_init(nav* n, const nav_config* cfg);

// One control tick.
nav_motors nav_step(nav* n, const nav_inputs* in);

const char* nav_state_name(nav_state state);

// Exposed for testing: normalised reflectance (0 white .. 1000 black) of
// sensor `i`, and the weighted line position. `found` is false when the
// summed reflectance is below cfg->line_lost.
uint16_t nav_normalise(const nav_config* cfg, unsigned i, uint16_t raw);
int32_t nav_line_position(const nav_config* cfg, const nav_inputs* in, bool* found);

#endif
