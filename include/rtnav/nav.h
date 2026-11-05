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
//   SEARCH  the line was lost: if it was roughly centred (a gap in the
//           tape), drive straight for a moment; otherwise, or if that
//           fails, turn toward the side it was last seen on
//   AVOID   an obstacle ahead: turn right, away from it, then arc left
//           around it until the line is under the sensors again
//   REJOIN  the arc meets the line at a steep angle: drive on until the axle
//           is over it, then spin right until it is centred, and follow
//   STOP    safe state after a search, avoidance or rejoin gives up
//
// Avoidance timings are open loop, in ticks: on a real chassis they are
// calibrated, here they were tuned in the simulation (sim/).

#include <stdbool.h>
#include <stdint.h>

#define NAV_LINE_SENSORS 5
#define NAV_ADC_MAX 1023
#define NAV_MOTOR_MAX 255
// Line position runs from -2000 (line under the leftmost sensor) to +2000.
#define NAV_POSITION_MAX 2000

typedef enum { NAV_FOLLOW, NAV_SEARCH, NAV_AVOID, NAV_REJOIN, NAV_STOP } nav_state;

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
  uint16_t gap_ticks;         // drive straight this long when the line vanishes centred
  uint16_t search_ticks;      // give up searching after this long
  uint16_t avoid_turn_ticks;  // phase 1 of avoidance: turn away, about 90 degrees
  uint16_t avoid_arc_ticks;   // phase 2: arc this long before looking for the line
  uint16_t avoid_max_ticks;   // give up avoiding after this long
  int16_t avoid_outer;        // arc wheel duties; their ratio sets the radius
  int16_t avoid_inner;
  uint16_t rejoin_cross_ticks;  // drive on after meeting the line
  uint16_t rejoin_max_ticks;    // give up rejoining after this long
  uint8_t fault_ticks;          // dark but apart from the line this long: sensor is faulty
} nav_config;

typedef struct {
  nav_config cfg;
  nav_state state;
  int32_t last_position;              // position on the previous tick, for the derivative
  int8_t last_side;                   // -1 line last seen to the left, +1 to the right
  bool bridging;                      // in SEARCH: lost while centred, driving straight first
  uint8_t masked;                     // bit i: line sensor i judged faulty and ignored
  uint8_t suspect[NAV_LINE_SENSORS];  // consecutive ticks dark apart from the line
  uint16_t ticks_in_state;
  uint32_t ticks;
} nav;

// Where the line is, from one tick of readings.
typedef struct {
  int32_t position;  // -2000 .. 2000
  bool found;
  uint8_t run;   // bit set of the sensors the position came from
  uint8_t dark;  // bit set of every sensor reading dark
} nav_fix;

// Reasonable defaults for the simulated robot (and a typical 5-sensor
// array with a Sharp-style rangefinder).
nav_config nav_default_config(void);

void nav_init(nav* n, const nav_config* cfg);

// One control tick.
nav_motors nav_step(nav* n, const nav_inputs* in);

const char* nav_state_name(nav_state state);

// Exposed for testing: normalised reflectance (0 white .. 1000 black) of
// sensor `i`.
uint16_t nav_normalise(const nav_config* cfg, unsigned i, uint16_t raw);

// The tape is narrower than two sensor pitches, so the real line darkens
// only adjacent sensors. The position is the weighted average over the run
// of adjacent dark sensors nearest `near` (the previous position), skipping
// `masked` sensors; a dark sensor outside that run is noise or a fault.
// `found` is false when the run's summed reflectance is below
// cfg->line_lost.
nav_fix nav_locate(const nav_config* cfg, const nav_inputs* in, uint8_t masked, int32_t near);

#endif
