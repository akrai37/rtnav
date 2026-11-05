#ifndef RTNAV_SIM_H
#define RTNAV_SIM_H

// Closed-loop simulation of the robot on a track, for the host.
//
// The world: a differential-drive body whose wheels follow the commanded
// duty with a first-order lag and a deadband, five reflectance sensors
// with noise and per-sensor calibration mismatch, and a Sharp GP2Y0A21-style
// IR rangefinder with the real sensor's non-linear response (which also
// falls again below 10 cm, so very close objects read as further away).
//
// The controller is a callback, so the same world can drive the portable
// navigation code directly or the compiled firmware inside an AVR simulator.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "rtnav/nav.h"

#define SIM_MAX_OBSTACLES 4
#define SIM_TICK_S 0.01

typedef struct {
  double x, y;
} sim_vec;

typedef struct {
  sim_vec c;
  double r;
} sim_obstacle;

typedef struct {
  const char* name;
  sim_vec* pts;   // centreline, closed, evenly spaced; the robot starts at pts[0]
  bool* painted;  // painted[i]: tape on the segment from pts[i] to pts[i+1]
  size_t n;
  double spacing;     // mm between consecutive points
  double length;      // mm
  double line_width;  // mm
  double min_radius;  // tightest curve, mm
  sim_obstacle obstacles[SIM_MAX_OBSTACLES];
  size_t n_obstacles;
} sim_track;

// Built-in tracks, NULL-terminated: "oval", "lobes", "gaps", "obstacle".
extern const char* const sim_track_names[];

// Returns false for an unknown name.
bool sim_track_build(sim_track* t, const char* name);
void sim_track_free(sim_track* t);

typedef struct {
  uint64_t seed;
  double line_noise;          // ADC counts, standard deviation
  double distance_noise;      // ADC counts, standard deviation
  double calibration_spread;  // +- ADC counts between individual line sensors
  double left_gain;           // motor strength, 1.0 nominal; < 1 is a weak motor
  double right_gain;
  int dead_sensor;  // line sensor stuck at dead_value, or -1
  uint16_t dead_value;
} sim_params;

sim_params sim_default_params(void);

typedef struct {
  const sim_track* track;
  sim_params p;
  uint64_t rng;
  double white[NAV_LINE_SENSORS];  // this robot's true sensor levels
  double black[NAV_LINE_SENSORS];
  double x, y, heading;    // axle centre (mm) and heading (radians)
  double v_left, v_right;  // wheel speeds, mm/s
  double time;             // s
  size_t nearest;          // centreline index nearest the axle
  double progress;         // mm along the track; negative if driven backwards
} sim_world;

void sim_world_init(sim_world* w, const sim_track* t, const sim_params* p);

// One ADC sample of every sensor.
void sim_sense(sim_world* w, nav_inputs* in);

// Applies the motor command for dt seconds.
void sim_drive(sim_world* w, nav_motors cmd, double dt);

// Distance from the sensor bar's centre to the track centreline, mm.
double sim_cross_track(const sim_world* w);

// Gap between the robot's body and the nearest obstacle, mm (negative:
// collision). Very large when the track has no obstacles.
double sim_clearance(const sim_world* w);

// Front rangefinder reading for an object `mm` away (no noise).
double sim_rangefinder_adc(double mm);

// One control tick: compute the motor command from the sensor readings and
// report the navigation state the controller is now in.
typedef nav_motors (*sim_controller)(void* ctx, const nav_inputs* in, nav_state* state);

// The portable navigation code as a controller; ctx is a nav*.
nav_motors sim_nav_controller(void* ctx, const nav_inputs* in, nav_state* state);

typedef struct {
  unsigned laps;     // 0 means 1
  double timeout_s;  // 0: derived from the distance to cover
  FILE* trace;       // optional CSV, one row per tick
} sim_run_options;

typedef struct {
  bool completed;  // covered the laps without a collision or giving up
  bool collided;
  double time_s;
  double max_xte_mm;  // cross-track error while in FOLLOW
  double rms_xte_mm;
  double min_clearance_mm;
  nav_state final_state;
  uint32_t ticks;
  unsigned searches;  // entries into SEARCH
  unsigned avoids;    // entries into AVOID
} sim_result;

sim_result sim_run(const sim_track* t, const sim_params* p, const sim_run_options* opt,
                   sim_controller ctl, void* ctx);

#endif
