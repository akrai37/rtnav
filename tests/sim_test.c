// The simulated world on its own (geometry, sensor and motor models), then
// the navigation code driving it closed loop on every track.

#include "sim.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "rtnav/nav.h"

#define SEEDS 5

static sim_params quiet(void) {
  sim_params p = sim_default_params();
  p.line_noise = 0;
  p.distance_noise = 0;
  p.calibration_spread = 0;
  return p;
}

static void test_tracks(void) {
  check_context = "tracks";
  sim_track t;
  CHECK(!sim_track_build(&t, "nope"));
  CHECK(sim_track_build(&t, "oval"));
  const double expect = 2000.0 + 2.0 * 3.14159265358979 * 300.0;
  CHECK(fabs(t.length - expect) / expect < 0.005);
  CHECK(fabs(t.spacing - 5.0) < 0.1);
  CHECK(fabs(t.min_radius - 300.0) < 10.0);
  const sim_vec a = t.pts[t.n - 1];
  const sim_vec b = t.pts[0];
  CHECK(fabs(hypot(a.x - b.x, a.y - b.y) - t.spacing) < 0.5);  // closed
  sim_track_free(&t);

  CHECK(sim_track_build(&t, "lobes"));
  CHECK(t.min_radius > 200.0 && t.min_radius < 250.0);
  sim_track_free(&t);

  CHECK(sim_track_build(&t, "gaps"));
  size_t unpainted = 0;
  for (size_t i = 0; i < t.n; ++i) {
    if (!t.painted[i]) {
      ++unpainted;
    }
  }
  CHECK(unpainted >= 14 && unpainted <= 18);  // 30 mm + 50 mm at 5 mm spacing
  sim_track_free(&t);

  CHECK(sim_track_build(&t, "obstacle"));
  CHECK(t.n_obstacles == 1);
  sim_track_free(&t);
}

static void test_rangefinder(void) {
  check_context = "rangefinder";
  // Falls monotonically from 10 cm out to 80 cm.
  for (double mm = 100; mm < 800; mm += 10) {
    CHECK(sim_rangefinder_adc(mm) > sim_rangefinder_adc(mm + 10));
  }
  // Closer than 10 cm it falls again: a very close object reads as far.
  CHECK(sim_rangefinder_adc(40) < sim_rangefinder_adc(100));
  // The default obstacle threshold sits between 15 and 20 cm.
  const nav_config cfg = nav_default_config();
  CHECK(sim_rangefinder_adc(150) > cfg.obstacle_adc);
  CHECK(sim_rangefinder_adc(200) < cfg.obstacle_adc);
}

static void test_line_sensors(void) {
  check_context = "line sensors";
  sim_track t;
  sim_track_build(&t, "oval");  // starts at the origin heading along +x
  const sim_params p = quiet();
  sim_world w;
  sim_world_init(&w, &t, &p);
  w.x = 300;
  nav_inputs in;
  sim_sense(&w, &in);
  CHECK(in.line[2] > 850 && in.line[1] < 100 && in.line[3] < 100);
  w.y = 14;  // robot 14 mm left of the line: it is under sensor 3
  sim_sense(&w, &in);
  CHECK(in.line[3] > 850 && in.line[2] < 100);
  w.y = 7;  // between sensors 2 and 3
  sim_sense(&w, &in);
  CHECK(in.line[2] > 500 && in.line[3] > 500 && abs(in.line[2] - in.line[3]) < 5);
  CHECK(in.distance < 100);  // nothing ahead
  sim_track_free(&t);
}

static void test_motors(void) {
  check_context = "motors";
  sim_track t;
  sim_track_build(&t, "oval");
  sim_params p = quiet();
  sim_world w;
  sim_world_init(&w, &t, &p);
  for (int i = 0; i < 100; ++i) {
    sim_drive(&w, (nav_motors){10, 10}, SIM_TICK_S);  // inside the deadband
  }
  CHECK(fabs(w.x) < 1e-9 && fabs(w.y) < 1e-9);
  for (int i = 0; i < 100; ++i) {
    sim_drive(&w, (nav_motors){200, 200}, SIM_TICK_S);
  }
  CHECK(w.x > 300 && w.x < 400 && fabs(w.heading) < 1e-9);
  CHECK(w.progress > 300 && w.progress < 400);

  p.left_gain = 0.7;  // weak left motor: drifts left
  sim_world_init(&w, &t, &p);
  for (int i = 0; i < 100; ++i) {
    sim_drive(&w, (nav_motors){200, 200}, SIM_TICK_S);
  }
  CHECK(w.heading > 0.5);
  sim_track_free(&t);
}

static sim_result drive(const char* track, const sim_params* p, const nav_config* cfg,
                        unsigned laps) {
  sim_track t;
  sim_track_build(&t, track);
  nav n;
  nav_init(&n, cfg);
  const sim_run_options opt = {.laps = laps};
  const sim_result r = sim_run(&t, p, &opt, sim_nav_controller, &n);
  sim_track_free(&t);
  return r;
}

static void test_closed_loop(void) {
  const nav_config cfg = nav_default_config();
  for (const char* const* name = sim_track_names; *name; ++name) {
    check_context = *name;
    for (uint64_t seed = 1; seed <= SEEDS; ++seed) {
      sim_params p = sim_default_params();
      p.seed = seed;
      const sim_result r = drive(*name, &p, &cfg, 2);
      CHECK(r.completed && !r.collided);
      CHECK(r.max_xte_mm < 15.0);
      if (strcmp(*name, "gaps") == 0) {
        CHECK(r.searches >= 4);  // two gaps a lap
      } else if (strcmp(*name, "obstacle") == 0) {
        CHECK(r.avoids == 2 && r.min_clearance_mm > 50.0);
      } else {
        CHECK(r.searches == 0 && r.avoids == 0);
      }
    }
  }
}

static void test_faults(void) {
  const nav_config cfg = nav_default_config();
  sim_params p = sim_default_params();
  check_context = "weak motor";
  p.left_gain = 0.7;
  CHECK(drive("lobes", &p, &cfg, 2).completed);
  p = sim_default_params();
  p.right_gain = 0.6;
  CHECK(drive("lobes", &p, &cfg, 2).completed);

  check_context = "stuck-dark outer sensor";
  for (uint64_t seed = 1; seed <= SEEDS; ++seed) {
    p = sim_default_params();
    p.seed = seed;
    p.dead_sensor = 4;
    p.dead_value = NAV_ADC_MAX;
    CHECK(drive("lobes", &p, &cfg, 2).completed);
  }

  check_context = "dead (white) centre sensor";
  p = sim_default_params();
  p.dead_sensor = 2;
  p.dead_value = 80;
  CHECK(drive("lobes", &p, &cfg, 2).completed);
}

static void test_without_avoidance_it_crashes(void) {
  check_context = "negative control";
  nav_config cfg = nav_default_config();
  cfg.obstacle_adc = NAV_ADC_MAX + 1;
  const sim_params p = sim_default_params();
  const sim_result r = drive("obstacle", &p, &cfg, 1);
  CHECK(r.collided && !r.completed);
}

static void test_deterministic(void) {
  check_context = "determinism";
  const nav_config cfg = nav_default_config();
  sim_params p = sim_default_params();
  const sim_result a = drive("lobes", &p, &cfg, 1);
  const sim_result b = drive("lobes", &p, &cfg, 1);
  CHECK(a.ticks == b.ticks && a.rms_xte_mm == b.rms_xte_mm);
  p.seed = 2;
  const sim_result c = drive("lobes", &p, &cfg, 1);
  CHECK(c.rms_xte_mm != a.rms_xte_mm);
}

int main(void) {
  test_tracks();
  test_rangefinder();
  test_line_sensors();
  test_motors();
  test_closed_loop();
  test_faults();
  test_without_avoidance_it_crashes();
  test_deterministic();
  return check_exit("sim_test");
}
