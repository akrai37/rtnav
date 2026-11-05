// The navigation logic on its own: sensor maths, steering direction and
// limits, every state transition, and determinism.

#include "rtnav/nav.h"

#include "check.h"

static nav_config cfg;

// Inputs with the line under a single sensor (index 0..4), or -1 for none.
static nav_inputs line_under(int sensor) {
  nav_inputs in;
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    in.line[i] = (int)i == sensor ? cfg.black[i] : cfg.white[i];
  }
  in.distance = 100;  // nothing ahead
  return in;
}

static void test_normalise(void) {
  check_context = "normalise";
  CHECK_EQ_U(nav_normalise(&cfg, 0, 0), 0);
  CHECK_EQ_U(nav_normalise(&cfg, 0, cfg.white[0]), 0);
  CHECK_EQ_U(nav_normalise(&cfg, 0, cfg.black[0]), 1000);
  CHECK_EQ_U(nav_normalise(&cfg, 0, NAV_ADC_MAX), 1000);
  const uint16_t mid = (uint16_t)((cfg.white[0] + cfg.black[0]) / 2);
  CHECK(nav_normalise(&cfg, 0, mid) >= 499 && nav_normalise(&cfg, 0, mid) <= 501);
  // A sensor with a broken calibration (black <= white) reads as white.
  nav_config bad = cfg;
  bad.black[2] = bad.white[2];
  CHECK_EQ_U(nav_normalise(&bad, 2, 1000), 0);
}

static void test_line_position(void) {
  check_context = "position";
  nav_inputs in = line_under(2);
  nav_fix f = nav_locate(&cfg, &in, 0, 0);
  CHECK(f.found && f.position == 0 && f.run == 1u << 2);
  in = line_under(0);
  f = nav_locate(&cfg, &in, 0, 0);
  CHECK(f.found && f.position == -2000);
  in = line_under(4);
  f = nav_locate(&cfg, &in, 0, 0);
  CHECK(f.found && f.position == 2000);
  // Halfway between sensors 2 and 3: both half-covered.
  in = line_under(-1);
  in.line[2] = (uint16_t)((cfg.white[2] + cfg.black[2]) / 2);
  in.line[3] = (uint16_t)((cfg.white[3] + cfg.black[3]) / 2);
  f = nav_locate(&cfg, &in, 0, 0);
  CHECK(f.found && f.position >= 495 && f.position <= 505 && f.run == ((1u << 2) | (1u << 3)));
  in = line_under(-1);
  f = nav_locate(&cfg, &in, 0, 0);
  CHECK(!f.found && f.dark == 0);
}

static void test_separate_runs(void) {
  check_context = "runs";
  // Line under sensor 1 and something dark under sensor 4: two runs. The
  // one nearer the previous position wins; masking removes the other.
  nav_inputs in = line_under(1);
  in.line[4] = cfg.black[4];
  nav_fix f = nav_locate(&cfg, &in, 0, -1000);
  CHECK(f.found && f.position == -1000 && f.dark == ((1u << 1) | (1u << 4)));
  f = nav_locate(&cfg, &in, 0, 1500);
  CHECK(f.position == 2000);
  f = nav_locate(&cfg, &in, 1u << 1, -1000);
  CHECK(f.position == 2000 && f.run == 1u << 4);
}

static void test_fault_masking(void) {
  check_context = "fault";
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(2);
  in.line[4] = NAV_ADC_MAX;  // stuck dark
  for (unsigned t = 1; t < cfg.fault_ticks; ++t) {
    nav_step(&n, &in);
  }
  CHECK(n.masked == 0);
  nav_step(&n, &in);
  CHECK(n.masked == 1u << 4);
  // A dark sensor that is part of the followed run is never suspect.
  nav_init(&n, &cfg);
  in = line_under(2);
  in.line[3] = cfg.black[3];
  for (unsigned t = 0; t < 2u * cfg.fault_ticks; ++t) {
    nav_step(&n, &in);
  }
  CHECK(n.masked == 0);
}

static void test_steering(void) {
  check_context = "steering";
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(2);
  nav_motors m = nav_step(&n, &in);
  CHECK(m.left == cfg.base_speed && m.right == cfg.base_speed);  // centred: straight

  nav_init(&n, &cfg);
  in = line_under(4);  // line far right: turn right (left faster)
  m = nav_step(&n, &in);
  CHECK(m.left > cfg.base_speed && m.right < cfg.base_speed);

  nav_init(&n, &cfg);
  in = line_under(0);  // far left: turn left
  m = nav_step(&n, &in);
  CHECK(m.left < m.right);

  // Commands never exceed the PWM range, even with absurd gains.
  nav_config hot = cfg;
  hot.kp_q8 = 32000;
  hot.kd_q8 = 32000;
  nav_init(&n, &hot);
  in = line_under(4);
  m = nav_step(&n, &in);
  CHECK(m.left <= NAV_MOTOR_MAX && m.right >= -NAV_MOTOR_MAX);
  in = line_under(0);
  m = nav_step(&n, &in);
  CHECK(m.left >= -NAV_MOTOR_MAX && m.right <= NAV_MOTOR_MAX);
}

static void test_derivative_damps(void) {
  check_context = "derivative";
  // Moving toward the centre, the D term opposes the P term.
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(4);
  nav_step(&n, &in);
  in = line_under(3);  // still right of centre but approaching
  const nav_motors approaching = nav_step(&n, &in);
  nav_init(&n, &cfg);
  nav_step(&n, &in);  // same position, no motion
  nav_step(&n, &in);
  const nav_motors steady = nav_step(&n, &in);
  CHECK(approaching.left - approaching.right < steady.left - steady.right);
}

static void test_search_and_recover(void) {
  check_context = "search";
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(4);  // last seen to the right
  nav_step(&n, &in);
  in = line_under(-1);
  nav_motors m = nav_step(&n, &in);
  CHECK(n.state == NAV_SEARCH);
  CHECK(m.left > 0 && m.right < 0);  // spin right, toward where it was
  in = line_under(2);
  nav_step(&n, &in);
  CHECK(n.state == NAV_FOLLOW);
}

static void test_gap_bridging(void) {
  check_context = "gap";
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(2);  // centred when the line vanishes: a gap
  nav_step(&n, &in);
  in = line_under(-1);
  nav_motors m = nav_step(&n, &in);
  CHECK(n.state == NAV_SEARCH && m.left > 0 && m.left == m.right);
  for (unsigned t = 1; t < cfg.gap_ticks; ++t) {
    m = nav_step(&n, &in);
  }
  CHECK(m.left > 0 && m.left == m.right);  // still straight at the end of the window
  m = nav_step(&n, &in);
  CHECK(m.left == -m.right && m.left != 0);  // then spins
}

static void test_search_gives_up(void) {
  check_context = "search timeout";
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(-1);
  for (unsigned t = 0; t <= cfg.search_ticks + 1u; ++t) {
    nav_step(&n, &in);
  }
  CHECK(n.state == NAV_STOP);
  const nav_motors m = nav_step(&n, &in);
  CHECK(m.left == 0 && m.right == 0);
  in = line_under(2);  // STOP is final, even if the line reappears
  nav_step(&n, &in);
  CHECK(n.state == NAV_STOP);
}

static void test_avoidance_sequence(void) {
  check_context = "avoid";
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(2);
  in.distance = cfg.obstacle_adc;
  nav_motors m = nav_step(&n, &in);
  CHECK(n.state == NAV_AVOID);
  CHECK(m.left == 0 && m.right == 0);  // stop first
  in = line_under(-1);
  m = nav_step(&n, &in);
  CHECK(m.left > 0 && m.right < 0);  // turn right, away
  for (unsigned t = 1; t < cfg.avoid_turn_ticks; ++t) {
    nav_step(&n, &in);
  }
  m = nav_step(&n, &in);
  CHECK(m.left > 0 && m.right > m.left);  // arc left around it
  // Line under the sensors during the arc does not end it early.
  in = line_under(2);
  nav_step(&n, &in);
  CHECK(n.state == NAV_AVOID);
  for (unsigned t = 0; t < cfg.avoid_arc_ticks && n.state == NAV_AVOID; ++t) {
    nav_step(&n, &in);
  }
  CHECK(n.state == NAV_REJOIN);  // arc done and line found
  // Drives on over the line, then spins right until it is centred.
  in = line_under(-1);
  for (unsigned t = 1; t < cfg.rejoin_cross_ticks; ++t) {
    m = nav_step(&n, &in);
    CHECK(m.left > 0 && m.left == m.right);
  }
  m = nav_step(&n, &in);
  CHECK(n.state == NAV_REJOIN && m.left > 0 && m.right == -m.left);
  in = line_under(4);  // seen, but not yet centred
  nav_step(&n, &in);
  CHECK(n.state == NAV_REJOIN);
  in = line_under(2);
  nav_step(&n, &in);
  CHECK(n.state == NAV_FOLLOW);
}

// Into AVOID, then through the arc onto the line: ends in REJOIN.
static void reach_rejoin(nav* n) {
  nav_init(n, &cfg);
  nav_inputs in = line_under(2);
  in.distance = cfg.obstacle_adc;
  nav_step(n, &in);
  in = line_under(-1);
  for (unsigned t = 0; t < (unsigned)cfg.avoid_turn_ticks + cfg.avoid_arc_ticks; ++t) {
    nav_step(n, &in);
  }
  in = line_under(2);
  nav_step(n, &in);
}

static void test_avoidance_gives_up(void) {
  check_context = "avoid timeout";
  nav n;
  nav_init(&n, &cfg);
  nav_inputs in = line_under(2);
  in.distance = cfg.obstacle_adc;
  nav_step(&n, &in);
  in = line_under(-1);  // never finds the line again
  for (unsigned t = 0; t <= cfg.avoid_max_ticks; ++t) {
    nav_step(&n, &in);
  }
  CHECK(n.state == NAV_STOP);

  check_context = "rejoin timeout";
  reach_rejoin(&n);
  CHECK(n.state == NAV_REJOIN);
  for (unsigned t = 0; t <= cfg.rejoin_max_ticks; ++t) {
    nav_step(&n, &in);  // line never centred
  }
  CHECK(n.state == NAV_STOP);
}

static void test_deterministic(void) {
  check_context = "determinism";
  nav a;
  nav b;
  nav_init(&a, &cfg);
  nav_init(&b, &cfg);
  for (unsigned t = 0; t < 2000; ++t) {
    nav_inputs in = line_under((int)(t / 37 % 7) - 1);  // includes -1 (no line)
    in.distance = (uint16_t)(t % 997 == 0 ? 600 : 100);
    const nav_motors ma = nav_step(&a, &in);
    const nav_motors mb = nav_step(&b, &in);
    CHECK(ma.left == mb.left && ma.right == mb.right);
  }
  CHECK(a.state == b.state && a.ticks == 2000);
}

int main(void) {
  cfg = nav_default_config();
  test_normalise();
  test_line_position();
  test_separate_runs();
  test_fault_masking();
  test_steering();
  test_derivative_damps();
  test_search_and_recover();
  test_gap_bridging();
  test_search_gives_up();
  test_avoidance_sequence();
  test_avoidance_gives_up();
  test_deterministic();
  return check_exit("nav_test");
}
