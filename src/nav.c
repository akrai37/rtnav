#include "rtnav/nav.h"

#include <string.h>

// Sensor weights for the position average: -2000, -1000, 0, 1000, 2000.
static const int16_t kWeight[NAV_LINE_SENSORS] = {-2000, -1000, 0, 1000, 2000};

nav_config nav_default_config(void) {
  nav_config c;
  memset(&c, 0, sizeof c);
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    c.white[i] = 80;
    c.black[i] = 900;
  }
  c.base_speed = 150;
  c.kp_q8 = 18;             // 0.07 duty per position unit: full deflection (2000) ~ 141
  c.kd_q8 = 160;            // reacts to the change in position between ticks
  c.line_lost = 200;        // summed normalised reflectance, out of 5000
  c.obstacle_adc = 300;     // ~17 cm on a GP2Y0A21-style rangefinder
  c.gap_ticks = 60;         // 0.6 s
  c.search_ticks = 300;     // 3 s
  c.avoid_turn_ticks = 80;  // ~90 degrees in place at half speed
  c.avoid_arc_ticks = 100;
  c.avoid_max_ticks = 800;
  c.avoid_outer = 150;  // ~300 mm radius: centred near an obstacle detected ~17 cm away
  c.avoid_inner = 105;
  c.rejoin_cross_ticks = 45;  // brings the axle, 70 mm behind the sensors, over the line
  c.rejoin_max_ticks = 300;
  c.fault_ticks = 50;  // 0.5 s
  return c;
}

void nav_init(nav* n, const nav_config* cfg) {
  memset(n, 0, sizeof *n);
  n->cfg = *cfg;
  n->state = NAV_FOLLOW;
  n->last_side = 1;
}

const char* nav_state_name(nav_state state) {
  switch (state) {
    case NAV_FOLLOW:
      return "follow";
    case NAV_SEARCH:
      return "search";
    case NAV_AVOID:
      return "avoid";
    case NAV_REJOIN:
      return "rejoin";
    case NAV_STOP:
      return "stop";
  }
  return "?";
}

uint16_t nav_normalise(const nav_config* cfg, unsigned i, uint16_t raw) {
  const uint16_t lo = cfg->white[i];
  const uint16_t hi = cfg->black[i];
  if (hi <= lo || raw <= lo) {
    return 0;
  }
  if (raw >= hi) {
    return 1000;
  }
  return (uint16_t)((uint32_t)(raw - lo) * 1000u / (uint32_t)(hi - lo));
}

// Normalised reflectance at or above this counts as dark (part of a run).
#define DARK 100

static int32_t distance(int32_t a, int32_t b) {
  return a > b ? a - b : b - a;
}

nav_fix nav_locate(const nav_config* cfg, const nav_inputs* in, uint8_t masked, int32_t near) {
  uint16_t v[NAV_LINE_SENSORS];
  nav_fix fix = {0, false, 0, 0};
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    v[i] = (masked & (1u << i)) ? 0 : nav_normalise(cfg, i, in->line[i]);
    if (v[i] >= DARK) {
      fix.dark |= (uint8_t)(1u << i);
    }
  }
  int32_t best_total = 0;
  int32_t best_distance = INT32_MAX;
  unsigned i = 0;
  while (i < NAV_LINE_SENSORS) {
    if (v[i] < DARK) {
      ++i;
      continue;
    }
    int32_t weighted = 0;
    int32_t total = 0;
    uint8_t run = 0;
    for (; i < NAV_LINE_SENSORS && v[i] >= DARK; ++i) {
      weighted += (int32_t)v[i] * kWeight[i];
      total += v[i];
      run |= (uint8_t)(1u << i);
    }
    const int32_t position = weighted / total;
    if (distance(position, near) < best_distance) {
      best_distance = distance(position, near);
      best_total = total;
      fix.position = position;
      fix.run = run;
    }
  }
  fix.found = best_total >= (int32_t)cfg->line_lost;
  if (!fix.found) {
    fix.position = 0;
  }
  return fix;
}

// A sensor that stays dark while apart from the line being followed is
// stuck (shorted, or blocked by debris) and is ignored from then on.
static void watch_faults(nav* n, const nav_fix* fix) {
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    const uint8_t bit = (uint8_t)(1u << i);
    if (!fix->found || !(fix->dark & bit) || (fix->run & bit)) {
      n->suspect[i] = 0;
    } else if (++n->suspect[i] >= n->cfg.fault_ticks) {
      n->masked |= bit;
    }
  }
}

static int16_t clamp_duty(int32_t v) {
  if (v > NAV_MOTOR_MAX) {
    return NAV_MOTOR_MAX;
  }
  if (v < -NAV_MOTOR_MAX) {
    return -NAV_MOTOR_MAX;
  }
  return (int16_t)v;
}

static void enter(nav* n, nav_state state) {
  n->state = state;
  n->ticks_in_state = 0;
}

// PD steering: a positive position (line to the right) slows the right
// motor and speeds up the left one, turning the robot right toward it.
static nav_motors follow(nav* n, int32_t position) {
  const int32_t derivative = position - n->last_position;
  const int32_t correction =
      ((int32_t)n->cfg.kp_q8 * position + (int32_t)n->cfg.kd_q8 * derivative) / 256;
  n->last_position = position;
  if (position > 300) {
    n->last_side = 1;
  } else if (position < -300) {
    n->last_side = -1;
  }
  return (nav_motors){clamp_duty(n->cfg.base_speed + correction),
                      clamp_duty(n->cfg.base_speed - correction)};
}

static nav_motors search(const nav* n, int16_t slow) {
  if (n->bridging && n->ticks_in_state < n->cfg.gap_ticks) {
    return (nav_motors){slow, slow};
  }
  // Spin in place toward where the line was last seen.
  return (nav_motors){(int16_t)(slow * n->last_side), (int16_t)(-slow * n->last_side)};
}

nav_motors nav_step(nav* n, const nav_inputs* in) {
  ++n->ticks;
  if (n->ticks_in_state < UINT16_MAX) {
    ++n->ticks_in_state;
  }
  const nav_fix fix = nav_locate(&n->cfg, in, n->masked, n->last_position);
  watch_faults(n, &fix);
  const bool found = fix.found;
  const int32_t position = fix.position;
  const bool obstacle = in->distance >= n->cfg.obstacle_adc;
  const int16_t base = n->cfg.base_speed;
  const int16_t slow = (int16_t)(base / 2);

  switch (n->state) {
    case NAV_FOLLOW:
      if (obstacle) {
        enter(n, NAV_AVOID);
        return (nav_motors){0, 0};
      }
      if (!found) {
        enter(n, NAV_SEARCH);
        n->bridging = n->last_position > -1000 && n->last_position < 1000;
        return search(n, slow);
      }
      return follow(n, position);

    case NAV_SEARCH:
      if (found) {
        enter(n, NAV_FOLLOW);
        n->last_position = position;
        return follow(n, position);
      }
      if (n->ticks_in_state >= n->cfg.search_ticks) {
        enter(n, NAV_STOP);
        return (nav_motors){0, 0};
      }
      return search(n, slow);

    case NAV_AVOID: {
      const uint16_t t = n->ticks_in_state;
      if (t >= n->cfg.avoid_max_ticks) {
        enter(n, NAV_STOP);
        return (nav_motors){0, 0};
      }
      if (t < n->cfg.avoid_turn_ticks) {
        return (nav_motors){slow, (int16_t)-slow};  // turn right, away from the obstacle
      }
      if (t < n->cfg.avoid_turn_ticks + n->cfg.avoid_arc_ticks || !found) {
        // Arc left around the obstacle; after the minimum arc, keep arcing
        // until the line is under the sensors again.
        return (nav_motors){n->cfg.avoid_inner, n->cfg.avoid_outer};
      }
      enter(n, NAV_REJOIN);
      return (nav_motors){slow, slow};
    }

    case NAV_REJOIN: {
      const uint16_t t = n->ticks_in_state;
      if (t >= n->cfg.rejoin_max_ticks) {
        enter(n, NAV_STOP);
        return (nav_motors){0, 0};
      }
      if (t < n->cfg.rejoin_cross_ticks) {
        return (nav_motors){slow, slow};
      }
      if (found && position > -500 && position < 500) {
        enter(n, NAV_FOLLOW);
        n->last_position = position;
        return follow(n, position);
      }
      // Passing on the right means the line continues to the right.
      return (nav_motors){slow, (int16_t)-slow};
    }

    case NAV_STOP:
      return (nav_motors){0, 0};
  }
  return (nav_motors){0, 0};
}
