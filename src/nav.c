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
  c.kp_q8 = 18;       // 0.07 duty per position unit: full deflection (2000) ~ 141
  c.kd_q8 = 160;      // reacts to the change in position between ticks
  c.line_lost = 200;  // summed normalised reflectance, out of 5000
  c.obstacle_adc = 450;
  c.search_ticks = 300;     // 3 s
  c.avoid_turn_ticks = 45;  // 0.45 s
  c.avoid_arc_ticks = 120;  // 1.2 s
  c.avoid_max_ticks = 600;  // 6 s
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

int32_t nav_line_position(const nav_config* cfg, const nav_inputs* in, bool* found) {
  int32_t weighted = 0;
  int32_t total = 0;
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    const int32_t v = nav_normalise(cfg, i, in->line[i]);
    weighted += v * kWeight[i];
    total += v;
  }
  *found = total >= (int32_t)cfg->line_lost;
  return *found ? weighted / total : 0;
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

nav_motors nav_step(nav* n, const nav_inputs* in) {
  ++n->ticks;
  if (n->ticks_in_state < UINT16_MAX) {
    ++n->ticks_in_state;
  }
  bool found;
  const int32_t position = nav_line_position(&n->cfg, in, &found);
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
        return (nav_motors){(int16_t)(slow * n->last_side), (int16_t)(-slow * n->last_side)};
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
      // Spin in place toward where the line was last seen.
      return (nav_motors){(int16_t)(slow * n->last_side), (int16_t)(-slow * n->last_side)};

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
        // Arc left around the obstacle; after the arc, keep arcing until
        // the line is under the sensors again.
        return (nav_motors){slow, base};
      }
      enter(n, NAV_FOLLOW);
      n->last_position = position;
      return follow(n, position);
    }

    case NAV_STOP:
      return (nav_motors){0, 0};
  }
  return (nav_motors){0, 0};
}
