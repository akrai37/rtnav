#include "sim.h"

#include <math.h>
#include <string.h>

// A small two-wheeled chassis. Positions are mm from the axle centre.
#define WHEELBASE_MM 120.0
#define SENSOR_AHEAD_MM 70.0  // line sensor bar
#define SENSOR_PITCH_MM 14.0
#define SPOT_RADIUS_MM 4.0  // each reflectance sensor sees a small spot
#define RANGER_AHEAD_MM 75.0
#define RANGER_MAX_MM 800.0
#define BODY_AHEAD_MM 20.0  // body modelled as a circle
#define BODY_RADIUS_MM 75.0
#define VMAX_MM_S 500.0  // wheel speed at full duty
#define DEADBAND 15.0    // smaller duties do not turn the wheel
#define MOTOR_TAU_S 0.06
#define PHYSICS_DT_S 0.001
#define NOMINAL_WHITE 80.0
#define NOMINAL_BLACK 900.0
#define TRACK_WINDOW 40  // centreline points searched either side when tracking progress

// xorshift64*: small, fast and the same on every platform.
static uint64_t next_u64(uint64_t* s) {
  uint64_t x = *s;
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  *s = x;
  return x * 0x2545F4914F6CDD1DULL;
}

static double uniform(uint64_t* s) {
  return (double)(next_u64(s) >> 11) * 0x1.0p-53;
}

static double gaussian(uint64_t* s) {
  double u1 = uniform(s);
  if (u1 < 1e-300) {
    u1 = 1e-300;
  }
  const double u2 = uniform(s);
  return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static double clamp(double v, double lo, double hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

static uint16_t to_adc(double v) {
  return (uint16_t)lround(clamp(v, 0, NAV_ADC_MAX));
}

sim_params sim_default_params(void) {
  return (sim_params){
      .seed = 1,
      .line_noise = 8,
      .distance_noise = 6,
      .calibration_spread = 25,
      .left_gain = 1.0,
      .right_gain = 1.0,
      .dead_sensor = -1,
      .dead_value = 0,
  };
}

void sim_world_init(sim_world* w, const sim_track* t, const sim_params* p) {
  memset(w, 0, sizeof *w);
  w->track = t;
  w->p = *p;
  w->rng = p->seed * 0x9E3779B97F4A7C15ULL + 0x632BE59BD9B4E019ULL;
  if (w->rng == 0) {
    w->rng = 1;
  }
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    w->white[i] = NOMINAL_WHITE + p->calibration_spread * (2.0 * uniform(&w->rng) - 1.0);
    w->black[i] = NOMINAL_BLACK + p->calibration_spread * (2.0 * uniform(&w->rng) - 1.0);
  }
  w->x = t->pts[0].x;
  w->y = t->pts[0].y;
  w->heading = atan2(t->pts[1].y - t->pts[0].y, t->pts[1].x - t->pts[0].x);
}

// A point `fwd` ahead of the axle and `left` to its left.
static sim_vec body_point(const sim_world* w, double fwd, double left) {
  const double c = cos(w->heading);
  const double s = sin(w->heading);
  return (sim_vec){w->x + fwd * c - left * s, w->y + fwd * s + left * c};
}

static double segment_distance(sim_vec p, sim_vec a, sim_vec b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  const double t = len2 > 0 ? clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0, 1) : 0;
  return hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

static double line_distance(const sim_track* t, sim_vec p, bool painted_only) {
  double best = HUGE_VAL;
  for (size_t i = 0; i < t->n; ++i) {
    if (painted_only && !t->painted[i]) {
      continue;
    }
    const double d = segment_distance(p, t->pts[i], t->pts[(i + 1) % t->n]);
    if (d < best) {
      best = d;
    }
  }
  return best;
}

double sim_rangefinder_adc(double mm) {
  const double cm = mm / 10.0;
  // GP2Y0A21-like: about 2.3 V at 10 cm falling to 0.4 V at 80 cm; closer
  // than 10 cm the output drops again.
  const double volts = cm >= 10.0 ? 2.3 * pow(10.0 / cm, 0.83) : 0.8 + 0.15 * cm;
  return volts / 5.0 * NAV_ADC_MAX;
}

// Distance along the ray from `p` in direction (dx, dy) to the nearest
// obstacle, capped at the rangefinder's reach.
static double ray_distance(const sim_track* t, sim_vec p, double dx, double dy) {
  double best = RANGER_MAX_MM;
  for (size_t i = 0; i < t->n_obstacles; ++i) {
    const sim_obstacle* o = &t->obstacles[i];
    const double ox = o->c.x - p.x;
    const double oy = o->c.y - p.y;
    const double along = ox * dx + oy * dy;
    const double off2 = ox * ox + oy * oy - along * along;
    const double r2 = o->r * o->r;
    if (ox * ox + oy * oy <= r2) {
      return 0;
    }
    if (along <= 0 || off2 > r2) {
      continue;
    }
    const double hit = along - sqrt(r2 - off2);
    if (hit < best) {
      best = hit;
    }
  }
  return best;
}

void sim_sense(sim_world* w, nav_inputs* in) {
  const sim_track* t = w->track;
  const double half = t->line_width / 2.0;
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    // Sensor 0 is the leftmost.
    const sim_vec p = body_point(w, SENSOR_AHEAD_MM, (2.0 - i) * SENSOR_PITCH_MM);
    const double d = line_distance(t, p, true);
    const double cover = clamp((half + SPOT_RADIUS_MM - d) / (2.0 * SPOT_RADIUS_MM), 0, 1);
    double raw = w->white[i] + (w->black[i] - w->white[i]) * cover;
    raw += w->p.line_noise * gaussian(&w->rng);
    in->line[i] = (int)i == w->p.dead_sensor ? w->p.dead_value : to_adc(raw);
  }
  const sim_vec r = body_point(w, RANGER_AHEAD_MM, 0);
  const double mm = ray_distance(t, r, cos(w->heading), sin(w->heading));
  in->distance = to_adc(sim_rangefinder_adc(mm) + w->p.distance_noise * gaussian(&w->rng));
}

static double wheel_target(int16_t duty, double gain) {
  const double mag = fabs((double)duty);
  if (mag <= DEADBAND) {
    return 0;
  }
  const double v = VMAX_MM_S * gain * (mag - DEADBAND) / (NAV_MOTOR_MAX - DEADBAND);
  return duty < 0 ? -v : v;
}

static void track_progress(sim_world* w) {
  const sim_track* t = w->track;
  const ptrdiff_t n = (ptrdiff_t)t->n;
  const sim_vec axle = {w->x, w->y};
  ptrdiff_t best = (ptrdiff_t)w->nearest;
  double best_d = HUGE_VAL;
  for (ptrdiff_t k = -TRACK_WINDOW; k <= TRACK_WINDOW; ++k) {
    const ptrdiff_t i = (((ptrdiff_t)w->nearest + k) % n + n) % n;
    const sim_vec p = t->pts[i];
    const double d = hypot(p.x - axle.x, p.y - axle.y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  ptrdiff_t delta = best - (ptrdiff_t)w->nearest;
  if (delta > n / 2) {
    delta -= n;
  } else if (delta < -n / 2) {
    delta += n;
  }
  w->progress += (double)delta * t->spacing;
  w->nearest = (size_t)best;
}

void sim_drive(sim_world* w, nav_motors cmd, double dt) {
  const double target_l = wheel_target(cmd.left, w->p.left_gain);
  const double target_r = wheel_target(cmd.right, w->p.right_gain);
  long steps = lround(dt / PHYSICS_DT_S);
  if (steps < 1) {
    steps = 1;
  }
  const double h = dt / (double)steps;
  for (long i = 0; i < steps; ++i) {
    w->v_left += (target_l - w->v_left) * h / MOTOR_TAU_S;
    w->v_right += (target_r - w->v_right) * h / MOTOR_TAU_S;
    const double v = (w->v_left + w->v_right) / 2.0;
    const double omega = (w->v_right - w->v_left) / WHEELBASE_MM;
    w->heading += omega * h / 2.0;
    w->x += v * cos(w->heading) * h;
    w->y += v * sin(w->heading) * h;
    w->heading += omega * h / 2.0;
  }
  w->time += dt;
  track_progress(w);
}

double sim_cross_track(const sim_world* w) {
  return line_distance(w->track, body_point(w, SENSOR_AHEAD_MM, 0), false);
}

double sim_clearance(const sim_world* w) {
  const sim_vec b = body_point(w, BODY_AHEAD_MM, 0);
  double best = 1e9;
  for (size_t i = 0; i < w->track->n_obstacles; ++i) {
    const sim_obstacle* o = &w->track->obstacles[i];
    const double gap = hypot(o->c.x - b.x, o->c.y - b.y) - o->r - BODY_RADIUS_MM;
    if (gap < best) {
      best = gap;
    }
  }
  return best;
}

nav_motors sim_nav_controller(void* ctx, const nav_inputs* in, nav_state* state) {
  nav* n = ctx;
  const nav_motors m = nav_step(n, in);
  *state = n->state;
  return m;
}

sim_result sim_run(const sim_track* t, const sim_params* p, const sim_run_options* opt,
                   sim_controller ctl, void* ctx) {
  sim_world w;
  sim_world_init(&w, t, p);
  const unsigned laps = opt->laps ? opt->laps : 1;
  const double distance = laps * t->length;
  const double timeout = opt->timeout_s > 0 ? opt->timeout_s : distance / 100.0 + 10.0;

  sim_result r;
  memset(&r, 0, sizeof r);
  r.min_clearance_mm = 1e9;
  nav_state prev = NAV_FOLLOW;
  double sum_sq = 0;
  unsigned following = 0;
  if (opt->trace) {
    fprintf(opt->trace, "t,x,y,heading,state,left,right,xte,distance_adc\n");
  }
  for (;;) {
    nav_inputs in;
    sim_sense(&w, &in);
    nav_state state;
    const nav_motors m = ctl(ctx, &in, &state);
    if (state != prev) {
      if (state == NAV_SEARCH) {
        ++r.searches;
      } else if (state == NAV_AVOID) {
        ++r.avoids;
      }
      prev = state;
    }
    sim_drive(&w, m, SIM_TICK_S);
    ++r.ticks;

    const double xte = sim_cross_track(&w);
    if (state == NAV_FOLLOW) {
      if (xte > r.max_xte_mm) {
        r.max_xte_mm = xte;
      }
      sum_sq += xte * xte;
      ++following;
    }
    const double clearance = sim_clearance(&w);
    if (clearance < r.min_clearance_mm) {
      r.min_clearance_mm = clearance;
    }
    if (opt->trace) {
      fprintf(opt->trace, "%.2f,%.1f,%.1f,%.4f,%s,%d,%d,%.1f,%u\n", w.time, w.x, w.y, w.heading,
              nav_state_name(state), m.left, m.right, xte, in.distance);
    }
    if (clearance < 0) {
      r.collided = true;
      break;
    }
    if (w.progress >= distance) {
      r.completed = true;
      break;
    }
    if (state == NAV_STOP || w.time >= timeout) {
      break;
    }
  }
  r.time_s = w.time;
  r.final_state = prev;
  r.rms_xte_mm = following ? sqrt(sum_sq / following) : 0;
  return r;
}
