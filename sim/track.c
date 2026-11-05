// Built-in tracks. Each is drawn as a dense closed polyline, then resampled
// to evenly spaced centreline points.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"

#define SPACING_MM 5.0
#define LINE_WIDTH_MM 19.0  // 3/4" electrical tape
#define OBSTACLE_RADIUS_MM 50.0

static const double kPi = 3.14159265358979323846;

const char* const sim_track_names[] = {"oval", "lobes", "gaps", "obstacle", NULL};

typedef struct {
  sim_vec* p;
  size_t n;
  size_t cap;
  double x, y, heading;
} pen;

static void pen_point(pen* d, double x, double y) {
  if (d->n == d->cap) {
    d->cap = d->cap ? d->cap * 2 : 4096;
    d->p = realloc(d->p, d->cap * sizeof *d->p);
    if (!d->p) {
      abort();
    }
  }
  d->p[d->n++] = (sim_vec){x, y};
  d->x = x;
  d->y = y;
}

static void pen_straight(pen* d, double len) {
  const unsigned steps = (unsigned)lround(len);
  for (unsigned i = 1; i <= steps; ++i) {
    pen_point(d, d->x + cos(d->heading) * len / steps, d->y + sin(d->heading) * len / steps);
  }
}

// Arc of `radius`, turning by `angle` radians (positive: left). Each step
// is a chord, so the points lie exactly on the circle.
static void pen_arc(pen* d, double radius, double angle) {
  const unsigned steps = (unsigned)lround(radius * fabs(angle));
  const double dh = angle / steps;
  const double chord = 2.0 * radius * sin(fabs(dh) / 2.0);
  for (unsigned i = 0; i < steps; ++i) {
    d->heading += dh / 2.0;
    pen_point(d, d->x + cos(d->heading) * chord, d->y + sin(d->heading) * chord);
    d->heading += dh / 2.0;
  }
}

static double dist(sim_vec a, sim_vec b) {
  return hypot(a.x - b.x, a.y - b.y);
}

// Evenly spaced points along the closed polyline p[0..n).
static void resample(sim_track* t, const sim_vec* p, size_t n) {
  double total = 0;
  for (size_t i = 0; i < n; ++i) {
    total += dist(p[i], p[(i + 1) % n]);
  }
  const size_t m = (size_t)lround(total / SPACING_MM);
  const double step = total / (double)m;
  t->pts = malloc(m * sizeof *t->pts);
  t->painted = malloc(m * sizeof *t->painted);
  if (!t->pts || !t->painted) {
    abort();
  }
  size_t seg = 0;
  double seg_start = 0;
  double seg_len = dist(p[0], p[1]);
  for (size_t k = 0; k < m; ++k) {
    const double target = (double)k * step;
    while (seg_start + seg_len < target && seg + 1 < n) {
      seg_start += seg_len;
      ++seg;
      seg_len = dist(p[seg], p[(seg + 1) % n]);
    }
    const double f = seg_len > 0 ? (target - seg_start) / seg_len : 0;
    const sim_vec a = p[seg];
    const sim_vec b = p[(seg + 1) % n];
    t->pts[k] = (sim_vec){a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f};
    t->painted[k] = true;
  }
  t->n = m;
  t->spacing = step;
  t->length = total;
}

// Tightest curve, from the circle through points 20 mm either side.
static double min_radius(const sim_track* t) {
  const size_t k = 4;
  double best = HUGE_VAL;
  for (size_t i = 0; i < t->n; ++i) {
    const sim_vec a = t->pts[(i + t->n - k) % t->n];
    const sim_vec b = t->pts[i];
    const sim_vec c = t->pts[(i + k) % t->n];
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (fabs(cross) < 1e-9) {
      continue;
    }
    const double r = dist(a, b) * dist(b, c) * dist(c, a) / (2.0 * fabs(cross));
    if (r < best) {
      best = r;
    }
  }
  return best;
}

static void unpaint(sim_track* t, double from_mm, double to_mm) {
  for (size_t i = 0; i < t->n; ++i) {
    const double s = (double)i * t->spacing;
    if (s >= from_mm && s < to_mm) {
      t->painted[i] = false;
    }
  }
}

static void add_obstacle(sim_track* t, double at_mm) {
  const size_t i = (size_t)lround(at_mm / t->spacing) % t->n;
  t->obstacles[t->n_obstacles++] = (sim_obstacle){t->pts[i], OBSTACLE_RADIUS_MM};
}

// 1 m straights joined by 300 mm-radius half circles, driven anticlockwise.
static void draw_oval(pen* d) {
  pen_point(d, 0, 0);
  pen_straight(d, 1000);
  pen_arc(d, 300, kPi);
  pen_straight(d, 1000);
  pen_arc(d, 300, kPi);
}

// r = 600 + 150 sin(3 theta): three bulges with inward S-bends between
// them; the tightest curve is about 225 mm.
static void draw_lobes(pen* d) {
  const unsigned n = 20000;
  for (unsigned i = 0; i < n; ++i) {
    const double th = 2.0 * kPi * i / n;
    const double r = 600.0 + 150.0 * sin(3.0 * th);
    pen_point(d, r * cos(th), r * sin(th));
  }
}

bool sim_track_build(sim_track* t, const char* name) {
  memset(t, 0, sizeof *t);
  pen d = {0};
  const char* const* known = sim_track_names;
  while (*known && strcmp(*known, name) != 0) {
    ++known;
  }
  if (!*known) {
    return false;
  }
  t->name = *known;
  if (strcmp(name, "lobes") == 0) {
    draw_lobes(&d);
  } else {
    draw_oval(&d);
  }
  // Drop a closing point that duplicates the first.
  if (d.n > 1 && dist(d.p[0], d.p[d.n - 1]) < 0.5) {
    --d.n;
  }
  resample(t, d.p, d.n);
  free(d.p);
  t->line_width = LINE_WIDTH_MM;
  t->min_radius = min_radius(t);

  // The oval's second straight runs from about 1942 mm to 2942 mm.
  if (strcmp(name, "gaps") == 0) {
    unpaint(t, 450, 480);    // 30 mm gap on the first straight
    unpaint(t, 2400, 2450);  // 50 mm gap on the second
  } else if (strcmp(name, "obstacle") == 0) {
    add_obstacle(t, 2442);  // centred on the line, mid second straight
  }
  return true;
}

void sim_track_free(sim_track* t) {
  free(t->pts);
  free(t->painted);
  memset(t, 0, sizeof *t);
}
