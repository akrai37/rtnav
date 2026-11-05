// rtnav-sim: drive the navigation code around a simulated track and print
// the outcome as one line of key=value pairs.

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtnav/nav.h"
#include "sim.h"

static void usage(void) {
  fprintf(stderr,
          "usage: rtnav-sim [options]\n"
          "  --track NAME        oval, lobes, gaps, obstacle (default oval)\n"
          "  --laps N            laps to drive (default 1)\n"
          "  --seed N            noise and calibration seed (default 1)\n"
          "  --speed DUTY        base speed, 0..255 (default %d)\n"
          "  --noise COUNTS      line sensor noise, ADC counts sd (default 8)\n"
          "  --left-gain G       left motor strength, 1.0 nominal\n"
          "  --right-gain G      right motor strength\n"
          "  --dead-sensor I:V   line sensor I stuck at ADC value V\n"
          "  --no-avoid          disable obstacle detection\n"
          "  --trace FILE        per-tick CSV\n"
          "  --track-csv FILE    centreline and obstacles as CSV\n",
          nav_default_config().base_speed);
}

static double number(const char* s, const char* what) {
  char* end;
  errno = 0;
  const double v = strtod(s, &end);
  if (errno || end == s || *end) {
    fprintf(stderr, "rtnav-sim: bad %s '%s'\n", what, s);
    exit(2);
  }
  return v;
}

static void write_track(const sim_track* t, const char* path) {
  FILE* f = fopen(path, "w");
  if (!f) {
    perror(path);
    exit(1);
  }
  fprintf(f, "kind,x,y,r,painted\n");
  for (size_t i = 0; i < t->n; ++i) {
    fprintf(f, "line,%.1f,%.1f,0,%d\n", t->pts[i].x, t->pts[i].y, t->painted[i]);
  }
  for (size_t i = 0; i < t->n_obstacles; ++i) {
    fprintf(f, "obstacle,%.1f,%.1f,%.1f,0\n", t->obstacles[i].c.x, t->obstacles[i].c.y,
            t->obstacles[i].r);
  }
  fclose(f);
}

int main(int argc, char** argv) {
  const char* track_name = "oval";
  const char* trace_path = NULL;
  const char* track_path = NULL;
  sim_params p = sim_default_params();
  nav_config cfg = nav_default_config();
  sim_run_options opt = {.laps = 1};

  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    const char* v = i + 1 < argc ? argv[i + 1] : NULL;
    if (strcmp(a, "--no-avoid") == 0) {
      cfg.obstacle_adc = NAV_ADC_MAX + 1;
      continue;
    }
    if (strcmp(a, "--help") == 0 || !v) {
      usage();
      return strcmp(a, "--help") == 0 ? 0 : 2;
    }
    ++i;
    if (strcmp(a, "--track") == 0) {
      track_name = v;
    } else if (strcmp(a, "--laps") == 0) {
      opt.laps = (unsigned)number(v, "lap count");
    } else if (strcmp(a, "--seed") == 0) {
      p.seed = (uint64_t)number(v, "seed");
    } else if (strcmp(a, "--speed") == 0) {
      cfg.base_speed = (int16_t)number(v, "speed");
    } else if (strcmp(a, "--noise") == 0) {
      p.line_noise = number(v, "noise");
    } else if (strcmp(a, "--left-gain") == 0) {
      p.left_gain = number(v, "gain");
    } else if (strcmp(a, "--right-gain") == 0) {
      p.right_gain = number(v, "gain");
    } else if (strcmp(a, "--dead-sensor") == 0) {
      unsigned idx;
      unsigned val;
      if (sscanf(v, "%u:%u", &idx, &val) != 2 || idx >= NAV_LINE_SENSORS || val > NAV_ADC_MAX) {
        fprintf(stderr, "rtnav-sim: bad --dead-sensor '%s' (want INDEX:VALUE)\n", v);
        return 2;
      }
      p.dead_sensor = (int)idx;
      p.dead_value = (uint16_t)val;
    } else if (strcmp(a, "--trace") == 0) {
      trace_path = v;
    } else if (strcmp(a, "--track-csv") == 0) {
      track_path = v;
    } else {
      usage();
      return 2;
    }
  }

  sim_track t;
  if (!sim_track_build(&t, track_name)) {
    fprintf(stderr, "rtnav-sim: unknown track '%s'\n", track_name);
    return 2;
  }
  if (track_path) {
    write_track(&t, track_path);
  }
  if (trace_path) {
    opt.trace = fopen(trace_path, "w");
    if (!opt.trace) {
      perror(trace_path);
      return 1;
    }
  }

  nav n;
  nav_init(&n, &cfg);
  const sim_result r = sim_run(&t, &p, &opt, sim_nav_controller, &n);
  if (opt.trace) {
    fclose(opt.trace);
  }
  printf(
      "track=%s length_mm=%.0f min_radius_mm=%.0f laps=%u seed=%llu speed=%d completed=%s "
      "collided=%s time_s=%.2f max_xte_mm=%.1f rms_xte_mm=%.1f min_clearance_mm=%.1f "
      "searches=%u avoids=%u final_state=%s\n",
      t.name, t.length, t.min_radius, opt.laps, (unsigned long long)p.seed, cfg.base_speed,
      r.completed ? "yes" : "no", r.collided ? "yes" : "no", r.time_s, r.max_xte_mm, r.rms_xte_mm,
      t.n_obstacles ? r.min_clearance_mm : -1.0, r.searches, r.avoids,
      nav_state_name(r.final_state));
  sim_track_free(&t);
  return r.completed ? 0 : 1;
}
