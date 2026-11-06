// rtnav-fil: firmware in the loop. Runs the compiled firmware (rtnav.elf)
// on simavr's cycle-accurate ATmega328P, closed loop with the simulated
// robot from sim/:
//
//   - each control tick the robot's sensor readings are applied to the ADC
//     pins as voltages; the firmware samples them through its own ADC driver
//   - the motor command is read back from the PWM compare registers and the
//     H-bridge direction pins
//   - the debug pin brackets every control step, giving its duration and the
//     tick period in CPU cycles
//   - the sensor snapshot the firmware used is read from its RAM and fed to
//     the host build of the navigation code; both must command the same
//     duties on every tick (lockstep)
//   - free RAM is painted before boot, so the deepest stack use shows after
//
// Prints one line of key=value pairs. Exit status 0 only if the run
// completed with no lockstep mismatch, ADC read-back error or missed tick.

#include <fcntl.h>
#include <gelf.h>
#include <libelf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "avr_adc.h"
#include "avr_ioport.h"
#include "rtnav/nav.h"
#include "sim.h"
#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_io.h"

#define F_CPU_HZ 16000000u
#define TICK_CYCLES (F_CPU_HZ / 100u)
#define VREF_MV 5000u
#define SRAM_OFFSET 0x800000u  // AVR data addresses in the ELF
#define RAMEND 0x08FFu
#define PAINT 0xA5u
#define HANG_CYCLES (10u * TICK_CYCLES)  // no step for 100 ms: give up

// ATmega328P data-space addresses.
#define REG_PORTB 0x25u
#define REG_PORTD 0x2Bu
#define REG_TCCR0A 0x44u
#define REG_OCR0A 0x47u
#define REG_OCR0B 0x48u

// Layout of rtnav_debug_t in firmware/main.c (AVR structs are unpadded).
#define DBG_STEPS 0u
#define DBG_MISSED 4u
#define DBG_WORST_END 6u
#define DBG_STATE 9u
#define DBG_MASKED 10u
#define DBG_IN 11u

typedef struct {
  avr_t* avr;
  uint16_t dbg;  // data address of rtnav_debug
  bool in_step;
  bool step_done;
  avr_cycle_count_t rise;
  avr_cycle_count_t last_rise;
  avr_cycle_count_t last_progress;
  uint32_t* durations;
  size_t n;
  size_t cap;
  int64_t worst_period_dev;
  nav host;  // the same navigation code built for the host, in lockstep
  unsigned mismatches;
  unsigned adc_errors;
  bool hung;
} fil;

static void die(const char* msg) {
  fprintf(stderr, "rtnav-fil: %s\n", msg);
  exit(2);
}

static bool elf_symbol(const char* path, const char* name, uint32_t* value) {
  if (elf_version(EV_CURRENT) == EV_NONE) {
    return false;
  }
  const int fd = open(path, O_RDONLY);
  if (fd < 0) {
    return false;
  }
  Elf* e = elf_begin(fd, ELF_C_READ, NULL);
  bool found = false;
  Elf_Scn* scn = NULL;
  while (e && !found && (scn = elf_nextscn(e, scn)) != NULL) {
    GElf_Shdr sh;
    if (!gelf_getshdr(scn, &sh) || sh.sh_type != SHT_SYMTAB || sh.sh_entsize == 0) {
      continue;
    }
    Elf_Data* d = elf_getdata(scn, NULL);
    const size_t count = sh.sh_size / sh.sh_entsize;
    for (size_t i = 0; d && i < count; ++i) {
      GElf_Sym s;
      if (!gelf_getsym(d, (int)i, &s)) {
        continue;
      }
      const char* nm = elf_strptr(e, sh.sh_link, s.st_name);
      if (nm && strcmp(nm, name) == 0) {
        *value = (uint32_t)s.st_value;
        found = true;
        break;
      }
    }
  }
  if (e) {
    elf_end(e);
  }
  close(fd);
  return found;
}

static uint16_t rd16(const fil* f, uint16_t addr) {
  return (uint16_t)(f->avr->data[addr] | (f->avr->data[addr + 1u] << 8));
}

static uint32_t rd32(const fil* f, uint16_t addr) {
  return (uint32_t)rd16(f, addr) | ((uint32_t)rd16(f, (uint16_t)(addr + 2u)) << 16);
}

static void on_debug_pin(struct avr_irq_t* irq, uint32_t value, void* param) {
  (void)irq;
  fil* f = param;
  const avr_cycle_count_t now = f->avr->cycle;
  if (value) {
    if (f->last_rise) {
      int64_t dev = (int64_t)(now - f->last_rise) - (int64_t)TICK_CYCLES;
      if (dev < 0) {
        dev = -dev;
      }
      if (dev > f->worst_period_dev) {
        f->worst_period_dev = dev;
      }
    }
    f->last_rise = now;
    f->rise = now;
    f->in_step = true;
  } else if (f->in_step) {
    f->in_step = false;
    if (f->n == f->cap) {
      f->cap = f->cap ? f->cap * 2 : 4096;
      f->durations = realloc(f->durations, f->cap * sizeof *f->durations);
      if (!f->durations) {
        die("out of memory");
      }
    }
    f->durations[f->n++] = (uint32_t)(now - f->rise);
    f->step_done = true;
  }
}

// Millivolts that simavr's ADC converts back to exactly `counts`.
static uint32_t millivolts(uint16_t counts) {
  return ((uint32_t)counts * VREF_MV + 1022u) / 1023u;
}

static int16_t motor(const fil* f, bool on, uint8_t ocr, bool fwd, bool rev) {
  const int16_t duty = on ? (int16_t)f->avr->data[ocr] : 0;
  if (fwd && !rev) {
    return duty;
  }
  if (rev && !fwd) {
    return (int16_t)-duty;
  }
  return 0;
}

static nav_motors read_motors(const fil* f) {
  const uint8_t tccr = f->avr->data[REG_TCCR0A];
  const uint8_t pd = f->avr->data[REG_PORTD];
  const uint8_t pb = f->avr->data[REG_PORTB];
  return (nav_motors){
      motor(f, tccr & 0x80u, REG_OCR0A, pd & 0x80u, pb & 0x01u),  // COM0A1, PD7, PB0
      motor(f, tccr & 0x20u, REG_OCR0B, pd & 0x10u, pd & 0x04u),  // COM0B1, PD4, PD2
  };
}

static nav_motors fil_controller(void* ctx, const nav_inputs* in, nav_state* state) {
  fil* f = ctx;
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    avr_raise_irq(avr_io_getirq(f->avr, AVR_IOCTL_ADC_GETIRQ, (int)(ADC_IRQ_ADC0 + i)),
                  millivolts(in->line[i]));
  }
  avr_raise_irq(avr_io_getirq(f->avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC5),
                millivolts(in->distance));

  f->step_done = false;
  const avr_cycle_count_t start = f->avr->cycle;
  while (!f->step_done) {
    const int s = avr_run(f->avr);
    if (s == cpu_Done || s == cpu_Crashed) {
      die("firmware stopped or crashed");
    }
    if (f->avr->cycle - start > HANG_CYCLES) {
      f->hung = true;
      *state = NAV_STOP;
      return (nav_motors){0, 0};
    }
  }

  const nav_motors m = read_motors(f);
  nav_inputs seen;
  for (unsigned i = 0; i < NAV_LINE_SENSORS; ++i) {
    seen.line[i] = rd16(f, (uint16_t)(f->dbg + DBG_IN + 2u * i));
  }
  seen.distance = rd16(f, (uint16_t)(f->dbg + DBG_IN + 2u * NAV_LINE_SENSORS));
  if (memcmp(&seen, in, sizeof seen) != 0) {
    ++f->adc_errors;
  }
  const nav_motors h = nav_step(&f->host, &seen);
  const nav_state fw_state = (nav_state)f->avr->data[f->dbg + DBG_STATE];
  if (h.left != m.left || h.right != m.right || f->host.state != fw_state ||
      f->host.masked != f->avr->data[f->dbg + DBG_MASKED]) {
    ++f->mismatches;
  }
  *state = fw_state;
  return m;
}

static int cmp_u32(const void* a, const void* b) {
  const uint32_t x = *(const uint32_t*)a;
  const uint32_t y = *(const uint32_t*)b;
  return x < y ? -1 : x > y;
}

static void usage(void) {
  fprintf(stderr,
          "usage: rtnav-fil --elf FILE [--track NAME] [--laps N] [--seed N]\n"
          "                 [--dead-sensor I:V] [--durations FILE]\n");
}

int main(int argc, char** argv) {
  const char* elf_path = NULL;
  const char* track_name = "oval";
  const char* durations_path = NULL;
  sim_params p = sim_default_params();
  sim_run_options opt = {.laps = 1};
  for (int i = 1; i + 1 < argc; i += 2) {
    const char* a = argv[i];
    const char* v = argv[i + 1];
    if (strcmp(a, "--elf") == 0) {
      elf_path = v;
    } else if (strcmp(a, "--track") == 0) {
      track_name = v;
    } else if (strcmp(a, "--laps") == 0) {
      opt.laps = (unsigned)strtoul(v, NULL, 10);
    } else if (strcmp(a, "--seed") == 0) {
      p.seed = strtoull(v, NULL, 10);
    } else if (strcmp(a, "--durations") == 0) {
      durations_path = v;
    } else if (strcmp(a, "--dead-sensor") == 0) {
      unsigned idx;
      unsigned val;
      if (sscanf(v, "%u:%u", &idx, &val) != 2 || idx >= NAV_LINE_SENSORS || val > NAV_ADC_MAX) {
        usage();
        return 2;
      }
      p.dead_sensor = (int)idx;
      p.dead_value = (uint16_t)val;
    } else {
      usage();
      return 2;
    }
  }
  if (!elf_path || argc % 2 == 0) {
    usage();
    return 2;
  }

  uint32_t dbg_addr;
  uint32_t end_addr;
  if (!elf_symbol(elf_path, "rtnav_debug", &dbg_addr) || !elf_symbol(elf_path, "_end", &end_addr)) {
    die("cannot find rtnav_debug and _end in the ELF");
  }

  elf_firmware_t fw;
  memset(&fw, 0, sizeof fw);
  if (elf_read_firmware(elf_path, &fw) != 0) {
    die("cannot read the ELF");
  }
  strcpy(fw.mmcu, "atmega328p");
  fw.frequency = F_CPU_HZ;
  fw.vcc = fw.avcc = fw.aref = VREF_MV;

  fil f;
  memset(&f, 0, sizeof f);
  f.avr = avr_make_mcu_by_name(fw.mmcu);
  if (!f.avr) {
    die("simavr has no atmega328p");
  }
  avr_init(f.avr);
  f.avr->log = 0;
  avr_load_firmware(f.avr, &fw);
  f.dbg = (uint16_t)(dbg_addr - SRAM_OFFSET);
  const uint16_t ram_free = (uint16_t)(end_addr - SRAM_OFFSET);
  for (uint32_t a = ram_free; a <= RAMEND; ++a) {
    f.avr->data[a] = PAINT;
  }
  avr_irq_register_notify(avr_io_getirq(f.avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 5), on_debug_pin, &f);

  const nav_config cfg = nav_default_config();
  nav_init(&f.host, &cfg);

  sim_track t;
  if (!sim_track_build(&t, track_name)) {
    die("unknown track");
  }
  const sim_result r = sim_run(&t, &p, &opt, fil_controller, &f);

  // Deepest stack use: the stack grows down from RAMEND into painted RAM.
  uint32_t low = ram_free;
  while (low <= RAMEND && f.avr->data[low] == PAINT) {
    ++low;
  }
  const unsigned stack_bytes = (unsigned)(RAMEND + 1u - low);

  if (f.n == 0) {
    die("no control steps observed");
  }
  uint32_t* sorted = malloc(f.n * sizeof *sorted);
  if (!sorted) {
    die("out of memory");
  }
  memcpy(sorted, f.durations, f.n * sizeof *sorted);
  qsort(sorted, f.n, sizeof *sorted, cmp_u32);
  const uint32_t worst = sorted[f.n - 1];
  const uint32_t p50 = sorted[f.n / 2];
  const uint32_t p99 = sorted[(f.n * 99) / 100];

  if (durations_path) {
    FILE* out = fopen(durations_path, "w");
    if (!out) {
      die("cannot write the durations file");
    }
    fprintf(out, "cycles\n");
    for (size_t i = 0; i < f.n; ++i) {
      fprintf(out, "%u\n", f.durations[i]);
    }
    fclose(out);
  }

  const uint16_t missed = rd16(&f, (uint16_t)(f.dbg + DBG_MISSED));
  const uint16_t worst_end = rd16(&f, (uint16_t)(f.dbg + DBG_WORST_END));
  printf(
      "track=%s laps=%u seed=%llu completed=%s collided=%s sim_time_s=%.2f avoids=%u "
      "searches=%u final_state=%s steps=%zu fw_steps=%u step_p50_cycles=%u step_p99_cycles=%u "
      "step_worst_cycles=%u step_worst_us=%.1f budget_pct=%.2f period_jitter_cycles=%lld "
      "fw_worst_end_us=%u missed_ticks=%u stack_bytes=%u ram_static_bytes=%u "
      "lockstep_mismatches=%u adc_readback_errors=%u hung=%s\n",
      t.name, opt.laps, (unsigned long long)p.seed, r.completed ? "yes" : "no",
      r.collided ? "yes" : "no", r.time_s, r.avoids, r.searches, nav_state_name(r.final_state), f.n,
      rd32(&f, f.dbg + DBG_STEPS), p50, p99, worst, worst / (F_CPU_HZ / 1e6),
      100.0 * worst / TICK_CYCLES, (long long)f.worst_period_dev, 4u * worst_end, missed,
      stack_bytes, ram_free - 0x100u, f.mismatches, f.adc_errors, f.hung ? "yes" : "no");

  free(sorted);
  free(f.durations);
  sim_track_free(&t);
  const bool ok = r.completed && !f.hung && f.mismatches == 0 && f.adc_errors == 0 && missed == 0;
  return ok ? 0 : 1;
}
