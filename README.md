# rtnav

Bare-metal C firmware for an ATmega328P (Arduino Uno) robot that follows a line and drives
around obstacles. It has a 100 Hz timer-interrupt control loop, interrupt-driven ADC sampling
of five IR line sensors and an IR rangefinder, PWM motor drive through an H-bridge, and a
fixed-point navigation state machine. It is written against the registers with avr-libc and
no Arduino library.

This is a re-implementation of a robot I built earlier on Arduino hardware. This version is
verified in simulation: the compiled firmware runs on a cycle-accurate AVR simulator in a
closed loop with a simulated robot. **It has not been measured on physical hardware.** All
timings below are simulator CPU cycles of a 16 MHz ATmega328P.

## Results

| | |
|---|---|
| Control step, worst case on the chip | **6,807 cycles = 425 µs, 4.3% of the 10 ms budget** (median 295 µs, p99 383 µs) |
| Tick-to-tick jitter | ≤ 72 cycles (4.5 µs) |
| Missed ticks | 0 in 12,027 control steps (all four tracks, 2 laps each) |
| Firmware vs host build, lockstep | **0 mismatches**: identical motor commands, state and fault mask on every tick |
| Footprint | 3,266 B flash (10%), 47 B static RAM, 237 B peak stack (of 2 KB SRAM) |
| Closed loop, host simulation | 20/20 seeds on each of 4 tracks; 10/10 with a weak motor, a dead sensor or 4× sensor noise; 2 of 5 stuck-dark sensor cases still fail ([table](#what-the-simulation-found)) |
| Negative controls | a busy-wait without `volatile` hangs; blocking ADC oversampling overruns the tick 2× (1,450 missed ticks). A torn-read build was **not** caught; [see why](#negative-controls) |

<p align="center"><img src="docs/step-time.svg" alt="Histogram of control-step durations: median 295 µs, worst 425 µs, against a 10 ms budget"></p>

<p align="center"><img src="docs/avoidance.svg" alt="Path of the robot around an obstacle: turn away, arc around, rejoin the line"></p>

## How it works

```
Timer1 CTC, 100 Hz ──► ISR sets tick_due ──► main loop wakes from idle sleep
                                              │
ADC ISR, round robin over ADC0..5             ▼
(~16 fresh samples per channel per tick) ─► snapshot (interrupts off, 6 × 16-bit)
                                              │
                                              ▼
                           nav_step(): locate line ─► state machine ─► PD steering
                                              │
                                              ▼
                           Timer0 fast PWM + H-bridge direction pins ─► sleep
```

- **Control loop** ([firmware/main.c](firmware/main.c)). The timer interrupt only sets a
  flag; the step runs in the main loop and the CPU sleeps in between. The sleep is race-free:
  interrupts stay off between checking the flag and executing `sleep`. Diagnostics live in RAM:
  step count, missed ticks (counted in the ISR), the worst end-of-step timer value, and a 60 ms
  watchdog.
- **ADC** ([firmware/adc.c](firmware/adc.c)). Each conversion-complete interrupt stores its
  result, switches the multiplexer and starts the next conversion. It does not use
  free-running mode, where a multiplexer change applies one conversion late and mislabels
  samples. The control step copies all six samples with interrupts off, because the 8-bit CPU
  reads each 16-bit sample in two halves.
- **Motors** ([firmware/motors.c](firmware/motors.c)). Timer0 fast PWM at 977 Hz, plus two
  direction pins per motor. Zero duty disconnects the PWM output, because fast PWM with a
  compare value of 0 still emits a one-count pulse.
- **Navigation** ([src/nav.c](src/nav.c)). Portable C with integer arithmetic only, compiled
  unchanged for the AVR and for the host. AVR `int` is 16 bits, so intermediates use explicit
  `int32_t`. It has five states:
  - FOLLOW: PD control on the line position.
  - SEARCH: if the line vanished while centred, drive straight to bridge a tape gap;
    otherwise spin toward where the line was last seen.
  - AVOID: turn away from the obstacle, then arc around it.
  - REJOIN: drive the axle over the line, then spin onto it.
  - STOP: safe state when search, avoidance or rejoin times out.

  All timing is counted in ticks.

## Verification

Each layer runs in CI ([.github/workflows/ci.yml](.github/workflows/ci.yml)):

1. **Unit tests** ([tests/nav_test.c](tests/nav_test.c)) cover the sensor maths, steering
   sign and saturation, every state transition and timeout, fault masking, and determinism.
   They run with ASan and UBSan on Linux (GCC, Clang) and macOS.
2. **Closed-loop host simulation** ([sim/](sim/), [tests/sim_test.c](tests/sim_test.c)). The
   navigation code drives a simulated differential-drive robot on four tracks:
   - an oval;
   - "lobes" with S-bends (tightest radius 227 mm);
   - an oval with 30 mm and 50 mm tape gaps;
   - an oval with an obstacle on the line.

   The model includes motor lag and deadband, sensor noise, per-sensor calibration spread, and
   a GP2Y0A21-style rangefinder curve. That curve is non-linear and folds back below 10 cm,
   so a very close object reads as far away.
3. **Firmware in the loop** ([fil/fil.c](fil/fil.c)). The compiled `rtnav.elf` runs on
   simavr's ATmega328P and drives the same simulated robot:
   - Sensor readings enter as voltages on the ADC pins.
   - Motor commands are read from the PWM compare registers and direction pins.
   - The debug pin brackets each control step, so durations are exact CPU cycles.
   - The sensor snapshot the firmware used is read from its RAM and replayed into the host
     build. Both must produce the same motor commands on every tick, which catches any 16-bit
     vs 32-bit arithmetic difference between the AVR and the host.
   - Free RAM is painted before boot to find the stack high-water mark.

## What the simulation found

Each finding below was a real failure in the simulation, fixed and then regression-tested:

- **Tape gaps made the robot spin.** Losing the line looked the same as missing a curve. Fix:
  if the line vanishes while centred, drive straight for up to 0.6 s first.
- **Turn-then-arc avoidance meets the line almost head-on.** Any arc that starts on the line
  and clears the obstacle returns at a steep angle, and line following cannot recover from
  that. Fix: a REJOIN state that drives the axle over the line and then spins onto it.
- **One outer sensor stuck dark sent the robot spinning (0/10 runs).** The weighted average
  dragged the position, and on a curve the stuck sensor alone kept "finding" the line. Fix:
  19 mm tape can only darken adjacent sensors. The position therefore comes from the run of
  adjacent dark sensors nearest the previous position. A sensor that stays dark apart from
  that run for 0.5 s is masked as faulty (sensor 4 stuck dark: now 10/10 on both tracks).

Fault matrix, from the host simulation with 10 seeds and 2 laps per cell
([results/sim-faults.txt](results/sim-faults.txt)):

| Fault | oval | lobes |
|---|---|---|
| none | 10/10 | 10/10 |
| left motor at 70% strength | 10/10 | 10/10 |
| right motor at 60% strength | 10/10 | 10/10 |
| sensor noise ×4 (30 ADC counts) | 10/10 | 10/10 |
| sensor 0 (outer) dead, reads white | 10/10 | 10/10 |
| sensor 2 (centre) dead, reads white | 10/10 | 10/10 |
| sensor 4 (outer) dead, reads white | 10/10 | 10/10 |
| sensor 0 (outer) stuck dark | 10/10 | **0/10** |
| sensor 1 (inner) stuck dark | 10/10 | 10/10 |
| sensor 2 (centre) stuck dark | 10/10 | 10/10 |
| sensor 3 (inner) stuck dark | **3/10** | **3/10** |
| sensor 4 (outer) stuck dark | 10/10 | 10/10 |

Two stuck-dark cases still fail, for two different reasons:

- **Sensor 3 stuck dark (3/10).** It sits next to the line on straights, so its run merges
  with the line's run, and one snapshot cannot separate them.
- **Sensor 0 stuck dark on the lobes track (0/10).** On some curves the stuck run is nearer
  the previous position than the real line is, so the locator picks it.

A power-on calibration sweep that flags sensors whose reading never changes would catch both;
it is not implemented.

## Negative controls

Deliberately broken builds (`make -C firmware VARIANT=<name>`), run through the same harness
([results/negative-controls.txt](results/negative-controls.txt)):

| Variant | What changes | Result |
|---|---|---|
| `blocking-adc` | 32× oversampling by polling inside the control step | worst step 329,714 cycles (**20.6 ms, 206% of budget**), 1,450 missed ticks; harness fails |
| `poll-no-volatile` | busy-wait on `tick_due` without `volatile` | **hangs**: the compiler loads the flag once and compiles the loop to `rjmp .-2` (a jump to itself); 0 steps; harness fails |
| `poll` | the same busy-wait, flag still `volatile` | works (control for the row above) |
| `no-volatile` | `volatile` removed, sleep loop kept | **still works**: `cli`/`sei` are compiler memory barriers, so the flag is re-read anyway. The bug is latent, not absent |
| `torn-read` | snapshot copied without disabling interrupts | **not caught**: 0 torn values in 30,000 stress ticks, as for the correct build |

The torn-read result is a limit of simulation-based testing. In stress mode the harness flips
every conversion between 255 and 256 (0x0FF and 0x100), so a torn copy would read as 0 or
511. But Timer1, the ADC and the CPU share one clock, so the ADC interrupt can only arrive at
a fixed set of phases relative to the snapshot. None of those phases falls in the
one-instruction gap between the two byte loads. The race is real (the disassembly shows the
unprotected `ld`/`ldd` pair), but a deterministic simulator never hits it. That is why the
code uses the lock rather than relying on a test to find this race.

## Limits

- This is simulation, not hardware. The robot model has no wheel slip, so it says nothing
  about maximum speed; the simulated robot completes the lobes track even at full duty. Motor
  and sensor constants are plausible values for a small chassis, not measurements of one.
- Avoidance timings are open loop, in ticks. They were tuned in this simulation and would need
  calibrating on a real chassis.
- The avoidance always passes on the right and assumes the line continues beyond the obstacle.
- Cycle counts come from simavr 1.6 (Ubuntu 24.04 package) on the ATmega328P model, which is
  cycle-accurate for the core. The host computer's speed does not affect them.

## Build and run

Needs `avr-gcc`, `avr-libc`, simavr (`libsimavr-dev`), libelf, CMake and Python 3, or Docker:

```sh
docker build -t rtnav-dev docker
docker run --rm -v "$PWD":/src -w /src rtnav-dev make -C firmware      # firmware: rtnav.hex
docker run --rm -v "$PWD":/src -w /src rtnav-dev scripts/experiments.sh # every number above
```

Host-only build and tests (any C11 compiler, no AVR tools needed):

```sh
cmake -S . -B build -DRTNAV_WERROR=ON -DRTNAV_SANITIZE=ON
cmake --build build -j && ctest --test-dir build --output-on-failure
./build/rtnav-sim --track lobes --laps 2 --dead-sensor 4:1023
```

Flashing a real Uno: `avrdude -p m328p -c arduino -P <port> -U flash:w:firmware/build/rtnav.hex`.
The pin map is in [firmware/board.h](firmware/board.h).

## Layout

```
firmware/   AVR drivers and main loop (Makefile, avr-gcc)
src/, include/rtnav/   portable navigation logic (shared by firmware and host)
sim/        robot, track and sensor models; tools/rtnav_sim.c runs one scenario
fil/        firmware-in-the-loop harness on simavr
tests/      unit, simulation and firmware-in-the-loop tests
scripts/    experiments.sh reproduces results/; charts.py draws docs/
```

## License

MIT, see [LICENSE](LICENSE).
