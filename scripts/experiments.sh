#!/bin/sh
# Reproduces every number in the README into results/, then redraws the
# charts. Needs avr-gcc, simavr, libelf, cmake and python3; the easy way:
#
#   docker build -t rtnav-dev docker
#   docker run --rm -v "$PWD":/src -w /src rtnav-dev scripts/experiments.sh
#
# STRESS_TICKS (default 30000) sets the length of the torn-read stress runs.
set -eu
cd "$(dirname "$0")/.."
STRESS_TICKS=${STRESS_TICKS:-30000}
B=build-experiments
R=results
mkdir -p "$R"

echo "== building firmware and negative-control variants"
make -s -C firmware > "$R/firmware-size.txt"
for v in torn-read no-volatile poll poll-no-volatile blocking-adc; do
  make -s -C firmware VARIANT=$v > /dev/null
done
cmake -S . -B $B -DCMAKE_BUILD_TYPE=Release -DRTNAV_FIL=ON > /dev/null
cmake --build $B -j > /dev/null
ELF=firmware/build/rtnav.elf
FIL=$B/rtnav-fil
SIM=$B/rtnav-sim

echo "== host simulation: fault matrix (pass count out of 10 seeds, 2 laps)"
{
  for f in "" "--left-gain 0.7" "--right-gain 0.6" "--noise 30" \
           "--dead-sensor 0:80" "--dead-sensor 2:80" "--dead-sensor 4:80" \
           "--dead-sensor 0:1023" "--dead-sensor 1:1023" "--dead-sensor 2:900" \
           "--dead-sensor 3:1023" "--dead-sensor 4:900"; do
    for tr in oval lobes; do
      ok=0
      for s in 1 2 3 4 5 6 7 8 9 10; do
        # shellcheck disable=SC2086
        $SIM --track $tr --laps 2 --seed $s $f > /dev/null && ok=$((ok + 1))
      done
      echo "fault=\"${f:-none}\" track=$tr passed=$ok/10"
    done
  done
  for tr in oval lobes gaps obstacle; do
    ok=0
    for s in $(seq 1 20); do $SIM --track $tr --laps 2 --seed $s > /dev/null && ok=$((ok + 1)); done
    echo "fault=none track=$tr passed=$ok/20"
  done
  $SIM --track obstacle --no-avoid || true
} > "$R/sim-faults.txt"

echo "== firmware in the loop: every track, 2 laps"
for tr in oval lobes gaps obstacle; do
  $FIL --elf $ELF --track $tr --laps 2 --durations $B/dur-$tr.csv 2> /dev/null \
    | grep -v '^Loaded' > "$R/fil-$tr.txt" &
done
wait
{
  echo "track,cycles"
  for tr in oval lobes gaps obstacle; do tail -n +2 $B/dur-$tr.csv | sed "s/^/$tr,/"; done
} > "$R/step-cycles.csv"
$FIL --elf $ELF --track obstacle --trace "$R/fil-obstacle-trace.csv" > /dev/null 2>&1
$SIM --track obstacle --track-csv "$R/track-obstacle.csv" > /dev/null

echo "== negative controls"
{
  for v in no-volatile poll poll-no-volatile blocking-adc; do
    printf 'variant=%s ' $v
    $FIL --elf firmware/build-$v/rtnav.elf --track oval 2> /dev/null | grep -v '^Loaded' || true
  done
  echo "-- poll-no-volatile wait loop (tick_due loaded once, then a jump to itself):"
  avr-objdump -d -S firmware/build-poll-no-volatile/rtnav.elf | grep -A7 'while (!tick_due)' | head -8
} > "$R/negative-controls.txt"
for v in build build-torn-read; do
  printf 'firmware=%s ' $v
  $FIL --elf firmware/$v/rtnav.elf --adc-stress "$STRESS_TICKS" 2> /dev/null | grep -v '^Loaded' || true
done > "$R/torn-read-stress.txt" &
wait

python3 scripts/charts.py
echo "== done: $R/ and docs/"
