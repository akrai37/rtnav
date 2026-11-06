#!/usr/bin/env python3
"""Draws the README charts from results/ into docs/ (standard library only).

  docs/avoidance.svg  the firmware's path around the obstacle, by state
  docs/step-time.svg  control-step duration on the chip vs the 10 ms budget
"""

import csv
import os
import statistics

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(ROOT, "results")
DOCS = os.path.join(ROOT, "docs")
F_CPU = 16_000_000
BUDGET_US = 10_000

# Light and dark tokens; the SVG follows the viewer's colour scheme.
STYLE = """<style>
  svg { --surface: #fcfcfb; --ink: #0b0b0b; --ink2: #52514e; --muted: #898781;
        --grid: #e1e0d9; --axis: #c3c2b7; --tape: #c3c2b7; --obstacle: #e1e0d9;
        --s1: #2a78d6; --s2: #eb6834; --s3: #1baf7a; }
  @media (prefers-color-scheme: dark) {
    svg { --surface: #1a1a19; --ink: #f0efec; --ink2: #c3c2b7; --muted: #898781;
          --grid: #2c2c2a; --axis: #383835; --tape: #4a4a46; --obstacle: #383835;
          --s1: #3987e5; --s2: #d95926; --s3: #199e70; }
  }
  text { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif;
         fill: var(--ink2); font-size: 12px; }
  .title { fill: var(--ink); font-size: 15px; font-weight: 600; }
  .strong { fill: var(--ink); font-weight: 600; }
  .muted { fill: var(--muted); font-size: 11px; }
</style>"""


def svg_open(w, h, label):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" '
        f'height="{h}" role="img" aria-label="{label}">',
        STYLE,
        f'<rect width="{w}" height="{h}" rx="8" fill="var(--surface)"/>',
    ]


def write(name, parts):
    os.makedirs(DOCS, exist_ok=True)
    with open(os.path.join(DOCS, name), "w") as f:
        f.write("\n".join(parts + ["</svg>"]) + "\n")


def avoidance():
    with open(os.path.join(RESULTS, "track-obstacle.csv")) as f:
        rows = list(csv.DictReader(f))
    line = [(float(r["x"]), float(r["y"])) for r in rows if r["kind"] == "line"]
    obstacle = next(r for r in rows if r["kind"] == "obstacle")
    with open(os.path.join(RESULTS, "fil-obstacle-trace.csv")) as f:
        trace = [(float(r["x"]), float(r["y"]), r["state"]) for r in csv.DictReader(f)]

    # World window (mm) around the manoeuvre; y up.
    x0, x1, y0, y1 = 100.0, 900.0, 520.0, 960.0
    w, h = 760, 520
    left, top, scale = 20, 80, (w - 40) / (x1 - x0)

    def px(x, y):
        return left + (x - x0) * scale, top + (y1 - y) * scale

    out = svg_open(w, h, "Robot path around an obstacle, coloured by navigation state")
    out.append('<text x="20" y="32" class="title">Obstacle avoidance, run by the firmware on '
               "the simulated ATmega328P</text>")
    out.append('<text x="20" y="54">Path of the robot\'s axle centre on the obstacle track, '
               "travelling right to left; 100 mm obstacle on the line.</text>")
    out.append(f'<clipPath id="win"><rect x="{left}" y="{top}" width="{(x1 - x0) * scale:.1f}" '
               f'height="{(y1 - y0) * scale:.1f}"/></clipPath>')
    out.append('<g clip-path="url(#win)">')
    tape = " ".join(f"{px(x, y)[0]:.1f},{px(x, y)[1]:.1f}" for x, y in line)
    out.append(f'<polyline points="{tape}" fill="none" stroke="var(--tape)" '
               f'stroke-width="{19 * scale:.1f}" stroke-linejoin="round"/>')
    cx, cy = px(float(obstacle["x"]), float(obstacle["y"]))
    out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{float(obstacle["r"]) * scale:.1f}" '
               'fill="var(--obstacle)" stroke="var(--muted)" stroke-width="1"/>')

    colour = {"follow": "var(--s1)", "avoid": "var(--s2)", "rejoin": "var(--s3)"}
    seg, state = [], None
    for x, y, s in trace:
        if s != state and seg:
            out.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{colour[state]}" '
                       'stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round"/>')
            seg = seg[-1:]
        state = s
        p = px(x, y)
        seg.append(f"{p[0]:.1f},{p[1]:.1f}")
    out.append(f'<polyline points="{" ".join(seg)}" fill="none" stroke="{colour[state]}" '
               'stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round"/>')
    out.append("</g>")

    # Where AVOID began, and direct labels.
    first_avoid = next((x, y) for x, y, s in trace if s == "avoid")
    first_rejoin = next((x, y) for x, y, s in trace if s == "rejoin")
    ax, ay = px(*first_avoid)
    rx, ry = px(*first_rejoin)
    out.append(f'<circle cx="{ax:.1f}" cy="{ay:.1f}" r="5" fill="var(--s2)" '
               'stroke="var(--surface)" stroke-width="2"/>')
    out.append(f'<text x="{ax - 6:.1f}" y="{ay + 26:.1f}" text-anchor="end">'
               '<tspan class="strong">obstacle seen</tspan> at ~17 cm: stop, turn right</text>')
    out.append(f'<text x="{cx:.1f}" y="{cy + 4:.1f}" text-anchor="middle" class="muted">'
               "obstacle</text>")
    out.append(f'<text x="{cx:.1f}" y="{px(0, 820)[1]:.1f}" text-anchor="middle">'
               '<tspan class="strong">AVOID</tspan>: arc around (~300 mm radius)</text>')
    out.append(f'<text x="{rx + 10:.1f}" y="{ry - 14:.1f}">'
               '<tspan class="strong">REJOIN</tspan>: drive over the line, spin onto it</text>')
    out.append(f'<text x="{px(860, 0)[0]:.1f}" y="{ay - 14:.1f}" text-anchor="end">'
               '<tspan class="strong">FOLLOW</tspan></text>')

    # Legend.
    lx = 20
    for name, key in (("FOLLOW", "s1"), ("AVOID", "s2"), ("REJOIN", "s3")):
        out.append(f'<rect x="{lx}" y="{h - 26}" width="14" height="4" rx="2" fill="var(--{key})"/>')
        out.append(f'<text x="{lx + 20}" y="{h - 20}">{name}</text>')
        lx += 100
    out.append(f'<rect x="{lx}" y="{h - 30}" width="14" height="12" fill="var(--tape)"/>')
    out.append(f'<text x="{lx + 20}" y="{h - 20}">19 mm tape</text>')
    write("avoidance.svg", out)


def step_time():
    with open(os.path.join(RESULTS, "step-cycles.csv")) as f:
        cycles = [int(r["cycles"]) for r in csv.DictReader(f)]
    us = [c * 1e6 / F_CPU for c in cycles]
    srt = sorted(us)
    p50 = statistics.median(srt)
    p99 = srt[int(len(srt) * 0.99)]
    worst = srt[-1]

    w, h = 760, 400
    left, right, top, bottom = 60, 20, 150, 330
    lo, hi, bin_us = 75, 450, 10
    nbins = (hi - lo) // bin_us
    counts = [0] * nbins
    for v in us:
        counts[min(nbins - 1, int((v - lo) // bin_us))] += 1
    cmax = max(counts)
    ymax = ((cmax // 1000) + 1) * 1000

    def x(v):
        return left + (v - lo) / (hi - lo) * (w - left - right)

    def y(c):
        return bottom - c / ymax * (bottom - top)

    out = svg_open(w, h, "Distribution of control-step durations against the 10 ms budget")
    out.append('<text x="20" y="32" class="title">Control-step time on the chip: worst '
               f"{worst:.0f} µs of a {BUDGET_US:,} µs budget ({100 * worst / BUDGET_US:.2f}%)</text>")
    out.append(f'<text x="20" y="54">{len(us):,} steps of the real firmware on simavr '
               "(ATmega328P, 16 MHz), all four tracks, timed by the debug pin.</text>")

    # Budget meter: the whole 10 ms tick, and the share the worst step used.
    mx0, mx1, my = 20, w - 20, 74
    out.append(f'<rect x="{mx0}" y="{my}" width="{mx1 - mx0}" height="14" rx="4" '
               'fill="var(--grid)"/>')
    out.append(f'<rect x="{mx0}" y="{my}" width="{(mx1 - mx0) * worst / BUDGET_US:.1f}" '
               'height="14" rx="4" fill="var(--s1)"/>')
    out.append(f'<text x="{mx0}" y="{my + 32}">worst step, to scale</text>')
    out.append(f'<text x="{mx1}" y="{my + 32}" text-anchor="end">10 ms control tick (100 Hz); '
               "the CPU sleeps for the rest</text>")

    for c in range(0, ymax + 1, 1000):
        out.append(f'<line x1="{left}" x2="{w - right}" y1="{y(c):.1f}" y2="{y(c):.1f}" '
                   f'stroke="var(--{"axis" if c == 0 else "grid"})" stroke-width="1"/>')
        out.append(f'<text x="{left - 8}" y="{y(c) + 4:.1f}" text-anchor="end" class="muted">'
                   f"{c:,}</text>")
    out.append(f'<text x="{left - 8}" y="{top - 12}" text-anchor="end" class="muted">steps</text>')
    for v in range(100, hi + 1, 50):
        out.append(f'<text x="{x(v):.1f}" y="{bottom + 18}" text-anchor="middle" class="muted">'
                   f"{v}</text>")
    out.append(f'<text x="{(left + w - right) / 2:.0f}" y="{bottom + 38}" text-anchor="middle" '
               'class="muted">control-step duration (µs)</text>')

    bw = (w - left - right) / nbins
    for i, c in enumerate(counts):
        if c == 0:
            continue
        bx = x(lo + i * bin_us) + 1
        out.append(f'<rect x="{bx:.1f}" y="{y(c):.1f}" width="{bw - 2:.1f}" '
                   f'height="{bottom - y(c):.1f}" rx="2" fill="var(--s1)">'
                   f"<title>{lo + i * bin_us}-{lo + (i + 1) * bin_us} µs: {c:,} steps</title></rect>")

    for v, name, dy in ((p50, "median", 0), (p99, "p99", 16), (worst, "worst", 32)):
        out.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="{top - 4}" y2="{bottom}" '
                   'stroke="var(--ink2)" stroke-width="1" stroke-dasharray="3 3"/>')
        out.append(f'<text x="{x(v) - 4:.1f}" y="{top + 8 + dy}" text-anchor="end">'
                   f'<tspan class="strong">{name}</tspan> {v:.0f} µs</text>')
    write("step-time.svg", out)
    return p50, p99, worst


if __name__ == "__main__":
    avoidance()
    p50, p99, worst = step_time()
    print(f"step time: median {p50:.1f} us, p99 {p99:.1f} us, worst {worst:.1f} us")
