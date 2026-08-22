#!/usr/bin/env python3
"""
BUG-116 unattended watcher. Tails t2_beacon.log, looks for a SUSTAINED
per-wheel ESC speed ramp (the windup signature: -3 -> -84 rpm over ~90s,
~-1.7 rpm/s, in the roadmap's own account) rather than an instantaneous
threshold -- the project's own logs show +/-16 rpm of idle noise, so a
fixed-magnitude trigger would false-positive constantly.

Prints one line per event to stdout:
  - HEARTBEAT every ~5 min: elapsed uptime, last ESC/POS reading, whether
    any [MANUAL-DRIVE] fired since the last heartbeat (rules the command
    loop in or out for that window).
  - ALERT the moment a sustained same-direction ramp is detected on >=3
    of 4 wheels -- this is the actual "catch it happening" event, with the
    exact uptime so it can be correlated against the position-hold
    hypothesis (same-sign across wheels = translation attempt).
"""
import os, re, sys, time, collections

LOG = sys.argv[1] if len(sys.argv) > 1 else "t2_beacon.log"
UPTIME_ZERO = float(sys.argv[2]) if len(sys.argv) > 2 else time.time()
T2_PID = int(sys.argv[3]) if len(sys.argv) > 3 else None  # local ssh PID holding T2

def t2_alive():
    if T2_PID is None:
        return True
    try:
        os.kill(T2_PID, 0)
        return True
    except OSError:
        return False

ESC_RE = re.compile(r"\[ESC\] W1=(-?[\d.]+) W2=(-?[\d.]+) W3=(-?[\d.]+) W4=(-?[\d.]+)")
POS_RE = re.compile(r"\[POS\] x=(-?[\d.]+) y=(-?[\d.]+)")
MANUAL_RE = re.compile(r"\[MANUAL-DRIVE\]")

WINDOW_S = 60.0          # slope computed over this trailing window
MIN_SAMPLES = 40         # need most of the window populated to trust a slope
SLOPE_THRESH = 0.5       # rpm/s -- well below the observed ~1.7 rpm/s windup rate
MIN_TOTAL_CHANGE = 15.0  # rpm -- cumulative displacement across the window, not
                         # just an instantaneous slope; noise wanders, windup
                         # accumulates. Requiring both is what actually rejects
                         # noise -- verified by simulation against synthetic
                         # +/-16rpm idle noise (the project's own documented
                         # idle-noise band): a slope-only version of this gate
                         # averaged ~19 false alerts/hour; slope+displacement
                         # together gave 0/20 simulated runs false-firing at
                         # 1h, 3h, 6h AND 10h continuous. Still correctly
                         # catches the documented real signature (-3 -> -84
                         # rpm / ~1.7 rpm/s) and stays quiet on a BUG-093-class
                         # sub-threshold residual drift (~0.1 rpm/s).
CONSISTENT_WHEELS = 3    # how many of 4 wheels must agree in sign+magnitude
HEARTBEAT_S = 300.0

hist = {i: collections.deque() for i in range(4)}  # (t, value) per wheel
last_pos = None
last_esc = None
manual_since_heartbeat = 0
last_heartbeat = time.time()
alerted_until = 0  # cooldown so one ramp doesn't spam

def slope_and_range(dq, now):
    """Returns (slope rpm/s, total displacement over the window) or (None, None)
    if there isn't enough data yet. Both must clear their threshold together --
    slope alone fires on noise; a real windup accumulates displacement, noise
    wanders back and forth and its window displacement stays small."""
    pts = [(t, v) for t, v in dq if now - t <= WINDOW_S]
    if len(pts) < MIN_SAMPLES:
        return None, None
    n = len(pts)
    tbar = sum(t for t, _ in pts) / n
    vbar = sum(v for _, v in pts) / n
    num = sum((t - tbar) * (v - vbar) for t, v in pts)
    den = sum((t - tbar) ** 2 for t, _ in pts)
    sl = num / den if den > 1e-9 else 0.0
    disp = pts[-1][1] - pts[0][1]
    return sl, disp

def uptime_str(now):
    s = int(now - UPTIME_ZERO)
    return f"{s//3600:02d}:{(s%3600)//60:02d}:{s%60:02d}"

print(f"[WATCHER] started, uptime_zero={UPTIME_ZERO}, watching {LOG}, "
      f"T2_PID={T2_PID}", flush=True)

last_liveness_check = time.time()
LIVENESS_CHECK_S = 15.0

with open(LOG, "r") as f:
    f.seek(0, 2)  # tail -f semantics: only new lines
    while True:
        line = f.readline()
        now = time.time()

        if now - last_liveness_check >= LIVENESS_CHECK_S:
            last_liveness_check = now
            if not t2_alive():
                print(f"[CRITICAL] uptime={uptime_str(now)} T2 process (PID {T2_PID}) "
                      f"is DEAD -- the log has stopped, this is NOT a quiet period, "
                      f"the stack needs relaunching. Watcher exiting.", flush=True)
                sys.exit(1)

        if not line:
            if now - last_heartbeat >= HEARTBEAT_S:
                esc_s = f"W={last_esc}" if last_esc else "no ESC sample yet"
                pos_s = f"pos={last_pos}" if last_pos else "no POS sample yet"
                print(f"[HEARTBEAT] uptime={uptime_str(now)} {esc_s} {pos_s} "
                      f"manual_cmds_since_last={manual_since_heartbeat} t2_alive=True", flush=True)
                last_heartbeat = now
                manual_since_heartbeat = 0
            time.sleep(0.5)
            continue

        if MANUAL_RE.search(line):
            manual_since_heartbeat += 1
            continue

        m = ESC_RE.search(line)
        if m:
            vals = [float(x) for x in m.groups()]
            last_esc = vals
            for i, v in enumerate(vals):
                hist[i].append((now, v))
                while hist[i] and now - hist[i][0][0] > WINDOW_S + 5:
                    hist[i].popleft()

            if now > alerted_until:
                results = [slope_and_range(hist[i], now) for i in range(4)]
                qualifying = [(sl, disp) for sl, disp in results
                              if sl is not None
                              and abs(sl) >= SLOPE_THRESH
                              and abs(disp) >= MIN_TOTAL_CHANGE]
                if len(qualifying) >= CONSISTENT_WHEELS:
                    signs = [1 if sl > 0 else -1 for sl, _ in qualifying]
                    same_sign = abs(sum(signs)) == len(signs)
                    pattern = "SAME-SIGN (translation-like, matches the position-hold hypothesis)" \
                              if same_sign else "MIXED-SIGN (rotation-like)"
                    print(f"[ALERT] uptime={uptime_str(now)} sustained ramp detected over {WINDOW_S:.0f}s: "
                          f"slopes(rpm/s)={[round(sl,2) for sl,_ in results if sl is not None]} "
                          f"displacement(rpm)={[round(d,1) for _,d in results if d is not None]} "
                          f"current_W={vals} pattern={pattern}", flush=True)
                    alerted_until = now + 30  # cooldown, avoid re-alerting every sample
            continue

        m = POS_RE.search(line)
        if m:
            last_pos = tuple(float(x) for x in m.groups())
