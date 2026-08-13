# Shortcuts — Carolus / RoboMaster

Shortcut scripts for frequent operations. Common prerequisites: robot powered on (double chime), Pi at `192.168.0.103`.

---

## `carolus_launcher.py`

**What:** Tkinter GUI (dark theme) — T1/T2/T3/T4/T5 sequence, live dashboard, manual chassis (ZQSD) and gimbal (numpad) piloting, interactive key blocks, LOCATE mode (beacon localization without advancing), LOCK button (periodic beacon re-centering, period configurable in seconds, 2026-07-23), beacon indicator + minimap (2026-07-23, see dedicated section), RECENTER CAM button (gimbal base position, 2026-07-23), CAM PREVIEW button (OFF by default — toggles the camera subscription, gains smoothness + network bandwidth), fullscreen (**F11** toggles, **Escape** exits, 2026-07-23), DOCKING BALISE panel — CALIBRATE/CAL STEP 2/START/ABORT buttons + status readout (2026-07-27, see T5 below).

**Why:** launches the stack with no commands to type; a live dashboard (SEARCH/APPROACH/STOP/LOCATE/MANUAL state, depth, robot battery, camera); immediate piloting without leaving the window; visual feedback for active keys.

**Usage:**
```bash
python3 shortcuts/carolus_launcher.py
```

| Button | What runs | Unlocked when |
|---|---|---|
| 1 · roscore + Pi | integrated SSH → `eth1 up` + `roscore` | port 11311 open (60s timeout) |
| 2 · Camera + Beacon | integrated SSH → `rm_cam_beacon.py` + `cam_view_helper.py` | `/camera/color/image_raw` published (60s timeout) |
| 3 · Carolus **[Pi]** | SSH → `roslaunch carolus_node testcarolus.launch ubuntu2204_preload:=false` **on the Pi** | — (manual) |
| 4 · TF Broadcaster (quat fix) | integrated SSH → `carolus_tf_broadcaster.py` on the Pi | — (manual, no wait — a lightweight node, near-instant startup) |
| 6 · MINS (simulation, Pi) | SSH → `roslaunch mins simulation.launch` in `~/mins_sandbox_ws` on the **Pi** | — (independent: own roscore, needs no topic from our pipeline while running its simulation) |
| 5 · Beacon Docking | local (lab PC) → `beacon_docking.py` | — (manual; unlocked after T4, no strict wait — depends on `/pose` (T3), `/odom`+`/carolus/gimbal_yaw_rel` (T2), which just need to already be running) |

**T4 — added 2026-07-20, following the BUG-048 fix** (Carolus→ROS quaternion remapping, a naive permutation replaced by composition `q_ros=r⊗q`). Republishes Carolus's `/pose` as a TF (`camera_link`→`beacon_observed`) via `carolus_tf_broadcaster.py`, run on the Pi. Has no effect on the current SEARCH/ALIGN/APPROACH pipeline (which consumes `/pose` directly, not the TF) — relevant for validating orientation (`rosrun tf tf_echo camera_link beacon_observed`) and lays the groundwork for Phase F's tf2_ROS/EKF adoption. Can be launched independently of T3, but won't have anything to republish until T3 (the source of `/pose`) is running.

**T5 — added 2026-07-27, first hardware session run the same day (journal entry 10).** Runs `beacon_docking.py` directly on the lab PC (like T3 — no SDK connection of its own, so no conflict with `rm_cam_beacon.py`'s single Pi-side SDK connection). Fixed-pose docking behavior relative to the beacon, currently the simplified `SIMPLE_APPROACH_ONLY` pipeline (gimbal align → chassis align via `yaw_rel` → drive to `DOCK_DISTANCE_M`, 0.20m). Controlled by the **DOCKING BALISE** panel: `CALIBRATE` + `CAL STEP 2` (2-click calibration — step 1 measures face-on, step 2 measures after the user pivots ~30-45°, no time pressure between them; the original single-shot 20s-timer design was unusable in practice and was replaced), `START`, `ABORT`. `BEACON_YAW_VALIDATED` is **currently `True`** on this robot (`BEACON_YAW_SIGN=+1.0`, `BEACON_YAW_OFFSET_DEG=+2.4`, calibrated 2026-07-27). Commands go through `cam_view_helper.py` (`/carolus/dock`, same relay pattern as `RECENTER`), status comes back via the `[DOCKSTATUS]` log line parsed the same way as `[BEACON]`. T5 only unlocks its buttons after seeing its own first `[DOCKSTATUS]` line (not just after the process starts) — sending a command too early used to be silently dropped (BUG-076, fixed). **Validated on hardware with mixed results**: one collision (BUG-078, root-caused and fixed — a silent gimbal-probe failure let the chassis align to the wrong reference), one clean approach that ended up close to the beacon but off-center (open issue: nothing corrects chassis heading during/after the drive, only the camera tracks — fix identified, not yet implemented, see roadmap next-session item 12).


**T6 — MINS, added 2026-08-04.** Runs on the **Pi**, not the lab PC — it is the machine carrying the sensors and the only one on Ubuntu 20.04, ROS Noetic's official target. Launches MINS's own `simulation.launch` from `~/mins_sandbox_ws` (a deliberately separate sandbox, not `carolus_ws`, while MINS is not integrated). Measured 2026-08-04: it works and is accurate (RMSE 0.113°/0.082 m, NEES 0.9/1.4) but runs at **0.3-0.4x real time** — with the simulation's own sensor load (2 cameras + LIDAR + IMU 200 Hz), far heavier than ours. Next step is pointing it at our real topics and re-measuring.

**Raspberry Pi status readout, added 2026-08-04.** The dashboard now shows the Pi's temperature, load, RAM and CPU frequency, refreshed every 20 s over SSH. It reads `/sys` and `/proc` directly rather than `vcgencmd`, which does not exist on Ubuntu. Three deliberate precautions, each for a failure already seen: the probe runs in its own **thread** (on 2026-08-04 the Pi answered pings while SSH hung indefinitely — a synchronous read would have frozen the GUI), the `ssh` call has a **hard timeout** with `BatchMode` (otherwise it waits forever for a password prompt), and the period is **slow on purpose** — this is context, not telemetry, and a fast probe would add SSH load to a Pi being watched precisely because it saturates. Colour turns amber at 65 °C and red at 75 °C, ahead of the Pi 4B's 80 °C thermal throttle.

**Logs — reworked 2026-07-20: one tab per terminal, extended 2026-07-27 to T5.** All 5 terminals (T1-T5) are **fully integrated** (their output is captured and shown in the app, no external gnome-terminal window). The Logs area is a `ttk.Notebook` (`T1 roscore+Pi`, `T2 Camera+Beacon`, `T3 Carolus Astrobee`, `T4 TF Broadcaster`, `T5 Docking`), each terminal writes only to its own tab — no more mixing in a single box. Global event messages (AUTO/MANUAL mode, kill, etc.) still get broadcast to all tabs at once. The "Copy logs" button now copies only the active tab's content. A change needed to launch T1 in integrated mode: pre-checked that `sudo` on the Pi doesn't ask for a password (`sudo -n true`), otherwise T1's `sudo ip link set eth1 up` command would block silently in the pipe.

**T3 moved to the Pi, 2026-08-04.** It ran on the lab PC until then. Measured the same afternoon, same beacon at 1.00 m, the only variable being which machine ran the node: **2.19 Hz on the lab PC against 13.04 Hz on the Pi**, a factor of 5.95 — on the PC every frame is an uncompressed 1280×720 `sensor_msgs/Image` (~2.76 MB) crossing the network first. `ubuntu2204_preload:=false` is **mandatory** in this invocation: the `LD_PRELOAD` it disables exists only for Ubuntu 22.04's library mismatch and hardcodes `x86_64` paths, so leaving it on under `aarch64` would be wrong, not merely useless. The lab PC remains the better machine for *developing* Carolus (compiling over SSH is slower) — in that case launch `roslaunch` by hand in a terminal; this launcher is the operations tool, and operations run on the Pi.

**The machine is in every tab label, deliberately** (`T1 roscore [Pi]`, `T3 Carolus [Pi]`, `T5 Docking [PC]`, …). On 2026-08-04 a `/pose` measurement was attributed to the Pi while Carolus was actually running on the lab PC: the ROS master lives on the Pi in both cases, so nothing on screen distinguished them. The machine name is the most useful thing an operator can read on that tab.

**Kill**: cancels any pending waits (`wait_for_roscore` / `wait_for_camera`) then kills the SSH and local processes. Reaps OS zombies (`proc.wait()`). A partial Kill (a row's Kill button) only kills that target and its downstream processes.

> **Remote kills go through `remote_kill()` since 2026-08-04 (BUG-095), and they now verify.** `ssh` runs a compound command inside a remote shell whose own `/proc` cmdline **contains the pattern being matched**, so a plain `pkill -9 -f rm_cam_beacon.py` could kill that shell before reaching the real target — non-deterministically, depending on PID scan order, and with nothing reported. Demonstrated on 2026-08-04: the unbracketed multi-line form produced *no output at all* (the shell died mid-way) and left 1 of 3 targets alive; the bracketed form printed its confirmation and left 0. Patterns are now bracketed automatically (`[r]m_cam_beacon.py`) by a single helper so they are never hand-written again, and `remote_kill()` **re-reads the Pi afterwards** — a survivor logs `> !! TOUJOURS VIVANT sur le Pi : …` in the tab instead of failing silently. The most important instance was the on-close cleanup, whose own comment explains that an orphaned `rm_cam_beacon.py` keeps the SDK connection and causes the next launch to hit the double-connection bug ("manual mode stopped working") — that cleanup was one of the broken ones, so it failed exactly in the case it exists to prevent. It now also kills `carolus_astrobee` and `roslaunch`, since T3 runs on the Pi.


> **Log drain ceiling raised 2026-08-10 (BUG-098).** Reader threads push into an
> unbounded `queue.Queue`; `_flush_log_queue` drained it on the main thread at
> **50 lines per 50 ms tick — a hard 1000 lines/s ceiling**. Measured on a real
> session (`logs/session-2026-08-10-15-54-11.log`, 467050 lines / 919 s): the
> arrival rate is a median of **614** and a peak of **915** lines/s, i.e. **92 %
> of that ceiling**, 99.5 % of it from T3's per-blob logging. At peak, one
> delayed tick backs the queue up and it never recovers — the operator then
> reads a log and a dashboard lagging reality by seconds while the robot moves.
> Raised to 400 lines/tick (**8000 lines/s**, ~9× headroom); the cap is kept so
> one tick's worst case after a backlog stays bounded, and a backlog now says so
> (`> !! log backlog N lines`, once per 5 s) instead of being absorbed silently.
> **This is a real defect found while investigating the GUI-freeze report — it
> is not a demonstrated cause of it.** That symptom is still unexplained; see
> `research-log/journal.md`, 2026-08-10 entries (10) and (11).


> **Camera preview decodes only on change, 2026-08-10.** `_refresh_cam` called
> `tk.PhotoImage(file=CAM_PNG)` every 50 ms unconditionally — a full PNG decode
> on the main thread, **measured at 6.19 ms, i.e. 12.4 % of it at that period**.
> The helper writes at its own 20 Hz and the two are not synchronised, so the
> same bytes were frequently decoded twice. An `os.path.getmtime` comparison now
> skips unchanged files: with a static image that is **1 decode instead of 20
> per second (~118 ms/s of main thread returned)**, and a changed file still
> goes through on the next tick. `_cam_png_mtime` is reset to `None` wherever
> the canvas is cleared (CAM OFF, and the dashboard reset) — without that the
> preview would stay blank until the helper happened to rewrite the file.


> **BUG-099 / BUG-100 fixed 2026-08-10 — manual piloting had no heartbeat, the
> gimbal had no deadman.** The Pi stops the chassis if no `/carolus/cmd_vel`
> arrives within 0.5 s (`MANUAL_CMDVEL_TIMEOUT`). The launcher only ever sent on
> a key *event*, and since BUG-060's auto-repeat debounce a **held** key
> produces no further events — so one command was sent, the deadman expired, and
> **the robot stopped with the key still down** ("navigation stopped
> responding"). Fixed with `_cmd_heartbeat`, re-sending every **200 ms**
> (`CMD_HEARTBEAT_MS`, 2.5x margin under the deadman) while any key is held.
> Verified it does not reintroduce BUG-060's sawtooth: 6 consecutive identical
> sends over 1.0 s, max gap 0.209 s, no zero interleaved.
>
> The gimbal had the opposite defect — `_gim_stamp` was written and **never
> read**, so the last commanded speed repeated at 20 Hz forever and a lost
> `KeyRelease` left it turning with nothing to stop it. `MANUAL_GIMBAL_TIMEOUT`
> added on the Pi, symmetric with the chassis. **Not yet run on hardware.**
>
> Together these predict both halves of the 2026-08-10 incident (gimbal pitching
> down *and* navigation unresponsive). Predict, not prove — reproducing it
> deliberately is still the next hardware step.


> **Layout: two columns, and the window finally fits the screen (2026-08-10).**
> Everything was stacked in one column, making the window **1406 px tall — 326 px
> more than the 1920x1080 primary screen**, so the log panel fell off the bottom;
> and `resizable(False, False)` meant there was no way to cope with it. Removing
> the live map freed the right column, so the logs (the tallest block) moved
> there: **1510 x 1043**, which fits. The window is also **resizable** now —
> locking a fixed layout is defensible, locking one that can open taller than the
> screen is not. Measured before/after with `winfo_reqwidth/reqheight`, not
> eyeballed.

---

### Live map — REMOVED 2026-08-10

The embedded `_LiveMapCanvas` panel (lab floor plan, obstacle blocks, robot and
beacon position over `mapv1.json`) and the **ÉDITEUR MAP** button that opened
`map_editor.py` were **removed at the user's request**. 178 lines gone from the
launcher; it now has one column instead of two.

Archived, not deleted — `archive/mapv1-2026-08-10/` holds `mapv1.json` (which
was empty: no blocks, no beacons), `map_editor.py`, and the extracted
`_LiveMapCanvas` class, so the feature can be reconstructed if it is ever
wanted again.

**The BEACON MINIMAP was deliberately KEPT.** It shares the word "map" and
nothing else: it shows where the beacon sits *within the camera frame* (green
when centred within ±3°), which is a live detection aid, not a map of the lab,
and it has no connection to `mapv1.json`. See its own section below.


---

### LOCATE mode

The **LOCATE** button in the control row (gold-yellow when active). Publishes `"LOCATE"` on `/carolus/mode`.

**Behavior in LOCATE mode:**
- The gimbal sweep continues (same as AUTO/SEARCH).
- As soon as the beacon is visible: the robot stops (`stop_gimbal` + `stop_chassis`).
- No ALIGN/APPROACH transition — the robot stays in place.
- If the beacon disappears: the sweep resumes.

**Auto-activation:** LOCATE activates automatically 500ms after T2 confirms the camera is ready. No need to manually click LOCATE at startup.

To switch back to AUTO (full tracking): click **MODE: AUTO**.

---

### LOCK (periodic beacon re-centering, 2026-07-23)

The **LOCK** button in the control row, with an input field next to it (period in seconds, default **1**). Publishes `"LOCK ON"`/`"LOCK OFF"` on `/carolus/gimbal_lock`, and the period on `/carolus/gimbal_lock_period` (via `cam_view_helper.py`).

**How it works:** every *N* seconds (N = the field's value), if there's a fresh pose, a **single** relative movement command (`gimbal.move()`) re-centers the beacon in frame, independent of chassis motion.

- **Live-configurable period**: type a value (e.g. 5, 10) into the field and press Enter. **Seconds only.** An unparseable value (text, negative, empty) is ignored without crashing — a silent fallback to 1s on the `rm_cam_beacon.py` side, like a standard web form field. Tested by changing the value on an already-running node, without restarting the stack.
- **Yaw only**: pitch stays disabled (`GIM_PITCH_TRACK_ENABLED=False`, since the BUG-058 incident — gimbal hit a mechanical stop → cable snagged).
- Active only in MANUAL mode, reset to OFF when entering/leaving MANUAL and on Kill.
- **Numpad gimbal control IGNORED while LOCK is ON** (2026-07-23 night): when LOCK is active, it has exclusive control of the gimbal; the 8/4/5/6 numpad keys have no effect. The chassis (ZQSD) stays normally controllable. Outside LOCK, the numpad regains control.
- Skips a tick if the angle error exceeds `GIM_LOCK_MAX_ERR_DEG` (45°) — likely an aberrant pose.
- **Deadband `GIM_LOCK_DEADBAND_DEG=5°`**: below this threshold, no re-correction (not worth re-centering that finely).
- **Speed `GIM_LOCK_YAW_SPEED=540°/s`** (the SDK's cap, explicit user request) — never tested on this robot above 80°/s before this choice, **confirmed working on hardware** on 2026-07-23 (evening).
- **History:** an earlier LOCK BEACON (a continuous 20Hz servo with gating/ramp/outlier-rejection, distinct from the mechanism above, sometimes called "v1") existed from 2026-07-22 to 2026-07-23 then was **removed entirely** on 2026-07-23 (evening), judged redundant. This button was then called "LOCK V2"; it was renamed simply "LOCK" once v1 was removed.
- **Confirmed working on hardware on 2026-07-23 (evening)** by the user ("everything works"), including at 540°/s.

---

### Beacon indicator + minimap (2026-07-23)

Below the dashboard's camera panel.

- **Indicator** (a circle + text, in English): `BEACON: DETECTED` (green) / `BEACON: LOST` (red). Fed by the `[BEACON] status=...` log that `rm_cam_beacon.py` publishes at 5Hz.
- **BEACON MINIMAP** (a small 100×100 canvas): a dot represents the beacon's position *within the camera frame* (green if centered within ±3°, orange otherwise). Not to be confused with the lab-floor live map, which was removed 2026-08-10.
- Reset on entering/leaving MANUAL and on Kill (same hygiene as the LOCK button).
- **REMEMBER BEACON/SEARCH BEACON buttons removed 2026-07-23 (night)**: had been confirmed working on hardware earlier that evening, then judged unsatisfactory by the user without further detail — a full removal was requested, no code trace remains. Replaced by the **RECENTER CAM** button (see the dedicated section below).

---

### RECENTER CAM (gimbal base position, 2026-07-23)

The **RECENTER CAM** button, below the camera panel. Publishes `"RECENTER"` on `/carolus/gimbal_recenter`.

- Returns the gimbal to its base position (pitch=0, yaw=0, the gimbal's power-on frame) via the SDK's `gimbal.recenter()` — orientation of the **camera**, independent of the robot chassis's orientation.
- Re-centering speed: 360°/s on both axes (the SDK's cap for `recenter()`, different from `move()` which caps at 540°/s).
- Active only in MANUAL mode (same scope as LOCK).
- **Bug fixed 2026-07-23 (night)**: wasn't working because the MANUAL loop kept re-sending `drive_speed(0,0)` at 20Hz, cancelling the re-centering action ~50ms after it started (a large re-centering angle takes ~0.7s). Fix: a 2.5s "gimbal busy" window during which the MANUAL loop and LOCK suspend their commands. **Fixed in code, not yet deployed/tested** (Pi unreachable at the time of the fix).

---

### Visual piloting blocks (MANUAL mode only)

Two blocks appear below the launch buttons. Keys light up gold when active (keyboard or mouse click).

**CHASSIS (ZQSD)**

```
      [Z]
  [Q] [S] [D]
```
- `Z` = forward · `S` = reverse · `Q` = rotate left · `D` = rotate right
- vx = 0.20 m/s · wz = 20 deg/s
- Auto-stop when all keys are released

**GIMBAL (NUM 8/4/5/6/2)**

```
      [8]
  [4] [5] [6]
      [2]
```
- `8` = pitch up · `2` = pitch down · `4` = yaw left · `6` = yaw right · `5` = stop gimbal
- pitch = 30 deg/s · yaw = 40 deg/s
- Works with NumLock ON (`KP_8`…) and NumLock OFF (`KP_Up`…)

**Activation:** click `MODE: AUTO` → switches to `MODE: MANUAL` (orange). ZQSD/numpad active both from the launcher **and** from the map editor window (bindings propagated to both windows). Guard: keys don't trigger a command if focus is on a text input widget. Back to AUTO: click again, chassis + gimbal stop immediately.

**Keyboard focus:** if ZQSD stops responding, **hover the mouse over the launcher window** — focus is reclaimed automatically (`<Enter>` → `focus_set()`, added 2026-07-01).

> **Corrected 2026-08-10.** This section used to say T1 and T3 open external `gnome-terminal` windows that steal focus. **They do not, and have not since 2026-07-20**, when every terminal became integrated (`_cmd_integrated`, one log tab each) — the code has said so for a month while this line still described the old behaviour. The map-editor window, the other thing named here, was removed the same day this was corrected. The hover binding is kept anyway: it costs nothing and still covers focus taken by *another application*, which no fix of ours can prevent. Worth recording because this stale line was, until it was checked, the leading candidate explanation for the unexplained GUI-freeze incident — and it turned out to describe a mechanism that no longer exists.

---

### Dashboard

| Indicator | Detail |
|---|---|
| Robot state dot | gray=SEARCH · orange=APPROACH · green=STOP · gold-yellow=LOCATE · blue=MANUAL |
| `depth = X.XXm` | beacon distance in APPROACH mode |
| Robot battery | green bar >40% · orange 15-40% · red <15% · `N/A` if not exposed |
| 320×180 camera | PNG thumbnail updated ~4 Hz via `cam_view_helper.py` |
| Pi connection | pinged every 5s → green/red dot + IP |

**Logs:** a selectable area, `Ctrl+A` to select all, `Ctrl+C` to copy, a **"Copy logs"** button. No freezing thanks to the async queue (50ms batches / max 50 lines, throughput 1000 lines/s). High-frequency telemetry (`[ESC]`, `[ATTI]`, `[POS]`, `[BAT]`, `[VEL]`, `[TOF]`) filtered out of the Logs area — shown only in the dashboard. `[BEACONPOS]` stays visible in the logs (useful for diagnosing the beacon's position).

**Session log on disk (added 2026-07-31).** Every line written to any tab is also appended to `shortcuts/logs/session-YYYY-MM-DD-HH-MM-SS.log`, one file per launcher start, each line prefixed with the time and the originating tab (`T1`..`T5`, or `--` for a global event broadcast to all tabs). Before this, logs lived only in the tkinter widgets: closing the launcher lost them, and each tab is truncated to 300 lines anyway. The concrete cost showed up on 2026-07-31 — the question *"is LOCK still ticking during a docking run?"* was unanswerable even though a run that would have answered it had already happened; the logs simply weren't kept. Grep by tab to answer that class of question directly:

```bash
grep '\[T2\].*\[LOCK\]' shortcuts/logs/session-*.log     # was LOCK active during the run?
grep '\[T5\]' shortcuts/logs/session-*.log                # everything the docking node said
```

Best-effort by construction: a write error never brings the GUI down. But it is **not silent** — if the file cannot be opened, a line goes to stderr saying so, because a log that isn't written without saying so is worse than no log at all (you believe you have the data and you don't). `shortcuts/logs/` is gitignored.

---

## `lever_arm_bearing.py`

**Quoi** — calcule, pour un decalage `d` entre centre de rotation du chassis et centre optique de la camera, le changement apparent de gisement d'une cible fixe apres une rotation sur place.

**Pourquoi** — le 2026-07-30, un run ALIGN a montre le gisement de la balise passer de +6.3 a +18.2 deg (environ +12) sur ~97 deg de rotation chassis, alors que `yaw_ground` restait plat (-142.3 -> -142.4) : la camera n'a donc PAS tourne dans le repere monde, et le changement ne peut pas venir d'une rotation camera. Deux causes candidates : geometrie du bras de levier, ou glissement lateral Mecanum reel. Ce script chiffre la premiere pour savoir s'il reste quelque chose a expliquer.

**Usage**
```bash
python3 shortcuts/lever_arm_bearing.py            # resout l'inverse sur le run du 2026-07-30
python3 shortcuts/lever_arm_bearing.py --d 0.12   # predit pour un d mesure (metres)
```

Sans `--d`, il repond a la question utile avant toute mesure : *quel bras de levier faudrait-il pour expliquer TOUT le changement observe ?* Reponse pour le run du 2026-07-30 : **12.3 cm** — une valeur physiquement plausible sur un S1, donc la geometrie est une explication complete credible et il suffit d'un metre-ruban pour trancher.

Avec `--d`, il donne la borne superieure (deplacement entierement perpendiculaire a la ligne de visee) puis le modele exact pour plusieurs orientations de depart. A noter : le signe depend fortement de l'orientation — a 12 cm le modele va de -6.8 a +11.8 deg selon l'angle de depart de la camera autour du centre de rotation. C'est en soi une prediction testable.

**Attendu** — un nombre en degres a comparer aux +12 deg observes. S'il explique l'essentiel, la question du glissement Mecanum disparait sans avoir besoin d'y repondre ; s'il n'explique qu'une fraction, le reste demande une autre cause (glissement, ou intrinseques fausses — points 2 et 14).

**Limite** — modele de translation pure. Ne dit rien des intrinseques ni du solveur ; ne remplace pas la mesure de `d`, il la rend exploitable en 30 secondes.

---

## `save_session.sh`

**What:** snapshots the active source files into `saves/YYYY-MM-DD-HH-MM/`.

**Why:** allows rolling back to a stable state if a change breaks something.

**Usage:**
```bash
bash shortcuts/save_session.sh "before gimbal test"
# Restore a file:
cp saves/2026-06-24-20-10/carolus_ws__src__robomaster_cam__scripts__rm_cam_beacon.py \
   carolus_ws/src/robomaster_cam/scripts/rm_cam_beacon.py
```

**Files saved:** `carolus_launcher.py`, `cam_view_helper.py`, `map_editor.py`, `rm_cam_beacon.py`, **`beacon_docking.py`**, **`beacon_absolute_pose.py`**, `testcarolus.launch`, plus the workspace's 5 `CMakeLists.txt` files (`src/`, `libuvgs_astrobee/`, `ff_msgs/`, `robomaster_cam/`, `carolus_node/`) — the `CMakeLists.txt` set was added 2026-07-13 to cover the CLAUDE.md rule listing them as critical files; the two docking scripts were added **2026-07-28**.

> **Why the two docking scripts were added (2026-07-28):** they were absent from the list while being the most heavily modified files of the docking work, so every backup during that work had to be made **by hand** — twice in a single session. A backup script that silently omits the file you are actually editing is worse than no script, because it gives the impression of a safety net that is not there. **Rule going forward: any source file under active modification must be in `FILES` before the work starts, not after it bites.**

**Expected:** a `saves/YYYY-MM-DD-HH-MM/` folder created with 12 files + `NOTE.txt`.

---

## `deploy_pi.sh`

**What:** synchronises to the Pi **every package that runs there** — `robomaster_cam`, `carolus_node`, `libuvgs_astrobee`, `ff_msgs` — reports drift per package by checksum, re-verifies after copying, and rebuilds on request.

**Why:** it used to push exactly one file, `rm_cam_beacon.py`, because that was the only program running on the Pi. **That stopped being true on 2026-08-04**, when Carolus itself moved there (13.04 Hz on the Pi against 2.19 Hz on the lab PC). `carolus_node`'s launch files and profile YAMLs, and `libuvgs_astrobee`'s C++, now live on the Pi too — and nothing kept them in step. They were rsynced by hand once and then drifted silently.

That is this project's most expensive failure pattern. On 2026-08-04 the Pi was running a `rm_cam_beacon.py` with **no `/imu` publisher at all** (0 occurrences against 8 on the lab PC), and a full diagnostic session was spent on stale code. So the script no longer pushes "the file I just edited": it pushes everything that runs there, and it **verifies**.

**Usage:**
```bash
bash shortcuts/deploy_pi.sh                 # DRY RUN: report drift, change nothing
bash shortcuts/deploy_pi.sh --apply         # push, then re-verify by checksum
bash shortcuts/deploy_pi.sh --apply --build # push, verify, and recompile on the Pi
```

| Step | Check |
|---|---|
| 0 | local `py_compile` on every `robomaster_cam` script — abort on a syntax error |
| 1 | Pi reachable — abort otherwise |
| 2 | reports the Pi's **git state** (HEAD + number of locally modified files) |
| 3 | per-package drift by checksum (`rsync -rc --dry-run`), dry-run unless `--apply` |
| 4 | **re-reads both sides after copying** — an rsync exiting 0 is not proof the two sides match |
| 5 | `ast.parse` with the Pi's own interpreter |
| 6 | `catkin_make` **only with `--build`**, and only if C++/launch changed |

**Why the rebuild is not automatic:** recompiling `carolus_node`/`libuvgs_astrobee` takes minutes on the Pi, and a launch-file or YAML change needs none. The script says whether one is required and lets the operator decide.

**Why it does not reconcile with git:** the Pi's `~/carolus_ws` is a real git checkout (verified 2026-08-04: HEAD `318409b`, 13 locally modified files). Two paths to the Pi already exist — git, and this script. A third, or a silent automatic merge between them, would be exactly the class of bug this is trying to remove. The script **reports** the git state and leaves the decision to the operator.

**After deploying:** restart the affected tabs — T2 for `rm_cam_beacon.py`, **T3 for Carolus** (which now runs on the Pi).

### All source comments are in English (2026-08-04)

A colleague reading the public repository pointed out that the Python comments were in French. They were: **622 comment and docstring lines** across seven files. All of them are now English, along with the user-visible log strings, which were mixed too.

The translation was verified rather than assumed: after each batch the file was recompiled, `beacon_docking.py --selftest` was re-run (it exercises the pure geometry functions and all their regression cases), and finally each file's **AST was compared against its pre-translation snapshot with every string constant blanked** — identical for both large files, proving that only comments and strings changed and no logic moved.

Three comments were **corrected rather than transposed**, because translating a false statement faithfully just preserves the error:

- `TARGET_FPS`'s header warned that Carolus's bottleneck was network transport and that raising the rate would only add bandwidth to a saturated link. True in July; false since 2026-08-04, when Carolus moved onto the Pi and the bottleneck became the Pi's CPU. The history is kept and marked as such, because the change of conclusion *is* the information.
- The BUG-089 comment still described the bug as unexplained ("the callback is NEVER called"). Rewritten to state the cause was ours and is fixed.
- A comment pointed at `research-log/07-perplexity/08-...`, a path that no longer exists and that a reader of the public repository could never open anyway. Replaced by the fact itself.

### Two numbers now measured on every run (2026-08-04)

Both used to require a dedicated session, and neither had ever been collected. They are produced by `rm_cam_beacon.py` continuously instead, so they can never be stale.

- **`[DUTY] beacon in view NN%`**, every 30 s. The fraction of time the beacon is actually visible. Carolus is the only drift-free source in the stack — everything else (wheel odometry, gyro integration) drifts without bound — so how often it can correct decides how much inertial quality has to be bought. Around 90% and the drift between corrections is negligible, which would retire the whole "the SDK caps the IMU at 50 Hz where VIO wants 200–500 Hz" concern. Around 20% and that ceiling becomes a structural limit of the robot.
- **`[LATENCY] /pose NN ms`**, averaged over 50 poses. We measured Carolus's pose *rate* (13.04 Hz on the Pi) and quoted it to the supervisor, but for drift correction **latency is the metric that decides quality**: a correction arriving at 13 Hz but 200 ms late is worse than one at 5 Hz arriving in 30 ms, because the filter must roll its state back that far. If the value is implausible the node says so rather than printing a meaningless number — that means `carolus_astrobee` is running without `stamp_from_acquisition`, or the two machines' clocks disagree.

**Expected:** `RESULT: tout est deja en phase avec le Pi`, or a per-package drift list. Reported in sync on 2026-08-04 after the day's changes.

---

## `doc_check.py`

**What** — checks `overleaf/technical.tex` against the code it documents: every fully-qualified path the manual names is confirmed to still exist on disk, and every changed/named file is checked for whether the manual mentions it (and where, so the review is fast).

**Why** — created 2026-08-12, the same day a documentation-sync check was done by hand with a plain `grep` and came back "nothing to review" for four components the manual actually documents. The reason: LaTeX escapes underscores, so `grep cam_view_helper` can never match `cam\_view\_helper.py` in the source. That false "nothing found" nearly stood as the answer, and it is exactly the failure mode this project already has a rule about elsewhere — a check that reports "nothing" whether the target is truly absent or the search is broken is not a check. This script's own search normalises LaTeX escaping away first, and — because a search that quietly breaks is the whole problem — it self-tests that normalisation against a known-escaped identifier before reporting anything, and refuses to output otherwise (exit code 2). The self-test is verified to actually fail when it should: deliberately disabling the escaping logic and re-running it produces the abort, not a false pass (a first version of this self-test used a canary that also appeared unescaped inside a code listing, so even the broken search still found it and the self-test itself was silently useless — caught by directly testing the failure path rather than assuming it worked).

**Usage**
```bash
python3 shortcuts/doc_check.py                 # audit + review your uncommitted changes
python3 shortcuts/doc_check.py --all           # audit + review every published file
python3 shortcuts/doc_check.py FILE [FILE...]  # review specific files
python3 shortcuts/doc_check.py --find NAME     # LaTeX-aware search for one identifier, with line numbers
```

**Expected** — `self-test OK` first (if this is missing or the run aborts with exit 2, do not trust anything below it), then a stale-reference list (empty is the good outcome) and a per-file list of manual line numbers to open and check by hand. It flags *that* a line mentions a changed file, not whether that line is still correct — "mentioned" is not "correct", and the tool says so in its own output.

**Deliberately narrow, and why**: an earlier version also flagged bare filenames (`patch.sh`, `target.yaml`, `dji.json`) as stale, which was wrong every time it fired — those are real, correct references to files in the S1 rooting toolkit and Kalibr, tools this manual describes without vendoring. A bare filename cannot tell "our file was deleted" from "their file we never had", so only fully-qualified `carolus_ws/`/`shortcuts/`/`overleaf/` paths are checked for existence. Naming-template strings (`session-YYYY-MM-DD-HH-MM-SS.log`, anything with a `*` glob) are excluded from that check for the same reason — they describe a pattern the code generates, not a file that should exist right now.

---

## `dep_check.py`

**What** — checks that this workspace can build on a machine that is **not this one**. Compares every active `find_package()` in each `CMakeLists.txt` against what the matching `package.xml` declares, validates each manifest is well-formed XML, and with `--resolve` confirms every declared key actually resolves via `rosdep` for a target OS.

**Why** — created 2026-08-13, after the supervisor's build failed on two consecutive days on two different missing dependencies (`image_transport`, then Ceres) while following the technical guide on his own Pi. The common cause was one thing: **`rosdep install` reads only `package.xml`**, so a library named solely in a `find_package()` call is invisible to it — it reports success, installs nothing, and the build dies at CMake configure on any machine that did not already happen to have the library. That is undetectable here by construction: our machines have had every dependency installed for months as a side effect of unrelated work, so the build succeeds locally for reasons unrelated to the build being correctly specified. Three separate name spaces are involved and getting them confused is its own failure mode — the CMake name (`Ceres`), the informal name (`ceres`, **not** a registered rosdep key), and the actual rosdep key (`libceres-dev`).

**Usage**
```bash
python3 shortcuts/dep_check.py                      # audit, target ubuntu:focal (the Pi)
python3 shortcuts/dep_check.py --resolve            # also resolve every key via rosdep
python3 shortcuts/dep_check.py --os ubuntu:jammy --resolve
```

**Expected** — one line per package (`OK` / `DRIFT` / `MALFORMED XML` / `skipped`), then, with `--resolve`, either `all N keys resolve` or a list of unresolvable keys. Exit 0 clean, 1 on any problem, so it is usable as a pre-push or CI gate.

**Verified to actually fail**, four ways, restoring byte-identical each time: removing both Ceres declarations reproduced the supervisor's exact bug and the tool named it precisely; a literal `--` inside an XML comment (illegal in XML, and a mistake genuinely made while fixing this) was caught as a malformed manifest; declaring the bogus key `ceres` was caught as unresolvable; and a sloppy first sabotage that removed only `build_depend` while leaving `exec_depend` correctly reported clean, which exposed the *test* as flawed rather than the tool. Requires `rosdep update --include-eol-distros` to have been run — Noetic is end-of-life and a plain `rosdep update` silently skips it, after which nothing resolves; the script detects that specific state and says so instead of blaming your manifests.

---

## `cleanroom_build.sh`

**What** — builds the project the way a stranger would: a disposable `ubuntu:20.04` Docker container with nothing pre-installed, ROS Noetic installed exactly as the guide instructs, then this working tree's `carolus_ws/src` through `rosdep install` → `catkin_make --pkg ff_msgs` → `catkin_make`.

**Why** — same 2026-08-13 cause as `dep_check.py`, but this is the ground truth rather than a static audit: it does not reason about the spec, it performs the reader's actual install and reports what really happens. It uses the working tree rather than a git clone deliberately, so a fix is validated **before** being pushed rather than after the supervisor finds it. `ros-base` is installed deliberately (not `desktop-full`) because that is what the guide recommends for the Pi and precisely the minimal choice that omitted the perception stack.

**Usage**
```bash
bash shortcuts/cleanroom_build.sh              # full run (slow, ~15-25 min)
bash shortcuts/cleanroom_build.sh --deps-only  # stop after rosdep install
```

**Expected** — staged `### [n/5]` progress, ending in `PASS -- a stranger following the guide can build this` or `FAIL (rc=N)` with the full log path under `/tmp/cleanroom-*.log`.

**It earned its place on its first run**, failing at a step nobody suspected: `rosdep resolve roscpp` returns "no rosdep rule" whenever `ROS_DISTRO` is unset, so rosdep silently cannot map any ROS package name at all. The guide masks this by appending the source line to `~/.bashrc`, which works interactively and not in a script. **Scope limit, stated rather than implied**: the container is x86_64, so this validates dependency declarations and the documented command sequence, *not* ARM-specific compilation — which is fine, because declarations are the failure mode actually being shipped, but it is not a Pi. `robot_localization` is excluded as vendored third-party, and the script prints that exclusion rather than hiding it.

---

## `leak_scan.sh`

**What:** a pattern-based scan (hardcoded passwords/API keys/tokens/private-key headers) over `carolus_ws/`, `shortcuts/`, `github/`, `research-log/`.

**Why:** a safety net before any external send-off (a first `git push`, an Overleaf upload) — created 2026-07-24 as part of the leak audit (`research-log/15-audit-fuites.md`), which found a real plaintext password in `journal.md` (since fixed). Keyword-only detection (no dedicated tool like gitleaks/trufflehog installed on this machine) — doesn't replace a manual reread of the terminal outputs pasted into the journal.

**Usage:**
```bash
bash shortcuts/leak_scan.sh                      # scan the 4 default folders
bash shortcuts/leak_scan.sh specific/path         # scan one specific folder
```

**Expected:** `Rien trouve sur les motifs connus.` if clean, otherwise a list of suspicious lines to check manually (false positives possible).

---

## `cam_view_helper.py`

**What:** a separate process with three roles — (1) a PNG camera thumbnail ~4 Hz with an overlaid HUD (crosshair at the image's geometric center + a tolerance ring + a beacon marker reprojected via the real intrinsics, 2026-07-23), (2) a GUI-keyboard → ROS chassis-topic gateway, (3) a GUI-numpad → ROS gimbal-topic gateway.

**Why:** an isolated process because `rospy.init_node` + SIGINT conflict with Tkinter. Centralizes every ROS publication coming from the GUI without a 2nd SDK connection. Launched/stopped automatically by `carolus_launcher.py`.

**Stdin commands (sent by the launcher over a PIPE):**

| Command | Effect |
|---|---|
| `MODE AUTO` | Publishes `"AUTO"` on `/carolus/mode` (latched) |
| `MODE MANUAL` | Publishes `"MANUAL"` on `/carolus/mode` (latched) |
| `MODE LOCATE` | Publishes `"LOCATE"` on `/carolus/mode` (latched) — sweep without advancing |
| `VX 0.20 WZ 20.0` | Publishes `Twist(linear.x, angular.z)` on `/carolus/cmd_vel` |
| `STOP` | Publishes a zeroed `Twist()` on `/carolus/cmd_vel` |
| `GIMBAL 30.0 0.0` | Publishes `Twist(angular.y=pitch, angular.z=yaw)` on `/carolus/gimbal_vel` |
| `LOCK ON` / `LOCK OFF` | Publishes `"ON"`/`"OFF"` on `/carolus/gimbal_lock` (periodic re-centering) |
| `LOCKPERIOD 5.0` | Publishes `"5.0"` on `/carolus/gimbal_lock_period` (period in seconds, falls back to default if invalid) |
| `RECENTER` | Publishes `"RECENTER"` on `/carolus/gimbal_recenter` (gimbal base position, 2026-07-23) |
| `DOCK CALIBRATE` / `DOCK CALSTEP2` / `DOCK START` / `DOCK ABORT` | Publishes `"CALIBRATE"`/`"CALSTEP2"`/`"START"`/`"ABORT"` on `/carolus/dock` (consumed by `beacon_docking.py`, T5, 2026-07-27) |

**Usage:** (automatic, via the launcher) — manual for debugging:
```bash
source /opt/ros/noetic/setup.bash && source carolus_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.103:11311 ROS_IP=192.168.0.100
python3 shortcuts/cam_view_helper.py /tmp/carolus_cam.png
```

**Expected:** `/tmp/carolus_cam.png` updates ~4×/s. In MANUAL mode, `rm_cam_beacon.py` responds to VX/WZ/GIMBAL commands within ~50ms.

---

## `map_editor.py` — disabled since 2026-08-10

**Not reachable from the GUI any more**: its launcher button and wiring were removed the same day as the live-map feature (`mapv1.json`, empty and unused, see the entry above). Kept in the code rather than archived, per an explicit decision 2026-08-12 — `map_collision.py`, still active inside `rm_cam_beacon.py`, reads exactly the JSON format this editor produces, so re-enabling the map feature later means re-wiring this file into `carolus_launcher.py`, not rewriting it. Running it directly (`python3 shortcuts/map_editor.py`) now prints a message explaining this instead of silently doing nothing — it has no standalone entry point of its own, being a `tk.Toplevel` meant to be opened from another Tk root.

**What it did:** a 2D map editor (a separate Toplevel window) — a 26×21-cell grid (10.4m×8.4m, 1 cell = 40 cm ≈ the S1's footprint), full/half/quarter blocks, a zone tool (drag fill), oriented half-block beacons, a grid-snapped robot overlay.

**Why it existed:** to map the lab's real obstacles (chairs, desks) before a session, position the beacon, and visualize the robot's position live. The exported map JSON was loaded by `map_collision.py` on the Pi for obstacle avoidance.

**Usage (historical):** opened from the "MAP EDITOR" button in `carolus_launcher.py`, before that button was removed.

| Tool | Left click | Right click |
|---|---|---|
| ▓ Full | Place a full block | No effect |
| ▬ Half | Place a half-block (auto-rotated by position) | Change rotation |
| ▪ Quarter | Place a quarter-block | Change rotation |
| ▦ Zone | Drag → fill a rectangle with full blocks | — |
| ◉ Beacon | Place beacon (MANUAL mode) | Rotate beacon 90° |
| ✕ Erase | Remove a block or beacon | — |

**Robot:** locked by default (🔒). Unlock via the palette → drag → auto-snap to the cell center. The position defines the (0,0) origin for every live SDK update.

**Beacons:**
- **MANUAL mode** (default): drag to place, right-click to rotate. A single orange beacon.
- **AUTO mode:** `add_auto_beacon(wx_m, wy_m, facing_deg)` called by the launcher on every `[BEACONPOS]`. Multi-beacon, deduplication < 0.5m, gold-yellow color.

**Save/Load:** JSON v3 — `blocks`, `beacon_man` (wx, wy, rot), `beacons_auto` (a list of wx/wy/facing). Copy the saved map to `/home/ubuntu/carolus_map.json` on the Pi to enable collision avoidance.

**Navigating the map:**
- **Mouse wheel** → zoom centered on the cursor (1.15× per notch, Linux Button-4/5 supported)
- **Right-click + drag** → pan (moves the view), every item moves together
- **Right-click without dragging** → change the rotation of the block under the cursor (unchanged behavior)
- Zoom/pan doesn't affect the stored coordinates (world, meters) — save/load stays correct at any zoom level

**Axis convention (EP SDK → map):**
- EP `x+` = forward → north on the canvas (py decreases)
- EP `y+` = right (east) → px increases — the reverse of the ROS REP-103 convention
- The robot is positioned at the grid's geometric center by default (column 13, row ~10.5 — 21 rows is odd, so the exact center isn't a whole cell — on a 26×21 grid, since the 2026-06-30 enlargement — previously at the bottom of the 20×15 grid).

**Expected:** the blue robot overlay (■▲) moves live via `update_robot()`, a temporary orange dot via `update_beacon()`, persistent beacons via `add_auto_beacon()`. Zoom with the wheel, pan with right-click drag. An optimized hover ghost (cached per cell/rotation — no redraw if the mouse stays in the same cell).

---

## measure_pi_pose.sh

**What** — measures Carolus's real `/pose` rate on the Raspberry Pi with a beacon in view, in one command, and captures several other findings while the stack is up.

**Why** — the 2026-08-04 Pi benchmark measured ~24 frames/s *processed*, counted with **no beacon in view**: no P4P solve ran per frame and the outlier filter was never exercised. The number the supervisor will actually quote — `/pose` under load — is still unmeasured. This script collects it in one command so a short hardware slot is spent on the robot rather than on typing commands.

It also captures, opportunistically: whether the BUG-087 non-convergence warning ever fires in a real run (surfaced 2026-08-03, never yet observed live), whether `[LOCK]` ticks during the run, blob-detection health, and CPU load.

**Usage**
```bash
bash shortcuts/measure_pi_pose.sh          # full run (~3 min)
DURATION=120 bash shortcuts/measure_pi_pose.sh   # longer measurement window
bash shortcuts/measure_pi_pose.sh --stop   # stop everything, leave the Pi clean
```

**Prerequisites it cannot check for you:** robot powered on, Pi reachable, and **a powered beacon in the camera's field of view** — without it `/pose` never publishes and the run measures nothing.

**Expected** — a full report written to `shortcuts/logs/pi-pose-measure-<timestamp>.log`, with the `/pose` rate as the headline figure. Fails early and clearly if the Pi is unreachable. `--stop` prints the surviving process list, or `Pi clean` if there is none.

> **`--stop` was silently broken until 2026-08-04 (BUG-090).** `ssh` runs the whole stop body as one remote shell, whose own command line contains the very strings being matched, so `pkill -f rm_cam_beacon` killed that shell before it reached anything. Patterns are now bracket classes (`"[r]m_cam_beacon"`). If you ever add a `pkill -f` over ssh anywhere in this project, use the same form.

> **The stack-startup step carried the same class of bug (BUG-097, 2026-08-08).** `nohup foo &` inside a single SSH command does not reliably survive that SSH session ending on this Pi — `nohup` blocks `SIGHUP` specifically, but session teardown here kills the whole process group, which `nohup` does not detach from. Caught live in an ad-hoc command of the same shape: `roscore` never actually started — no log file, port 11311 closed, the dependent process silently blocked forever. Fixed here with `setsid nohup foo >log 2>&1 </dev/null & disown`, which gives the process its own session. Applied as a precaution; not independently re-verified on this exact script since the live failure was caught elsewhere — confirm on the next Pi session. If you ever background a long-lived process over ssh anywhere in this project, use the same form.

---

## sync_repo.sh — retired 2026-08-12

Reconciled `carolus_repo/` (a hand-assembled mirror of `carolus_ws/src/`, `cmake_shims/` and `shortcuts/`) with the live working files, since nothing else kept them in sync and the published repo drifted silently more than once. Removed once `carolus_repo/` itself was dissolved into a single tree rooted at the project root — there is no longer a second copy to reconcile, so there is nothing left for this script to do.

---

## `capture_checkerboard.py`

**What** — subscribes to `/camera/color/image_raw` and saves a live preview frame to disk on keypress, for the MATLAB Camera Calibration Toolbox procedure (`technical.tex`, Chapter "Camera Calibration").

**Why** — created 2026-08-11 after reviewing the LEO/LIMO documents Hector sent: they use the exact same MATLAB checkerboard method, and this project's own manual already documented the procedure (25 mm checkerboard, MATLAB's Camera Calibration app) without ever having a way to actually save frames from the live camera stream — the values currently loaded (`camera_info.yaml`) were a pragmatic stand-in, never the output of this procedure. No existing script in this project wrote a camera frame to disk.

**Usage**
```bash
python3 shortcuts/capture_checkerboard.py [output_dir]   # default: data/checkerboard/
```
Live preview window. `s` saves the current frame as `checkerboard_NNN.png`; `q` quits. Aim for at least 15 frames, checkerboard tilted more than 45° from the optical axis in each, varied position/orientation across the field of view, occupying roughly 20–25% of the frame — per the manual's own already-written recommendation.

**Expected** — a folder of PNG frames ready to upload into MATLAB's Camera Calibration app (Apps tab), enable tangential distortion + three radial distortion coefficients, run, and transcribe the exported `fx`/`fy`/`cx`/`cy`/distortion into `testcarolus.launch`.

**Needs the robot powered on**, camera streaming, and a printed 25 mm checkerboard.

**BUG-102, fixed 2026-08-11 (first real run crashed):** this lab PC has a numpy 2.2.6 install shadowing the system numpy `cv2` needs, and a system `cv_bridge` linked against a different OpenCV build than `cv2` itself (same defect class as BUG-101, same day) — both are corrected automatically by a self-re-exec guard at the top of the script; the plain command above is unaffected and needs no extra flags.

---
