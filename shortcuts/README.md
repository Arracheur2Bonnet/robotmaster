# Shortcuts — Carolus / RoboMaster

Shortcut scripts for frequent operations. Common prerequisites: robot powered on (double chime), Pi at `192.168.0.103`.

---

## `carolus_launcher.py`

**What:** Tkinter GUI (dark theme) — T1/T2/T3/T4/T5 sequence, live dashboard, integrated live map, manual chassis (ZQSD) and gimbal (numpad) piloting, interactive key blocks, LOCATE mode (beacon localization without advancing), LOCK button (periodic beacon re-centering, period configurable in seconds, 2026-07-23), beacon indicator + minimap (2026-07-23, see dedicated section), RECENTER CAM button (gimbal base position, 2026-07-23), CAM PREVIEW button (OFF by default — toggles the camera subscription, gains smoothness + network bandwidth), fullscreen (**F11** toggles, **Escape** exits, 2026-07-23), DOCKING BALISE panel — CALIBRATE/CAL STEP 2/START/ABORT buttons + status readout (2026-07-27, see T5 below).

**Why:** launches the stack with no commands to type; a live dashboard (SEARCH/APPROACH/STOP/LOCATE/MANUAL state, depth, robot battery, camera); a real-time live map (robot + beacon position over a JSON background); immediate piloting without leaving the window; visual feedback for active keys.

**Usage:**
```bash
python3 shortcuts/carolus_launcher.py
```

| Button | What runs | Unlocked when |
|---|---|---|
| 1 · roscore + Pi | gnome-terminal → SSH → `eth1 up` + `roscore` | port 11311 open (60s timeout) |
| 2 · Camera + Beacon | integrated SSH → `rm_cam_beacon.py` + `cam_view_helper.py` | `/camera/color/image_raw` published (60s timeout) |
| 3 · Carolus Astrobee | gnome-terminal → `roslaunch carolus_node testcarolus.launch` | — (manual) |
| 4 · TF Broadcaster (quat fix) | integrated SSH → `carolus_tf_broadcaster.py` on the Pi | — (manual, no wait — a lightweight node, near-instant startup) |
| 5 · Beacon Docking | local (lab PC) → `beacon_docking.py` | — (manual; unlocked after T4, no strict wait — depends on `/pose` (T3), `/odom`+`/carolus/gimbal_yaw_rel` (T2), which just need to already be running) |

**T4 — added 2026-07-20, following the BUG-048 fix** (Carolus→ROS quaternion remapping, a naive permutation replaced by composition `q_ros=r⊗q`). Republishes Carolus's `/pose` as a TF (`camera_link`→`beacon_observed`) via `carolus_tf_broadcaster.py`, run on the Pi. Has no effect on the current SEARCH/ALIGN/APPROACH pipeline (which consumes `/pose` directly, not the TF) — relevant for validating orientation (`rosrun tf tf_echo camera_link beacon_observed`) and lays the groundwork for Phase F's tf2_ROS/EKF adoption. Can be launched independently of T3, but won't have anything to republish until T3 (the source of `/pose`) is running.

**T5 — added 2026-07-27, first hardware session run the same day (journal entry 10).** Runs `beacon_docking.py` directly on the lab PC (like T3 — no SDK connection of its own, so no conflict with `rm_cam_beacon.py`'s single Pi-side SDK connection). Fixed-pose docking behavior relative to the beacon, currently the simplified `SIMPLE_APPROACH_ONLY` pipeline (gimbal align → chassis align via `yaw_rel` → drive to `DOCK_DISTANCE_M`, 0.20m). Controlled by the **DOCKING BALISE** panel: `CALIBRATE` + `CAL STEP 2` (2-click calibration — step 1 measures face-on, step 2 measures after the user pivots ~30-45°, no time pressure between them; the original single-shot 20s-timer design was unusable in practice and was replaced), `START`, `ABORT`. `BEACON_YAW_VALIDATED` is **currently `True`** on this robot (`BEACON_YAW_SIGN=+1.0`, `BEACON_YAW_OFFSET_DEG=+2.4`, calibrated 2026-07-27). Commands go through `cam_view_helper.py` (`/carolus/dock`, same relay pattern as `RECENTER`), status comes back via the `[DOCKSTATUS]` log line parsed the same way as `[BEACON]`. T5 only unlocks its buttons after seeing its own first `[DOCKSTATUS]` line (not just after the process starts) — sending a command too early used to be silently dropped (BUG-076, fixed). **Validated on hardware with mixed results**: one collision (BUG-078, root-caused and fixed — a silent gimbal-probe failure let the chassis align to the wrong reference), one clean approach that ended up close to the beacon but off-center (open issue: nothing corrects chassis heading during/after the drive, only the camera tracks — fix identified, not yet implemented, see roadmap next-session item 12).

**Logs — reworked 2026-07-20: one tab per terminal, extended 2026-07-27 to T5.** All 5 terminals (T1-T5) are **fully integrated** (their output is captured and shown in the app, no external gnome-terminal window). The Logs area is a `ttk.Notebook` (`T1 roscore+Pi`, `T2 Camera+Beacon`, `T3 Carolus Astrobee`, `T4 TF Broadcaster`, `T5 Docking`), each terminal writes only to its own tab — no more mixing in a single box. Global event messages (AUTO/MANUAL mode, kill, etc.) still get broadcast to all tabs at once. The "Copy logs" button now copies only the active tab's content. A change needed to launch T1 in integrated mode: pre-checked that `sudo` on the Pi doesn't ask for a password (`sudo -n true`), otherwise T1's `sudo ip link set eth1 up` command would block silently in the pipe.

**Kill**: cancels any pending waits (`wait_for_roscore` / `wait_for_camera`) then kills the SSH and local processes. Reaps OS zombies (`proc.wait()`). A partial Kill (a row's Kill button) only kills that target and its downstream processes.

---

### Live map (right-hand panel)

The `_LiveMapCanvas` panel shown to the right of all the controls. 520×420 px canvas (26 cells × 20 px), grid background, obstacle blocks from the JSON.

| Element | Description |
|---|---|
| Blue square (■▲) | Real-time robot position + heading (updated via `[POS]` + `[ATTI]`) |
| Yellow dot | Most recent detected beacon position (updated via `[BEACONPOS]`). **Auto-disappears** if there's been no detection for 1.5s (`BEACON_FRESH_S`) — serves as a visual indicator that detection is active without having to read the T3 logs. |
| Gray blocks | Obstacles loaded from `mapv1.json` at startup |
| **Load map** button | Open another JSON file from disk |

**Auto-load:** `mapv1.json` (project root) auto-loaded 500ms after the launcher starts.

**Axis convention (same as map_editor):**
- EP x (forward/north) → up on the canvas
- EP y (right/east) → right on the canvas

---

### LOCATE mode

The **LOCATE** button in the control row (gold-yellow when active). Publishes `"LOCATE"` on `/carolus/mode`.

**Behavior in LOCATE mode:**
- The gimbal sweep continues (same as AUTO/SEARCH).
- As soon as the beacon is visible: the robot stops (`stop_gimbal` + `stop_chassis`), position published to the live map.
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
- **BEACON MINIMAP** (a small 100×100 canvas): a dot represents the beacon's position *within the camera frame* (green if centered within ±3°, orange otherwise) — distinct from the existing robot/grid live map, which shows position within the lab.
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

**Keyboard focus:** T1 and T3 open external `gnome-terminal` windows that steal the system's keyboard focus. If ZQSD stops responding after launching T1/T3, just **hover the mouse over the launcher or map editor window** — focus is reclaimed automatically (`<Enter>` → `focus_set()`, fixed 2026-07-01).

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

**Session log on disk (added 2026-07-31).** Every line written to any tab is also appended to `shortcuts/logs/session-YYYY-MM-DD-HH-MM-SS.log`, one file per launcher start, each line prefixed with the time and the originating tab (`T1`..`T5`, or `--` for a global event broadcast to all tabs). Before this, logs lived only in the tkinter widgets: closing the launcher lost them, and each tab is truncated to 300 lines anyway. The concrete cost showed up on 2026-07-31 — the question *"is LOCK still ticking during a docking run?"* (point 4 of `research-log/21-points-a-creuser/`) was unanswerable even though a run that would have answered it had already happened; the logs simply weren't kept. Grep by tab to answer that class of question directly:

```bash
grep '\[T2\].*\[LOCK\]' shortcuts/logs/session-*.log     # was LOCK active during the run?
grep '\[T5\]' shortcuts/logs/session-*.log                # everything the docking node said
```

Best-effort by construction: a write error never brings the GUI down. But it is **not silent** — if the file cannot be opened, a line goes to stderr saying so, because a log that isn't written without saying so is worse than no log at all (you believe you have the data and you don't). `shortcuts/logs/` is gitignored.

---

## `lever_arm_bearing.py`

**Quoi** — calcule, pour un decalage `d` entre centre de rotation du chassis et centre optique de la camera, le changement apparent de gisement d'une cible fixe apres une rotation sur place.

**Pourquoi** — le 2026-07-30, un run ALIGN a montre le gisement de la balise passer de +6.3 a +18.2 deg (environ +12) sur ~97 deg de rotation chassis, alors que `yaw_ground` restait plat (-142.3 -> -142.4) : la camera n'a donc PAS tourne dans le repere monde, et le changement ne peut pas venir d'une rotation camera. Deux causes candidates (point 2 de `research-log/21-points-a-creuser/`) : geometrie du bras de levier, ou glissement lateral Mecanum reel. Ce script chiffre la premiere pour savoir s'il reste quelque chose a expliquer.

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

**What:** deploys `rm_cam_beacon.py` to the Pi (SCP) and verifies integrity via a local vs. remote md5 checksum + `ast.parse` on the Pi side.

**Why:** makes deployment reliable in one command (instead of a manual SCP + eyeballed check), with guardrails: refuses to send if the file doesn't compile locally, if the Pi is unreachable, or if the checksum differs after copying.

**Usage:**
```bash
bash shortcuts/deploy_pi.sh
```

| Step | Check |
|---|---|
| 0 | local `py_compile` — abort on a syntax error |
| 1 | Pi reachable (SSH ConnectTimeout 5s) — abort otherwise |
| 2 | `scp rm_cam_beacon.py` → `/home/ubuntu/carolus_ws/.../rm_cam_beacon.py` |
| 3 | local md5 == remote md5 — abort if different |
| 4 | `ast.parse` on the Pi side (warn only) |

**Note:** only `rm_cam_beacon.py` runs on the Pi. `carolus_launcher.py` and `cam_view_helper.py` run on the lab PC → picked up on the launcher's next launch (no SCP needed). After deploying: restart T2 (Kill T2 → `> 2 Camera+Beacon`) to load the new code.

**Expected:** `checksum identique -> deploiement verifie`, then a reminder to restart T2.

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

## `map_editor.py`

**What:** a 2D map editor (a separate Toplevel window) — a 26×21-cell grid (10.4m×8.4m, 1 cell = 40 cm ≈ the S1's footprint), full/half/quarter blocks, a zone tool (drag fill), oriented half-block beacons, a grid-snapped robot overlay.

**Why:** lets you map the lab's real obstacles (chairs, desks) before a session, position the beacon, and visualize the robot's position live. The exported map JSON is loaded by `map_collision.py` on the Pi for obstacle avoidance.

**Usage:** opened from the "MAP EDITOR" button in `carolus_launcher.py` (no direct launch).

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
