# Shortcuts — Carolus / RoboMaster

Shortcut scripts for frequent operations. Common prerequisites: robot powered on (double chime), Pi at `192.168.0.103`.

---

## `carolus_launcher.py`

**What:** Tkinter GUI (dark theme, English UI), sequenced launch of 5 stages
on the Pi, MANUAL-only ZQSD (chassis) + numpad (gimbal) piloting, a live
state dashboard, one log tab per stage, camera and blob-detection preview
panels. Full pitch, feature tour and design rationale (why MANUAL-only is
the permanent default, why unused features get deleted rather than kept,
why ZQSD stayed ZQSD): [`docs/carolus-launcher.md`](../docs/carolus-launcher.md).

**Why:** launches the whole stack with no commands to type; live feedback
(dashboard, camera, logs) without leaving the window.

**Usage:**
```bash
python3 shortcuts/carolus_launcher.py
```

| Button | What runs | Unlocked when |
|---|---|---|
| 1 · roscore + Pi | integrated SSH -> `eth1 up` + `roscore` | port 11311 open (60s timeout) |
| 2 · Camera + Beacon | integrated SSH -> `rm_cam_beacon.py` + `cam_view_helper.py` | `/camera/color/image_raw` published (60s timeout) |
| 3 · Carolus **[Pi]** | SSH -> `roslaunch carolus_node testcarolus.launch ubuntu2204_preload:=false` | manual |
| 4 · TF Broadcaster | integrated SSH -> `carolus_tf_broadcaster.py` on the Pi | manual, near-instant |
| 5 · MINS (simulation, Pi) | SSH -> `roslaunch mins simulation.launch` in `~/mins_sandbox_ws` | independent — own roscore, not killed by KILL ALL |

**Current controls and panels**, verified against the running code, not
carried over from an earlier version:
- **ZQSD** (chassis, vx=0.20 m/s, wz=20°/s) and **numpad 8/4/5/6/2** (gimbal,
  pitch=30°/s, yaw=40°/s), auto-stop on key release. Hover the window to
  reclaim keyboard focus if input stops responding.
- **LOCK** (control row, period field in seconds): periodic beacon
  re-centering via a single `gimbal.move()` per tick, yaw only, active only
  in MANUAL. Numpad gimbal control is ignored while LOCK is on.
- **CAM PREVIEW** / **BLOB VIEW** (both OFF by default — each toggles its
  own ROS subscription, so OFF frees real bandwidth): camera thumbnail and
  the black-and-white blob-detection overlay, resizable up to 3x.
- **Beacon DETECTED/LOST indicator** and the **brightness-signature
  readout** (saturated-pixel count + hue, judged against
  `beacon_reference.yaml`) — catches the case where an over-bright beacon
  reads as "0 contours found", identical to no beacon in view.
- **Dashboard**: robot state (SEARCH/ALIGN/APPROACH/STOP), depth, battery,
  and a separate Raspberry Pi panel (temperature/load/RAM, SSH probe on its
  own thread with a hard timeout so a hung Pi cannot freeze the GUI).
- **Session log on disk** — every tab's output also lands in
  `shortcuts/logs/session-YYYY-MM-DD-HH-MM-SS.log`, tagged by originating
  tab, so a specific terminal can be grepped back after the fact:
  ```bash
  grep '\[T2\].*\[LOCK\]' shortcuts/logs/session-*.log     # was LOCK active?
  grep '\[T3\]' shortcuts/logs/session-*.log                 # everything Carolus said
  ```
- **Kill** goes through `remote_kill()` (bracketed `pkill` patterns, so the
  SSH shell running the kill command can't match and kill itself) and
  re-reads the Pi afterwards, logging a survivor instead of failing silently.

**Removed, not present in the running GUI**: MODE AUTO, LOCATE, WHEELS
tilt, BEACON MINIMAP, RECENTER CAM, and the whole BEACON DOCKING tab —
`beacon_docking.py` itself is untouched and still runs standalone. The
lab-floor live map is also gone (`archive/mapv1-2026-08-10/` holds the
extracted class if it's ever wanted back).

**Expected:** with a beacon in view and the first four stages running,
`BEACON: DETECTED` lights up green, the dashboard shows a live depth
reading, and the log tabs scroll with each stage's own output.

Full change history — every dated addition, removal and bug fix from
2026-07-20 onward — lives in `research-log/journal.md` only, not here.
**Rewritten 2026-09-05** from a 27,628-character entry that had accumulated
into an undated-feeling changelog and, in two places, described LOCATE and
BEACON MINIMAP as currently active when both were removed 2026-08-14 —
checked against the running source before writing this version, not
assumed from the old text.


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

## `beacon_brightness_live.py`

**What:** live meter for the beacon's LED intensity — saturated-pixel count, hue, and detection rate, on one refreshing line.

**Why:** the intensity knob is physical and has no readout, and getting it wrong is not a mild degradation. On 2026-08-17, too-bright made the four LEDs bloom into one merged blob of **88 628 saturated px (9.6% of the frame)**, which fails `max_area` and `min_circularity` — and Carolus then logs `Not enough blobs with required circularity` followed by `0 contours found`. **That wording points you at "beacon missing" when the truth is the exact opposite.** This meter shows you which.

**Usage** (from the lab PC — needs `-t` for the refreshing line):
```bash
ssh -t -i ~/.ssh/carolus_nopass ubuntu@192.168.0.103 'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311; python3 /tmp/beacon_brightness_live.py'
```

**Expected:** `sat px 13613  GOOD  hue 103.6  RATE 5.4 Hz ok`

**Tune to maximise RATE, not to hit a pixel target.** The first version of this tool showed brightness only and that was the wrong target — a setting that looked plausible by pixel count was delivering 1.5 Hz. Reference bands: ≥8 Hz excellent, 4–8 Hz usable, <4 Hz do not measure on it. The known-good signature is recorded in `carolus_ws/src/carolus_node/config/beacon_reference.yaml`.

> **The LEDs are PWM-driven**, established 2026-08-17: across 333 frames the saturated count ranged 0–37 182, and a frame reading exactly zero is the camera catching the off phase. So dimming does not merely shrink the blobs — it lowers the detection *rate*, because more frames land mid-off and fail outright. That is the real trade-off when turning the knob down.

---

## 1.4b measurement tools (`q14b_capture.py`, `q14b_bracket.py`)

**What:** capture and analyse the quaternion-validation readings (`plan-fin-de-stage.md` item 1.4b).

**Why:** three separate failure modes made this measurement impossible by hand, each found the hard way on 2026-08-14, and each is now handled in code rather than in the operator's head.
- **Stale TF reads.** A freehand attempt produced four consecutive "samples" that were one cached transform. `q14b_capture.py` de-duplicates on `header.stamp` — a re-read of the same transform is not a measurement.
- **P4P ambiguity.** On a stationary rig, ~8-12% of samples land on the solver's alternate near-planar solution (roll +41 vs +0.5). A mean over both describes neither, so it uses the median with an interquartile filter and *reports* the outlier fraction. Above 35% it refuses the reading outright.
- **Chassis drift.** The chassis rotates ~9°/min on its own, so the camera has turned by an unknown amount between two captures. `q14b_bracket.py` implements the standard fix: reference → rotated → reference again, measure the drift from the two references, subtract it from the rotated reading.

**Usage** (on the Pi, ROS sourced):
```bash
python3 /tmp/q14b_capture.py REF1 15      # beacon straight, centred
python3 /tmp/q14b_capture.py ROT  15      # after rotating it in place
python3 /tmp/q14b_capture.py REF2 15      # after returning it to straight
# then, on the lab PC, paste the three COPYME lines:
python3 shortcuts/q14b_bracket.py "REF1 …" "ROT …" "REF2 …"
```

**Expected:** each capture prints `STABLE … COPYME <label> <roll> <pitch> <yaw> <t>`, or refuses with a reason. The bracket step prints the drift-corrected rotation per axis and names the dominant one.

**The stability bar is 5°, set from what the test needs** — 1.4b reads the *sign* of 30-90° rotations, so 5° leaves a 6:1 margin. An earlier 1° bar came from six `tf_echo` reads that agreed to 0.5°, but those were almost certainly one cached transform; this rig holds 0.1-0.3° on roll/pitch and ~2° on yaw.

---

## `optical_drift_observer.py`

**What:** measures whether the camera is rotating in the world, using only `/camera/color/image_raw`.

**Why:** during a launch bisection the usual instruments are part of what is being tested — `gimbal_yaw_rel` needs T2, and the beacon bearing normally needs T3's `/pose`. This needs neither, so a stage can be measured without the measurement presupposing the stage. Brightness-centroid only, numpy, no cv2/cv_bridge (which drag in the OpenCV ABI collisions behind BUG-101/102/108).

**Usage:** `OBS_THRESH=252 python3 /tmp/optical_drift_observer.py 100 "label" [csv_path]` — threshold needs calibrating per lighting; 252 isolated the 4 saturated LEDs (~118 px) against 321 px at 230. `csv_path` defaults to `/tmp/optical_drift_<label>_<timestamp>.csv`.

**Expected:** one line naming the CSV it wrote, then `n=` samples with a rate, then the drift as `px/min` and `deg/min` with an R², ending `DRIFTING` or `stable`. `INSUFFICIENT DATA` means the beacon is not bright enough or the camera topic is silent — retune the threshold rather than trusting a long run that ends that way.

**Promoted 2026-08-31 from bisection tool to primary instrument** for Protocol 25 (`research-log/02-protocoles/protocoles-terrain.md`), after `chassis.sub_attitude` was disqualified as a drift-measurement channel. Two changes for that role:
- **Raw samples now go to CSV**, written *before* the summary is computed, so a session can be re-analysed against a different calibration without re-running the robot — and so a divide-by-zero in a summary line cannot lose the expensive part. The CSV carries the raw pixel centroid, not only the normalised position, which is exactly what makes a later recalibration possible.
- **Pixels-per-degree is now the measured 12.53** (2026-08-13/14: a commanded 14.6° gimbal move produced 182.9 px), overriding the previous "a frame half-width is ~50°" estimate. Override with `OBS_PX_PER_DEG=`. The two agree to ~2 % (640 px / 12.53 = 51.1°) — the point is traceability to a measurement, not that the number moved.

**Self-tested offline 2026-08-31**, without ROS or a robot: fed a synthetic 12.53 px drift across exactly 60 s, it reports exactly `1.0000 deg/min` with R²=1.000000; fed a flat signal it reports exactly `0.000000 px/min`. The negative control is the point — a drift detector that cannot report "no drift" would be useless for Protocol 25's condition 3.

⚠️ **Not covered by `deploy_pi.sh`**, which syncs `robomaster_cam`/`carolus_node`/`libuvgs_astrobee`/`ff_msgs` and not `shortcuts/`. Copy it across by hand: `scp shortcuts/optical_drift_observer.py <pi>:/tmp/`.

---

## `pyaudit.py`

**What:** AST audit of the project's Python — unused imports, unreferenced top-level definitions, `except: pass`, bare `except:`, mutable default arguments.

**Why:** pyflakes, ruff, vulture and pylint are **none of them installed here**, and the check that reported them present was itself broken (`python3 -m x --version | head -1 && echo OK` prints OK on empty input). Rather than depend on a tool that may not exist, this walks the AST directly. **It self-tests against a planted defect before reporting** — the same rule `doc_check.py` follows.

**Usage:** `python3 shortcuts/pyaudit.py $(find carolus_ws/src shortcuts -name '*.py' ...)`

**Expected:** per-file findings then totals; `clean` if nothing. Triage matters more than the count — 47 `except: pass` were found on 2026-08-14 and only three were real defects (silent motion-stop failures), the rest being telemetry and teardown where best-effort is correct.

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

**Files saved:** `carolus_launcher.py`, `cam_view_helper.py`, `map_editor.py`, `rm_cam_beacon.py`, **`beacon_docking.py`**, **`beacon_absolute_pose.py`**, `testcarolus.launch`, `carolus_node/config/robomaster_s1.yaml`, **`optical_drift_observer.py`**, plus the workspace's 5 `CMakeLists.txt` files (`src/`, `libuvgs_astrobee/`, `ff_msgs/`, `robomaster_cam/`, `carolus_node/`) and **`libuvgs_astrobee/src/carolus_astrobee.cpp`, `ceresP4P.cpp`, `pose_est.cpp`** — the `CMakeLists.txt` set was added 2026-07-13 to cover the CLAUDE.md rule listing them as critical files; the two docking scripts were added **2026-07-28**; `robomaster_s1.yaml` was added 2026-08-14, on discovering it was absent at the exact moment it was about to be edited (the `min_area` retune); **the three C++ files were added 2026-08-18**, on the same discovery repeating a fourth time — this time at the start of the core-extraction rework (Hector's ROS-portability request), the most structurally significant change these files have had; **`optical_drift_observer.py` was added 2026-08-31 — for once *before* the edit rather than after it bit**, on noticing it was absent at the moment it was becoming Protocol 25's primary instrument; and **`carolus_node/config/logitech_1080p.yaml` and `carolus_node/config/robomaster_s1_longrange.yaml` were added 2026-09-04**, on the discovery repeating a fifth time — `robomaster_s1.yaml` sat in `FILES` in the very same directory while its two neighbours did not, found at the exact moment `logitech_1080p.yaml` became the file being patched for BUG-138.

> **Why the omission keeps recurring:** every one of these additions was caught the same way — a file became the active target of real work while absent from `FILES`, so the backup gave a false sense of a safety net. A backup script that silently omits the file you are actually editing is worse than no script. **Rule, restated because it keeps needing restating: any source file under active modification must be in `FILES` before the work starts, not after it bites.**

**Expected:** a `saves/YYYY-MM-DD-HH-MM/` folder created with **37** files + `NOTE.txt`. *(Was 35 from 2026-08-31 until 2026-09-04, when the two config files above were added. Verified rather than assumed: `FILES` holds 37 entries, and the save made 2026-09-04 (`2026-09-04-15-56`) contains exactly 37 plus `NOTE.txt`. Corrected twice before — 2026-08-21 (said 35 against a real 34) and implicitly by every addition above — which is why the number is re-checked against an actual save each time it changes rather than incremented on faith.)*

**The ROS2 package has moved three times; the list currently points at the third
location.** `carolus_ws/src/carolus_ros2/` → repository root (2026-08-19: a ROS2
(ament) package inside a catkin source space makes `catkin_make` refuse to
configure the *entire* workspace) → **`raspberry5-carolus-ros2/carolus_ros2/`**
(2026-08-20, made self-contained: the C++ core is copied in, no relative
reference back into `carolus_ws/`). Those copies are an independent **snapshot**
and are backed up as their own entries, not assumed to mirror
`carolus_ws/src/libuvgs_astrobee/`. `pose_filter.{hpp,cpp}` were added the moment
they were created, when the FIFO outlier filter was extracted out of the ROS1
node into `carolus_core`. *(This paragraph was itself one move stale until
2026-08-21 — it still described the root as the current location, two days after
the second move; the script's own comments were correct throughout.)*
Both matter here for the same reason: the copy loop skips any path that does not
exist, so a stale entry produces a *silently smaller* backup rather than an
error — the run still prints "Sauvegarde ->" and looks like it worked. If you
move or rename a tracked file, fix this list in the same commit, and check the
file count above against what actually landed.

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

**What** — checks **both LaTeX manuals** against the code they document: every fully-qualified path a manual names is confirmed to still exist on disk, and every changed/named file is checked for whether the manual mentions it (and where, so the review is fast).

**Covers BOTH manuals since 2026-09-05** — `overleaf/technical.tex` and `raspberry5-carolus-ros2/technical-ros2.tex`. Until then it read the first only, so a change touching just the ROS2 tree produced a clean run that meant nothing: the same reassuring zero this script exists to eliminate, reintroduced by scope rather than by escaping.

**Also enforces the no-bug-numbers rule since 2026-09-05.** Both manuals forbid `BUG-XXX` labels — a reader holding only the repository cannot resolve them. The rule had broken twice (seven references removed 2026-08-25, thirteen more found 2026-09-05), both times while writing up a real bug, which is exactly when reaching for its number is most automatic. The check greps both files and **exits 1** on any hit, naming file and line. Verified by sabotage: injecting `BUG-999` makes it fail and report that line; removing it makes it pass again.

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

## `tear_check.py`

**What** — measures whether the camera produces **torn frames while the chassis is driving** (BUG-105). Samples frames in three conditions — idle, rotating in place, driving forward/back — and scores each for a tear seam. Derives its own threshold from the idle baseline measured in the same run, so it adapts to the actual lighting instead of inheriting a number from a differently-lit session.

**Why** — 2026-08-12 tested idle and gimbal motion and found nothing, but never tested **chassis** motion, which is exactly the condition the 2.3 rectangular-motion rosbag runs under. There a torn frame is not a shrugged-off bad reading: it is ~1000 spurious contours inside a continuous trajectory MINS will process as real.

**Usage**
```bash
# on the Pi, with roscore + rm_cam_beacon.py running and the robot ON THE FLOOR
python3 /tmp/tear_check.py
```

**Safety** — the robot drives itself unattended, so motion is **oscillatory**: the sign flips every few seconds, keeping net displacement near zero (max ~18 cm forward excursion, rotation in place). `rm_cam_beacon.py`'s own 0.5 s deadman brakes the chassis if the script dies.

**Result 2026-08-13** — **0/60 torn rotating, 0/60 driving, 0/60 idle**, detector validated 6/6 against synthetic seams. `[MANUAL-DRIVE]` fired 13 times, confirming the chassis genuinely moved rather than the test silently doing nothing.

**Two traps this tool exists to warn about, both hit while building it:**
- **A brightness detector is not a tear detector.** The first version counted bright-value contours, reproducing the 2026-08-12 signature (~1000 contours). That signature was measured with the lab lights **off**; with them on it fires on 100% of frames in *every* condition including idle. It was measuring illumination.
- **Validate the negative, and validate it correctly.** A "0 torn frames" result means nothing until the detector is shown to catch a real seam. The first self-test spliced two frames of a static scene grabbed 1 s apart — near-identical, so the splice had no seam and the detector correctly saw nothing, which *looked* like detector failure. Splice genuinely different content instead (roll one frame horizontally, or slew the gimbal between grabs).

---

## `ros2_sync_check.sh`

**What** — compares the 10 shared core files (`beacon_detector`, `ceresP4P`, `pose_est`, `pose_filter`, `carolus_types`, `compute_jacobian`) between the ROS1 workspace and their copy in `raspberry5-carolus-ros2/`, and fails if any has drifted.

**Why** — `raspberry5-carolus-ros2/` deliberately holds its *own copy* of that core rather than referencing `carolus_ws/` by path, so the folder stays portable and hand-over-able (decision of 2026-08-20, `CLAUDE.md`'s "ROS2 manual" section). The cost is that a fix on one side never reaches the other and **nothing warns you**. This script is that warning, and exists so the sync rule is mechanically checkable instead of relying on someone remembering it.

**This is also why there is no third tree.** A `test_ros/` folder for the ROS1-vs-ROS2 benchmark was considered and rejected 2026-09-04: a third copy of the core would need its own drift check on top of this one, for a comparison that belongs in documentation, not in code that can silently diverge. See `technical-ros2.tex`'s "Where this node genuinely differs from the ROS1 one" section.

**Usage**
```bash
bash shortcuts/ros2_sync_check.sh
```

**Expected** — one line per file (`in sync` / `DRIFTED` / `MISSING`), then `RESULT: in sync. 10 files compared, all byte-identical.` Exit 0 in sync, 1 on any drift or missing file, so it works as a pre-push or CI gate.

Deliberately does **not** compare `carolus_astrobee.cpp` (ROS1 node) against `carolus_node_ros2.cpp` (ROS2 wrapper) — those are middleware-specific by design and *supposed* to differ; comparing them would produce a permanent false alarm.

**Verified to actually fail**, both ways, restoring byte-identical each time: appending one comment line to the ROS2 copy of `pose_filter.cpp` produced `DRIFTED pose_filter.cpp` and exit 1; removing `ceresP4P.cpp` entirely produced `MISSING (ROS2)` with the expected path and exit 1. Normal state re-confirmed exit 0 afterwards.

On drift, the script deliberately refuses to say which side is right: usually the ROS1 node is the original, but not always — BUG-125/126 (2026-08-20, the raw-YUV crash and the RGB/BGR hue swap) were both found and fixed on the ROS2 side first.

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

**Expected** — one line per package (`OK` / `DRIFT` / `MALFORMED XML` / `skipped` / `ros2` / `MISSING`), then, with `--resolve`, either `all N keys resolve` or a list of unresolvable keys. Exit 0 clean, 1 on any problem, so it is usable as a pre-push or CI gate.

**2026-08-20 (2) — the stale-path check caught its own list going stale.** `carolus_ros2` moved a second time the same day (repository root → `raspberry5-carolus-ros2/`), and the very next run reported `MISSING carolus_ros2 (listed in EXTRA_PACKAGE_DIRS, no package.xml there -- moved? renamed?)` with exit 1 rather than quietly dropping it from coverage. Path corrected; back to `RESULT: clean`. This is the failure mode the list was added for, reproduced and caught within hours of being written.

**2026-08-20 — scope extended beyond `carolus_ws/src`.** The script scanned only that directory, so when `carolus_ros2/` moved to the repository root (BUG-122: an ament package inside a catkin source space makes `catkin_make` refuse to configure the whole workspace) it dropped out of coverage while the script went on printing `RESULT: clean` — one package fewer than the day before, with nothing saying so. An `EXTRA_PACKAGE_DIRS` list now pulls in out-of-tree packages, and a missing entry there is counted as a **problem** (exit 1), not merely printed: the first version of this very fix printed `MISSING` and still exited 0, which is the same silent-pass defect it was written to close. ROS2 packages are checked structurally (manifest well-formed, `find_package()` matched against declarations) but are **visibly** excluded from the `ubuntu:focal` resolution with the reason printed — `rclcpp` and friends do not exist under Noetic and would fail for a meaningless reason. Both new paths were negative-tested: an undeclared `find_package(Boost)` in `carolus_ros2` produces `DRIFT`, and moving the package away produces `RESULT: 1 problem(s)` with exit 1.

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

**What:** a separate process with four roles — (1) a PNG camera thumbnail ~20 Hz (plain resize, no overlay — the reticle/beacon-marker HUD it carried 2026-07-23 to 2026-08-14 was removed, see the launcher's own changelog above), (2) a second PNG thumbnail of `/postprocessed/image` (Carolus's blob-detection view, 2026-08-14), (3) a GUI-keyboard → ROS chassis-topic gateway, (4) a GUI-numpad → ROS gimbal-topic gateway.

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

> `RECENTER`, `DOCK …` and `WHEELS …` were removed from this relay on 2026-08-14 along with their launcher buttons. `MODE` now only ever carries `MANUAL`.

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

**Why** — created 2026-08-11 after reviewing the LEO/LIMO documents Hector sent: they use the exact same MATLAB checkerboard method, and this project's own manual already documented the procedure (MATLAB's Camera Calibration app) without ever having a way to actually save frames from the live camera stream — the values currently loaded (`camera_info.yaml`) were a pragmatic stand-in, never the output of this procedure. No existing script in this project wrote a camera frame to disk.

**Usage**
```bash
python3 shortcuts/capture_checkerboard.py [output_dir]   # default: data/checkerboard/
```
Live preview window. `s` saves the current frame as `checkerboard_NNN.png`; `q` quits. Aim for at least 15 frames, checkerboard tilted more than 45° from the optical axis in each, varied position/orientation across the field of view, occupying roughly 20–25% of the frame — per the manual's own already-written recommendation.

**Expected** — a folder of PNG frames ready to upload into MATLAB's Camera Calibration app (Apps tab), enable tangential distortion + three radial distortion coefficients, run, and transcribe the exported `fx`/`fy`/`cx`/`cy`/distortion into `testcarolus.launch`.

---

## `capture_checkerboard_ros2.py`

**What** — the ROS2/Logitech-C920 equivalent of `capture_checkerboard.py` above. Subscribes to `/image_raw` (`usb_cam`'s topic, not `/camera/color/image_raw`), keypress-triggered (Enter saves the current frame, `q` quits), saves PPM — no cv2/cv_bridge, no GUI window needed.

**Why** — created 2026-09-02 for Hector's mail-13 objective 2 (<1cm accuracy at 2m, MATLAB calibration): the Logitech C920 has never had a real calibration run against it — `logitech_1080p.yaml`'s current values are Hector's own approximate estimate (2026-08-17), not the output of this procedure. `capture_checkerboard.py` cannot serve this: it's ROS1 (`rospy`), needs `cv2`/`cv_bridge`, subscribes to the RoboMaster's own onboard-camera topic, and pops a GUI window — none of which apply to a headless ROS2/Jazzy machine reached over SSH. This is a different tool for a different physical camera, not a replacement.

**Usage**
```bash
python3 shortcuts/capture_checkerboard_ros2.py [out_dir]   # default: data/checkerboard/logitech_capture
# On a remote machine without this project checked out (this session's actual
# case, captured on a separate Dell over SSH), pass an explicit out_dir and
# move the result into data/checkerboard/<name>/ afterward -- do not leave it
# loose in $HOME, that was a real mistake caught and corrected 2026-09-02.
```
Run interactively, in your own terminal (needs real stdin — piping it through a non-interactive SSH command doesn't work). Press Enter to save the live frame, move the board, Enter again; `q` + Enter to stop. Same targets as the manual's own procedure: ≥10 frames (15+ better), board filling 20-25% of the frame, tilted >45° off the optical axis, varied positions.

**Do not enable "three radial distortion coefficients" for this camera** — that setting is specifically justified in `technical.tex` for the RoboMaster's own wider-FOV onboard camera, not the C920. Use MATLAB's default 2-coefficient radial model, matching `logitech_1080p.yaml`'s existing 4-slot `distortion: [k1, k2, p1, p2]` format.

**Expected** — a folder of PPM frames MATLAB's Camera Calibration app reads natively. First real run, 2026-09-02: 200 frames captured (more than needed — variety matters more than count past ~15-20; let MATLAB's own detection and per-image error review do the filtering rather than pre-selecting by hand).

**Needs the robot powered on** and camera streaming. **Corrected 2026-08-25**: this project has nothing to print — the checkerboard is on the back of the project's own beacon (`overleaf/technical.tex` §"Recommended method", confirmed against the manual, not assumed). Only use a separate printed checkerboard if calibrating a different camera/beacon combination.

**BUG-102, fixed 2026-08-11 (first real run crashed):** this lab PC has a numpy 2.2.6 install shadowing the system numpy `cv2` needs, and a system `cv_bridge` linked against a different OpenCV build than `cv2` itself (same defect class as BUG-101, same day) — both are corrected automatically by a self-re-exec guard at the top of the script; the plain command above is unaffected and needs no extra flags.

---

## `watch_windup.py`

**What** — tails the T2 log during a long unattended run and raises an ALERT the moment BUG-116's uncommanded wheel motion starts, with a heartbeat every ~5 min so a quiet session is distinguishable from a dead watcher.

**Why** — BUG-116 (wheels ramping up with no command sent) is intermittent and has never been caught in the act, so its cause is still unattributed. A fixed-magnitude trigger is useless here: the project's own logs show ±16 rpm of idle noise. This watches for the *sustained same-direction ramp* that is the actual documented signature (−3 → −84 rpm over ~90 s), on ≥3 of 4 wheels, and reports whether any `[MANUAL-DRIVE]` fired in the same window — which rules our own command loop in or out for that window rather than leaving it ambiguous.

**Usage**
```bash
python3 shortcuts/watch_windup.py <t2_logfile> [uptime_zero_epoch] [t2_ssh_pid]
```

**Expected** — `HEARTBEAT` lines every ~5 min showing uptime, last ESC/POS reading, and whether the chassis was commanded; `ALERT` with the exact uptime if a ramp starts; `[CRITICAL]` if T2 dies, so a robot power-down is never mistaken for a quiet period.

**Validated before use, not just written**: 0 false alerts over 20 simulated runs at 1 h/3 h/6 h/10 h against this project's own documented idle-noise band, while still catching the documented real signature. Run for real 2026-08-18: 2 h 22 min unattended, zero alerts, ended cleanly on the user's power-down.

**Moved into `shortcuts/` on 2026-08-21** — it had been living in a session scratchpad, which is wiped between sessions, while the plan called for reusing it. That was a real risk of losing a validated tool.

---

## `bench_carolus_rate.py`

**What** — measures Carolus-ROS2's maximum update rate by feeding it synthetic frames faster than any camera can, then comparing `/pose`'s output rate against the harness's own achieved rate.

**Why** — Hector asked (2026-08-22) what Carolus's max rate is, whether the container costs performance, and what to expect on the Pi 5. None of it was answerable: the C920 offers only 10 FPS at 1280x720, so every run this project had ever done was input-bound and "10 Hz" measured the camera, not Carolus.

**Usage**
```bash
# terminal 1
ros2 run carolus_ros2 carolus_node --ros-args -p image_threshold:=150
# terminal 2
python3 shortcuts/bench_carolus_rate.py --mode beacon --duration 30 --label "what this run is"

python3 shortcuts/bench_carolus_rate.py --selftest    # no ROS needed
```

`--mode beacon` projects four synthetic blobs and exercises the full path including the Ceres solve — this is the number that answers "Carolus's update rate". `--mode empty` sends a black frame, rejected at the contour stage; the gap between the two is the solver's share. `/pose` is never published in `empty` mode (correct behaviour), so that path is counted from the node's own `Time to find contours` log lines instead.

**Expected** — a published rate, a `/pose` rate, and a verdict line. **The verdict is the point.** The obvious way to build this tool measures the *publisher* whenever the publisher is slower than the node, and reports a harness limit as if it were a Carolus limit. So the script measures its own rate too and **refuses to report a maximum** when `/pose` comes within 15 % of the input rate, printing `INPUT-BOUND ... this is a LOWER BOUND, not a maximum` instead. `--selftest` exercises that refusal on seven cases including both edges of the band, with no ROS involved.

It earned its keep on first use: a run showed `/pose` at 238 Hz against 165 Hz of input and returned `IMPLAUSIBLE -- check for a second publisher`. A container from an earlier configuration was still running its own node, and DDS discovery was crossing Docker's default bridge. That number would otherwise have been recorded as a result.

**Measured 2026-08-22**, 1280x720, beacon mode, 30 s after a 3 s discarded warm-up:

| Configuration | `/pose` | Harness | Headroom |
|---|---|---|---|
| Lab PC, Humble **native**, x86\_64 | **264.9 Hz** | 657.6 Hz | 40 % |
| Lab PC, Humble **in a container**, x86\_64 | **266.6 Hz** | 646.8 Hz | 41 % |
| Lab PC, Jazzy **in a container**, x86\_64 | **275.9 Hz** | 690.6 Hz | 40 % |
| Raspberry Pi 5, Jazzy **native**, aarch64 | **52.8 Hz** | 103.1 Hz | 51 % |
| Lab PC, Jazzy **native (QEMU/KVM VM)**, x86\_64 | 204.1 Hz | 664.9 Hz | 31 % |

Same distro either side of the container boundary (rows 1 and 2, the only pair that isolates the variable): **+0.6 %, no measurable cost**. Detection-only throughput on the native host was 458.2 Hz, with contour finding at 0.258 ms mean over 400 samples — so the sort-plus-solve stage costs about 1.6 ms per frame.

**Row 5 (the VM) is not a clean comparison against row 3** — no spare bare-metal 24.04 x86\_64 machine was available, so it ran under QEMU/KVM while the host carried unrelated concurrent load (uptime load average 1.53). The gap to row 3's 275.9 Hz is far more likely virtualisation + host contention than a real Jazzy-native-vs-container difference. Row 4 (Pi 5) is a clean, real result: 51 % headroom over its own harness, ~5× slower than the lab PC and still ~5× faster than any camera used so far.

**Includes the BUG-102 guard** (`PYTHONNOUSERSITE`, self-re-exec), same trap `capture_checkerboard.py` hits: a pip-installed numpy 2.2.6 in `~/.local` shadows the system numpy that apt's `cv2` was built against. The `cv_bridge` half of that script's guard is deliberately not copied — it is ROS1-specific.

---

## `beacon_hold.py` — superseded 2026-08-14, kept for `--status` and as a record

**What** — a gimbal visual servo that held the beacon centred in the image by commanding gimbal velocity from the LED centroid. The chassis is never commanded; only the gimbal moves. `--status` reports detection stability and commands nothing.

**Why it exists, and why it does not work.** Written 2026-08-14 to cancel the gimbal drift then attributed to BUG-111, which made any long measurement impossible (the beacon left the usable frame after ~4 min and Carolus logged "Not enough blobs < 4" 1120 times in a row). It was superseded the same day: it servos on **its own** blob detector, whose count swung between 4 and 13 as room reflections came and went, so the centroid jumped and the servo chased noise — it actively moved the gimbal *off* a good position the operator had just set. The real fix was the robot mode (`chassis_lead`), which removes the drift at source instead of correcting it downstream.

**Usage**
```bash
python3 shortcuts/beacon_hold.py --status          # report only, commands nothing -- the one mode still worth running
python3 shortcuts/beacon_hold.py                   # servo (superseded, do not use for measurements)
python3 shortcuts/beacon_hold.py --duration 1800
```

**Expected** — from `--status`, a live readout of how many LEDs are actually detected and how much the centroid moves. Use it to check detection stability *before* trusting any servo or long run. A count that swings outside 4 means the scene, not the algorithm, needs fixing.

**Retained deliberately, not by neglect** — the failure is instructive and cheap to repeat otherwise: a proxy detector that disagrees with the real one is worse than no detector. Full account in `journal.md` 2026-08-14 (13).

**Documented here 2026-08-21** — the script had shipped on 2026-08-14 with an `00-index.md` row but no README section, the one combination the folder's own rule forbids.

---
