#!/usr/bin/env bash
# deploy_pi.sh - deploys to the Pi everything that RUNS there, and verifies by checksum.
#
# ============================================================================
# WHY THIS SCRIPT'S SCOPE CHANGED (2026-08-04)
# ============================================================================
# Until 2026-08-04 this script pushed exactly ONE file, rm_cam_beacon.py,
# because that was the only program running on the Pi. That is no longer true:
# Carolus itself runs on the Pi since the 2026-08-04 measurement
#     /pose on the lab PC : 2.19 Hz
#     /pose on the Pi     : 13.04 Hz   (x5.95, same beacon at 1.00 m)
# So carolus_node (launch files + YAML profiles), libuvgs_astrobee (the C++)
# and ff_msgs live on the Pi too, and nothing kept them in step: they were
# rsynced there by hand once, then drifted silently.
#
# That is this project's most expensive failure pattern. On 2026-08-04 the Pi
# was running a rm_cam_beacon.py with no /imu publisher at all (0 occurrences
# against 8 on the lab PC), and a whole diagnostic session was spent on stale
# code. So this no longer pushes "the file I just edited": it pushes everything
# that runs there, and it VERIFIES.
#
# ============================================================================
# WHAT THIS SCRIPT DELIBERATELY DOES NOT DO
# ============================================================================
# The Pi's ~/carolus_ws is a REAL git checkout (verified 2026-08-04: HEAD
# 318409b, 13 locally modified files). Two paths to the Pi therefore already
# exist -- git, and this script. We do not create a third, and we do not try to
# reconcile them automatically: the script REPORTS the Pi's git state and
# leaves the decision to the operator. A silent automatic merge would be
# exactly the class of bug this is trying to eliminate.
#
# Usage :
#   bash shortcuts/deploy_pi.sh              # dry run: report drift, change nothing
#   bash shortcuts/deploy_pi.sh --apply      # push, then verify
#   bash shortcuts/deploy_pi.sh --apply --build   # push, verify, AND rebuild on the Pi
#
# The rebuild is NOT automatic: recompiling carolus_node/libuvgs_astrobee takes
# minutes on the Pi, and a launch-file or YAML change needs none. The script
# says whether one is required; the operator decides.
#
# Prerequisites: Pi reachable (RNDIS up), SSH key ~/.ssh/carolus_nopass.
# ============================================================================

set -u

PI="ubuntu@192.168.0.103"
KEY="$HOME/.ssh/carolus_nopass"
SSH_OPTS=(-i "$KEY" -o IdentitiesOnly=yes -o StrictHostKeyChecking=no -o ConnectTimeout=5)
export SSH_AUTH_SOCK=''      # offer ONLY the key above: avoids the passphrase pop-up

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/carolus_ws/src"
REMOTE_WS="/home/ubuntu/carolus_ws"

APPLY=0; BUILD=0
for a in "$@"; do
    case "$a" in
        --apply) APPLY=1 ;;
        --build) BUILD=1 ;;
        *) echo "Unknown option: $a" >&2; exit 2 ;;
    esac
done

# Packages that run on the Pi. robomaster_cam is the SDK bridge (pure Python);
# the other three are needed to build AND to run Carolus.
PKGS=(robomaster_cam carolus_node libuvgs_astrobee ff_msgs)
# A change in these requires a rebuild; robomaster_cam does not
# (catkin_install_python is commented out, scripts are invoked directly).
NEEDS_BUILD=(carolus_node libuvgs_astrobee ff_msgs)

sshpi() { ssh "${SSH_OPTS[@]}" "$PI" "$@"; }

echo "== deploy_pi.sh =="
[ "$APPLY" -eq 1 ] && echo "  mode: APPLY" || echo "  mode: DRY-RUN (nothing will be modified)"

# --- 0. local syntax check before sending anything -------------------------
fail=0
while IFS= read -r f; do
    python3 -m py_compile "$f" 2>/dev/null || { echo "  !! does not compile: $f" >&2; fail=1; }
done < <(find "$SRC/robomaster_cam" -name '*.py' 2>/dev/null)
[ "$fail" -eq 1 ] && { echo "ABORT: local syntax error." >&2; exit 1; }
echo "  py_compile local OK"

# --- 1. reachability -------------------------------------------------------
sshpi "echo ok" >/dev/null 2>&1 || {
    echo "ABORT: Pi unreachable ($PI). Check RNDIS / power." >&2; exit 1; }
echo "  Pi reachable"

# --- 2. the Pi's git state, for information (see the header) ---------------
echo
echo "--- git state of the remote checkout ---"
sshpi "cd $REMOTE_WS 2>/dev/null && { echo -n '  HEAD: '; git rev-parse --short HEAD 2>/dev/null || echo 'not a git repository'; \
       n=\$(git status --porcelain 2>/dev/null | wc -l); echo \"  files modified locally on the Pi: \$n\"; }" 2>/dev/null

# --- 3. drift by checksum, package by package ------------------------------
echo
DRIFT=0; BUILD_NEEDED=0
for p in "${PKGS[@]}"; do
    [ -d "$SRC/$p" ] || { echo "  (absent locally, skipped: $p)"; continue; }
    echo "--- $p ---"
    OUT="$(rsync -rc --dry-run --itemize-changes --exclude '__pycache__' --exclude '*.pyc' \
             -e "ssh ${SSH_OPTS[*]}" "$SRC/$p/" "$PI:$REMOTE_WS/src/$p/" 2>/dev/null \
           | grep -E '^[><ch.]f' )"
    if [ -z "$OUT" ]; then
        echo "    in sync"
    else
        echo "$OUT" | sed 's/^/    /'
        DRIFT=1
        for b in "${NEEDS_BUILD[@]}"; do [ "$p" = "$b" ] && BUILD_NEEDED=1; done
        if [ "$APPLY" -eq 1 ]; then
            rsync -rc --exclude '__pycache__' --exclude '*.pyc' \
                  -e "ssh ${SSH_OPTS[*]}" "$SRC/$p/" "$PI:$REMOTE_WS/src/$p/" >/dev/null 2>&1 \
                && echo "    -> SYNCED" || { echo "    !! rsync FAILED" >&2; exit 1; }
        fi
    fi
done

if [ "$DRIFT" -eq 0 ]; then
    echo
    echo "RESULT: everything is already in sync with the Pi."
    exit 0
fi

if [ "$APPLY" -eq 0 ]; then
    echo
    echo "RESULT: DRIFT FOUND (nothing was modified). Re-run with --apply."
    [ "$BUILD_NEEDED" -eq 1 ] && echo "        A rebuild will be needed (--build): C++/launch files changed."
    exit 0
fi

# --- 4. re-verification by checksum AFTER the copy --------------------------
# An rsync exiting 0 does not prove both sides match; we read them back.
echo
echo "--- post-copy verification (checksum) ---"
BAD=0
for p in "${PKGS[@]}"; do
    [ -d "$SRC/$p" ] || continue
    LEFT="$(rsync -rc --dry-run --itemize-changes --exclude '__pycache__' --exclude '*.pyc' \
              -e "ssh ${SSH_OPTS[*]}" "$SRC/$p/" "$PI:$REMOTE_WS/src/$p/" 2>/dev/null \
            | grep -cE '^[><ch.]f' )"
    if [ "${LEFT:-0}" -ne 0 ]; then echo "  !! $p : $LEFT file(s) still different" >&2; BAD=1
    else echo "  $p : identical"; fi
done
[ "$BAD" -eq 1 ] && { echo "ABORT: the copy did not converge." >&2; exit 1; }

# --- 5. syntax check on the Pi, with the Pi's own interpreter --------------
sshpi "python3 -c \"import ast,glob,sys
bad=[f for f in glob.glob('$REMOTE_WS/src/robomaster_cam/scripts/*.py')
     if not ast.parse(open(f).read())=='' and False] if False else []
for f in glob.glob('$REMOTE_WS/src/robomaster_cam/scripts/*.py'):
    ast.parse(open(f).read())
print('  ast.parse on the Pi OK')\"" 2>/dev/null \
  || echo "  WARN: ast.parse on the Pi failed (check manually)" >&2

# --- 6. rebuild, only when asked -------------------------------------------
if [ "$BUILD_NEEDED" -eq 1 ]; then
    if [ "$BUILD" -eq 1 ]; then
        echo
        echo "--- catkin_make on the Pi (several minutes) ---"
        sshpi "source /opt/ros/noetic/setup.bash && cd $REMOTE_WS && \
               catkin_make --pkg ff_msgs >/tmp/build_pi.log 2>&1 && \
               catkin_make >>/tmp/build_pi.log 2>&1 && echo '  build OK'" \
          || { echo "  !! BUILD FAILED - see /tmp/build_pi.log on the Pi" >&2; exit 1; }
    else
        echo
        echo "  WARNING: C++/launch files changed -> REBUILD REQUIRED."
        echo "             Re-run with --build, otherwise the Pi runs the old binary."
    fi
fi

echo
echo "OK. In the launcher: restart the affected tabs (Kill then relaunch)."
echo "    T2 for rm_cam_beacon.py, T3 for Carolus."
