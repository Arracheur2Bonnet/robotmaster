#!/usr/bin/env bash
# measure_pi_pose.sh — measure Carolus's real pose rate on the Raspberry Pi,
# with a beacon in view. Run from the lab PC; everything happens on the Pi.
#
# Usage:
#   bash shortcuts/measure_pi_pose.sh            # full run (~3 min)
#   bash shortcuts/measure_pi_pose.sh --stop     # stop everything, leave Pi clean
#
# ============================================================================
# WHY THIS EXISTS
# ============================================================================
# The 2026-08-04 Pi benchmark measured ~24 frames/s PROCESSED, counted from the
# node's per-frame log with NO beacon in view -- so no P4P solve ran per frame,
# and the outlier filter (filter_size / fifo_on) was never exercised. Both
# caveats are recorded in the technical manual.
#
# What is still unmeasured is the number the supervisor will actually quote:
# the rate of /pose itself, under load, with four LEDs visible. This script
# collects that in one command so a short hardware slot is spent on the robot
# rather than on typing.
#
# It also captures two things worth having while the stack is up:
#   - whether the BUG-087 non-convergence warning ever fires in a real run
#     (surfaced 2026-08-03, never yet observed live -- the actual rate of
#     non-convergence is unknown, and no rejection policy can be designed
#     until it is)
#   - whether [LOCK] ticks during the run (point 4 of 21-points-a-creuser)
#
# PREREQUISITES, none of which this script can check for you:
#   - robot powered on (double chime), Pi reachable
#   - a BEACON POWERED ON AND IN THE CAMERA'S FIELD OF VIEW  <-- the whole point
# ============================================================================

set -u

PI_HOST="${CAROLUS_PI_HOST:-192.168.0.103}"
PI="ubuntu@${PI_HOST}"
SSH_OPTS=(-o ConnectTimeout=10 -o StrictHostKeyChecking=no)
DURATION="${DURATION:-60}"       # seconds of /pose measurement
OUT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/shortcuts/logs"
STAMP="$(date +%Y-%m-%d-%H-%M-%S)"
REPORT="${OUT_DIR}/pi-pose-measure-${STAMP}.log"

remote() { timeout 300 ssh "${SSH_OPTS[@]}" "$PI" "$@" 2>&1 | grep -v "sign_and_send_pubkey"; }

stop_all() {
    echo "Stopping Carolus / camera / roscore on the Pi..."
    remote 'pkill -f roslaunch; sleep 2; pkill -f carolus_astrobee; sleep 1;
            pkill -f rm_cam_beacon; sleep 2; pkill -f roscore; sleep 1;
            pgrep -af "roscore|roslaunch|carolus_astrobee|rm_cam_beacon" | grep -v pgrep \
              || echo "  Pi clean"'
}

if [ "${1:-}" = "--stop" ]; then stop_all; exit 0; fi

mkdir -p "$OUT_DIR"
echo "== measure_pi_pose.sh ==" | tee "$REPORT"
echo "Pi: $PI_HOST   duration: ${DURATION}s   report: $REPORT" | tee -a "$REPORT"
echo | tee -a "$REPORT"
echo ">>> CHECK BEFORE CONTINUING: is a powered beacon in the camera's view?" | tee -a "$REPORT"
echo "    (without it /pose never publishes and this run measures nothing)" | tee -a "$REPORT"
echo | tee -a "$REPORT"

# --- reachability ----------------------------------------------------------
if ! timeout 15 ping -c 2 "$PI_HOST" >/dev/null 2>&1; then
    echo "!! Pi unreachable at $PI_HOST — is it powered on?" | tee -a "$REPORT"; exit 1
fi
echo "[1/6] Pi reachable." | tee -a "$REPORT"

# --- RNDIS link to the robot ----------------------------------------------
remote 'sudo -n ip link set eth1 up 2>/dev/null
        sudo -n ip addr add 192.168.42.3/24 dev eth1 2>/dev/null | grep -v "File exists"
        sleep 2
        timeout 8 ping -c 2 192.168.42.2 >/dev/null 2>&1 \
          && echo "  robot reachable over RNDIS" || echo "  !! ROBOT NOT REACHABLE"' \
    | tee -a "$REPORT"
echo "[2/6] RNDIS link checked." | tee -a "$REPORT"

# --- start the stack on the Pi --------------------------------------------
# The SDK needs Python 3.7 (venv) AND ROS's python deps (system) -- neither
# interpreter alone has both, so the venv must be activated with ROS sourced.
remote 'source /opt/ros/noetic/setup.bash
        export ROS_MASTER_URI=http://'"$PI_HOST"':11311 ROS_IP='"$PI_HOST"'
        pgrep -f roscore >/dev/null || { nohup roscore >/tmp/roscore.log 2>&1 & sleep 6; }
        source ~/Python-3.7.17/env/bin/activate
        cd ~/carolus_ws
        pgrep -f rm_cam_beacon >/dev/null || \
          { nohup python src/robomaster_cam/scripts/rm_cam_beacon.py >/tmp/cam.log 2>&1 & }
        sleep 22
        echo "  nodes:"; rosnode list 2>&1 | sed "s/^/    /"' | tee -a "$REPORT"
echo "[3/6] Camera bridge up." | tee -a "$REPORT"

remote 'source /opt/ros/noetic/setup.bash
        export ROS_MASTER_URI=http://'"$PI_HOST"':11311 ROS_IP='"$PI_HOST"'
        cd ~/carolus_ws && source devel/setup.bash
        pgrep -f carolus_astrobee >/dev/null || \
          { nohup roslaunch carolus_node testcarolus.launch ubuntu2204_preload:=false \
              >/tmp/carolus.log 2>&1 & }
        sleep 20; echo "  carolus started"' | tee -a "$REPORT"
echo "[4/6] Carolus up." | tee -a "$REPORT"

# --- THE measurement -------------------------------------------------------
echo | tee -a "$REPORT"
echo "=== /pose rate over ${DURATION}s (THE number) ===" | tee -a "$REPORT"
remote 'source /opt/ros/noetic/setup.bash
        export ROS_MASTER_URI=http://'"$PI_HOST"':11311 ROS_IP='"$PI_HOST"'
        timeout '"$((DURATION+5))"' rostopic hz /pose 2>&1 | tail -6' | tee -a "$REPORT"

echo | tee -a "$REPORT"
echo "=== /postprocessed/image rate (frames actually processed) ===" | tee -a "$REPORT"
remote 'source /opt/ros/noetic/setup.bash
        export ROS_MASTER_URI=http://'"$PI_HOST"':11311 ROS_IP='"$PI_HOST"'
        timeout 20 rostopic hz /postprocessed/image 2>&1 | tail -4' | tee -a "$REPORT"

echo | tee -a "$REPORT"
echo "=== one /pose sample (is the geometry plausible?) ===" | tee -a "$REPORT"
remote 'source /opt/ros/noetic/setup.bash
        export ROS_MASTER_URI=http://'"$PI_HOST"':11311 ROS_IP='"$PI_HOST"'
        timeout 15 rostopic echo -n1 /pose 2>&1 | head -20' | tee -a "$REPORT"
echo "[5/6] Rates captured." | tee -a "$REPORT"

# --- opportunistic findings while the stack is up --------------------------
echo | tee -a "$REPORT"
echo "=== BUG-087: did the solver ever fail to converge? ===" | tee -a "$REPORT"
remote 'grep -c "did NOT converge" /tmp/carolus.log 2>/dev/null || echo 0' | tee -a "$REPORT"
echo "  (0 = never fired in this run; >0 = the rate is now known, see /tmp/carolus.log)" | tee -a "$REPORT"

echo | tee -a "$REPORT"
echo "=== blob detection health (last lines) ===" | tee -a "$REPORT"
remote 'tail -6 /tmp/carolus.log' | tee -a "$REPORT"

echo | tee -a "$REPORT"
echo "=== CPU / load on the Pi under this load ===" | tee -a "$REPORT"
remote 'uptime; top -bn1 | grep "%Cpu" | head -1' | tee -a "$REPORT"

echo | tee -a "$REPORT"
echo "=== point 4: did LOCK tick during the run? ===" | tee -a "$REPORT"
remote 'grep -c "\[LOCK\]" /tmp/cam.log 2>/dev/null || echo 0' | tee -a "$REPORT"

echo | tee -a "$REPORT"
echo "[6/6] Done. Full report: $REPORT" | tee -a "$REPORT"
echo "Stop the stack with: bash shortcuts/measure_pi_pose.sh --stop" | tee -a "$REPORT"
