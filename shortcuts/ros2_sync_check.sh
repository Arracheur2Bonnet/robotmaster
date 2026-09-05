#!/usr/bin/env bash
# ros2_sync_check.sh -- has the ROS2 folder's copy of the Carolus core drifted
# from the ROS1 original?
#
# raspberry5-carolus-ros2/ holds its OWN COPY of the detection/solver core
# rather than referencing carolus_ws/ by path, so the folder stays portable
# (deliberate, 2026-08-20 -- see CLAUDE.md's "ROS2 manual" section). The cost
# of that choice is that a fix made on one side does not reach the other, and
# nothing warns you. This script is that warning.
#
# Run it after ANY change to carolus_ws/src/libuvgs_astrobee/{src,include}/.
# Exit 0 = in sync, 1 = drifted (or a file is missing on either side).
#
# There are exactly TWO maintained trees: carolus_ws/ (ROS1) and
# raspberry5-carolus-ros2/ (ROS2). A third copy for testing/comparison
# (e.g. a "test_ros/") was considered and deliberately rejected 2026-09-04 --
# this script exists precisely so two copies stay safe without a third.
# The ROS1-vs-ROS2 comparison itself lives in technical-ros2.tex's
# "Where this node genuinely differs from the ROS1 one" section, not in code.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS1_SRC="$ROOT/carolus_ws/src/libuvgs_astrobee/src"
ROS1_INC="$ROOT/carolus_ws/src/libuvgs_astrobee/include/carolus_node"
ROS2_SRC="$ROOT/raspberry5-carolus-ros2/carolus_ros2/src"
ROS2_INC="$ROOT/raspberry5-carolus-ros2/carolus_ros2/include/carolus_node"

# The shared core ONLY. carolus_astrobee.cpp (ROS1 node) and
# carolus_node_ros2.cpp (ROS2 wrapper) are each middleware-specific by design
# and must NOT be compared -- they are supposed to differ.
SRC_FILES="beacon_detector.cpp ceresP4P.cpp pose_est.cpp pose_filter.cpp"
INC_FILES="beacon_detector.hpp ceresP4P.hpp pose_est.hpp pose_filter.hpp carolus_types.hpp compute_jacobian.h"

echo "== ros2_sync_check.sh -- ROS1 core vs its copy in raspberry5-carolus-ros2/ =="
echo ""

problems=0
checked=0

check_pair() {
    local a="$1" b="$2" name="$3"
    if [ ! -f "$a" ]; then
        echo "  MISSING (ROS1)  $name  -- expected at $a"
        problems=$((problems+1)); return
    fi
    if [ ! -f "$b" ]; then
        echo "  MISSING (ROS2)  $name  -- expected at $b"
        problems=$((problems+1)); return
    fi
    checked=$((checked+1))
    if diff -q "$a" "$b" > /dev/null 2>&1; then
        echo "  in sync         $name"
    else
        echo "  DRIFTED         $name"
        echo "                  diff $a \\"
        echo "                       $b"
        problems=$((problems+1))
    fi
}

for f in $SRC_FILES; do check_pair "$ROS1_SRC/$f" "$ROS2_SRC/$f" "$f"; done
for f in $INC_FILES; do check_pair "$ROS1_INC/$f" "$ROS2_INC/$f" "$f"; done

echo ""
if [ "$problems" -eq 0 ]; then
    echo "RESULT: in sync. $checked files compared, all byte-identical."
    exit 0
else
    echo "RESULT: $problems problem(s) across $checked comparable file(s)."
    echo "Decide DELIBERATELY which side is correct -- do not sync by reflex."
    echo "The ROS1 node is usually the original, but not always: BUG-125/126"
    echo "(2026-08-20) were found and fixed on the ROS2 side first."
    exit 1
fi
