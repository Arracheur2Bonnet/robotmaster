#!/usr/bin/env python3
"""Check that every catkin package can actually be built on a MACHINE THAT IS NOT THIS ONE.

WHY THIS EXISTS (2026-08-13). Our supervisor followed the technical guide on his
own Raspberry Pi and the build failed twice, on two different missing
dependencies, on two consecutive days:

    catkin_make --pkg ff_msgs
      -> Could not find image_transport (missing: image_transport_DIR)
      -> then, after that fix:
      -> Could not find a package configuration file provided by "Ceres"

Neither failure was reproducible here, and no amount of re-reading the guide
would ever have found them. The reason is structural and worth stating plainly:

    OUR machines have had every dependency installed for months, as a side
    effect of unrelated work. A missing declaration is therefore INVISIBLE to
    us -- the build succeeds locally for a reason that has nothing to do with
    the build being correctly specified.

The specific defect underneath both reports is the same one:

    CMakeLists.txt says   find_package(Ceres REQUIRED)
    package.xml says      (nothing)

`rosdep install`, which is what the guide tells the reader to run, reads ONLY
package.xml. Anything that exists solely in a find_package() call is invisible
to it, so it silently installs nothing and the build dies at CMake configure
time on any machine that did not already happen to have the library.

This script compares the two, per package, and fails loudly on drift.

It also validates that each package.xml is well-formed XML, because that check
was itself missing and this script's own first fix introduced a broken manifest
(a literal "--" inside an XML comment, which is illegal) that rosdep rejected
outright. A guardrail that can be silently bypassed by malformed input is not a
guardrail.

WHAT THIS DOES NOT DO. It cannot tell you a declared key is installable, only
that it is declared. Use --resolve to additionally check every key against a
target OS via rosdep -- that is what catches `ceres` (not a rosdep key) versus
`libceres-dev` (the real one), a distinction that cost a full round-trip.

USAGE
    python3 shortcuts/dep_check.py                # audit, target ubuntu:focal
    python3 shortcuts/dep_check.py --resolve      # also resolve every key via rosdep
    python3 shortcuts/dep_check.py --os ubuntu:jammy --resolve

Exit code 0 = clean, 1 = drift or unresolvable key found. Suitable for CI.
"""

import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
WS_SRC = os.path.normpath(os.path.join(HERE, "..", "carolus_ws", "src"))

# CMake package names that are provided by catkin itself or by the toolchain,
# and so never need their own package.xml entry.
CMAKE_IGNORE = {"catkin", "PkgConfig", "ament_cmake", "GTest", "Threads", "PythonLibs", "PythonInterp"}

# Vendored third-party packages we do not author and do not police. Listed
# explicitly rather than pattern-matched, so adding one is a deliberate act.
SKIP_PACKAGES = {"robot_localization"}

# CMake find_package() name -> the rosdep key that provides it. These differ
# often enough to be their own failure mode: `Ceres` is the CMake name,
# `libceres-dev` is the rosdep key, and `ceres` is neither.
CMAKE_TO_ROSDEP = {
    "Ceres": "libceres-dev",
    "Eigen3": "eigen",
    "Eigen": "eigen",
    "OpenCV": "libopencv-dev",
    "Boost": "boost",
    "yaml": "libyaml-cpp-dev",
    "yaml-cpp": "libyaml-cpp-dev",
}


def strip_cmake_comments(text):
    """Remove # comments. A commented-out find_package() is not a dependency --
    robomaster_cam has exactly one, and counting it produced a false positive
    the first time this audit was run by hand."""
    return re.sub(r"#.*", "", text)


def find_packages(src):
    out = []
    if not os.path.isdir(src):
        sys.exit("dep_check: workspace source dir not found: %s" % src)
    for name in sorted(os.listdir(src)):
        d = os.path.join(src, name)
        if os.path.isfile(os.path.join(d, "package.xml")):
            out.append((name, d))
    return out


def declared_keys(pkg_xml):
    """rosdep reads these tags and only these."""
    tags = ("build_depend", "exec_depend", "run_depend", "depend",
            "buildtool_depend", "build_export_depend", "test_depend")
    root = ET.parse(pkg_xml).getroot()
    keys = set()
    for t in tags:
        for el in root.findall(t):
            if el.text:
                keys.add(el.text.strip())
    return keys


def required_cmake(cml):
    with open(cml) as f:
        text = strip_cmake_comments(f.read())
    return {m for m in re.findall(r"find_package\(\s*([A-Za-z0-9_]+)", text)
            if m not in CMAKE_IGNORE}


def rosdep_resolve(key, target_os):
    try:
        r = subprocess.run(["rosdep", "resolve", key, "--os=" + target_os],
                           capture_output=True, text=True, timeout=30)
        return r.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--os", default="ubuntu:focal",
                    help="target OS for rosdep resolution (default: the Pi's ubuntu:focal)")
    ap.add_argument("--resolve", action="store_true",
                    help="also check every declared key actually resolves via rosdep")
    args = ap.parse_args()

    print("== dep_check.py -- can this workspace build on a machine that is not this one? ==\n")
    problems = 0
    all_keys = set()

    for name, d in find_packages(WS_SRC):
        pkg_xml = os.path.join(d, "package.xml")

        # 1. Manifest must parse. rosdep refuses the whole workspace otherwise.
        try:
            declared = declared_keys(pkg_xml)
        except ET.ParseError as e:
            print("  MALFORMED XML  %s/package.xml -- %s" % (name, e))
            print("                 (note: a literal '--' inside an XML comment is illegal)")
            problems += 1
            continue

        if name in SKIP_PACKAGES:
            print("  skipped        %s (vendored third-party, not ours to police)" % name)
            continue

        all_keys |= declared

        # 2. Every active find_package() must have a matching declaration.
        cml = os.path.join(d, "CMakeLists.txt")
        if not os.path.isfile(cml):
            continue
        missing = []
        for cm in sorted(required_cmake(cml)):
            expected = CMAKE_TO_ROSDEP.get(cm, cm)
            if expected not in declared and cm not in declared and cm.lower() not in declared:
                missing.append((cm, expected))
        if missing:
            problems += 1
            print("  DRIFT          %s" % name)
            for cm, expected in missing:
                print("                 CMakeLists.txt calls find_package(%s) but package.xml"
                      " declares no '%s'" % (cm, expected))
                print("                 -> rosdep will NOT install it; the build dies at configure"
                      " on a clean machine")
        else:
            print("  OK             %s" % name)

    # 3. Every declared key must actually be installable on the target OS.
    if args.resolve:
        print("\n--- resolving every declared key against %s ---" % args.os)
        probe = rosdep_resolve("roscpp", args.os)
        if probe is None:
            print("  SKIPPED: rosdep not available on this machine")
        elif not probe:
            print("  CANNOT CHECK: even 'roscpp' does not resolve.")
            print("  This is the end-of-life trap: plain `rosdep update` now SKIPS Noetic.")
            print("  Fix with:  rosdep update --include-eol-distros")
            problems += 1
        else:
            bad = []
            for k in sorted(all_keys):
                if k == "catkin":
                    continue
                if os.path.isdir(os.path.join(WS_SRC, k)):
                    continue  # a sibling package in this workspace, not a system dep
                if not rosdep_resolve(k, args.os):
                    bad.append(k)
            if bad:
                problems += 1
                for k in bad:
                    print("  UNRESOLVABLE   '%s' is declared but is not a rosdep key on %s" % (k, args.os))
                print("                 (CMake names and rosdep keys differ: Ceres -> libceres-dev)")
            else:
                print("  OK             all %d keys resolve" % len([k for k in all_keys if k != "catkin"]))

    print()
    if problems:
        print("RESULT: %d problem(s). A clean machine would fail to build this workspace." % problems)
        return 1
    print("RESULT: clean. Declarations match find_package(), and every key resolves.")
    print("NOTE: this proves the SPEC is consistent. It does not replace actually")
    print("      building on a blank machine -- see shortcuts/cleanroom_build.sh.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
