#!/usr/bin/env bash
# sync_repo.sh — reconcile carolus_repo/ (the published git repo) with the
# live working files.
#
# Usage:
#   bash shortcuts/sync_repo.sh          # DRY RUN: report drift, change nothing
#   bash shortcuts/sync_repo.sh --apply  # copy live -> repo, then report
#
# ============================================================================
# WHY THIS EXISTS (2026-08-04)
# ============================================================================
# carolus_repo/ is not a checkout of the working tree -- it is a hand-assembled
# subset pulled from THREE separate places:
#
#     carolus_ws/src/          -> carolus_repo/src/          (minus robot_localization)
#     carolus_ws/cmake_shims/  -> carolus_repo/cmake_shims/
#     shortcuts/               -> carolus_repo/shortcuts/    (PROJECT ROOT, not carolus_ws!)
#
# plus two directories that exist ONLY in the repo and must never be
# overwritten from anywhere: overleaf/ and README.md.
#
# Nothing reconciled these automatically, so the repo drifted silently. Real
# instances found on 2026-08-03/04, each one a file the published manual
# describes but the published repo did not contain:
#   - rm_cam_beacon.py missing the /imu publisher
#   - three files missing the Ceres solver-diagnostic fix
#   - leak_scan.sh still the OLD version, blind to PDF/DOCX -- a reader
#     would have run it, got "nothing found", and believed it
#
# DRY RUN IS THE DEFAULT ON PURPOSE. A blind copy could clobber a repo-only
# edit. Always read the report before passing --apply.
# ============================================================================

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

REPO="carolus_repo"
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

# Generated or machine-local: never belongs in the published repo.
EXCL=(--exclude='__pycache__' --exclude='*.pyc' --exclude='*.swp' --exclude='*.swo'
      --exclude='.git' --exclude='logs' --exclude='build' --exclude='devel')

# source_path : repo_destination
PAIRS=(
    "carolus_ws/src/:$REPO/src/"
    "carolus_ws/cmake_shims/:$REPO/cmake_shims/"
    "shortcuts/:$REPO/shortcuts/"
)

# Deliberately NOT published:
#   robot_localization  36 MB unmodified upstream clone; the README tells the
#                       reader to clone it themselves instead.
#   /CMakeLists.txt     in carolus_ws this is an absolute symlink into
#                       /opt/ros/noetic/. The repo holds a real copy of
#                       toplevel.cmake instead, which is what a reader with a
#                       different ROS path needs -- syncing would replace a
#                       portable file with a link broken on their machine.
EXCL_SRC=(--exclude='robot_localization' --exclude='/CMakeLists.txt')

echo "== sync_repo.sh =="
if [ "$APPLY" -eq 1 ]; then
    echo "Mode: APPLY (files will be copied live -> repo)"
else
    echo "Mode: DRY RUN (nothing will be changed; pass --apply to sync)"
fi
echo

DRIFT=0
for pair in "${PAIRS[@]}"; do
    src="${pair%%:*}"
    dst="${pair##*:}"
    [ -d "$src" ] || { echo "!! missing source: $src"; continue; }

    extra=()
    [ "$src" = "carolus_ws/src/" ] && extra=("${EXCL_SRC[@]}")

    echo "--- $src -> $dst ---"
    # -c compares by CHECKSUM, not timestamp. Without it rsync reports every
    # file whose mtime differs even when the content is byte-identical, which
    # buries the handful of real differences in hundreds of lines of noise.
    # -i itemize, -n dry run: show what WOULD change, change nothing.
    out="$(rsync -rcin --delete --links "${EXCL[@]}" "${extra[@]}" "$src" "$dst" 2>/dev/null \
           | grep -vE '^\.[dfL]|^\.\.\.\.\.' )"   # drop "no change" entries

    if [ -z "$out" ]; then
        echo "    in sync"
    else
        echo "$out" | sed 's/^/    /'
        DRIFT=1
        if [ "$APPLY" -eq 1 ]; then
            rsync -ac --delete --links "${EXCL[@]}" "${extra[@]}" "$src" "$dst"
            echo "    -> SYNCED"
        fi
    fi
    echo
done

echo "Note: overleaf/ and README.md live only in the repo and are never"
echo "touched by this script -- edit them there directly."
echo

if [ "$DRIFT" -eq 0 ]; then
    echo "RESULT: repo matches the working files."
elif [ "$APPLY" -eq 1 ]; then
    echo "RESULT: drift found and synced. Review with 'git -C $REPO status'"
    echo "        and 'git -C $REPO diff' before committing."
else
    echo "RESULT: DRIFT FOUND (nothing changed). Re-run with --apply to sync."
    echo "        rsync codes: '>f' = file differs/new, '*deleting' = removed."
fi

echo
echo "Reminder: run 'bash shortcuts/leak_scan.sh $REPO' before any push,"
echo "and scan 'git -C $REPO log -p' before making the repository public --"
echo "a clean working tree says nothing about what is still in the history."
