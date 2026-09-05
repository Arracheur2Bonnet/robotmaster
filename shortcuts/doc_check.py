#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check the technical manual against the code it documents.

WHY THIS EXISTS (2026-08-12). Asked whether `technical.tex` had been kept in
step with the day's code changes, the obvious check was run:

    grep -c cam_view_helper overleaf/technical.tex     ->  0

and it was WRONG. LaTeX escapes underscores, so the manual says
`cam\\_view\\_helper` and a search for `cam_view_helper` can never match. Four
components were documented in the manual while that grep reported none of
them. Acting on it would have left a genuinely false passage in place -- and
one was there: the manual claimed `cam_view_helper.py` runs over SSH on the
Pi when it runs locally on the lab PC.

The lesson is not "remember the backslashes". It is that a grep against a
.tex file is a check that CANNOT FAIL: it returns 0 whether the manual is
silent or whether the search is broken, and those two readings demand
opposite actions. This script exists so that question is never answered by
hand again.

WHAT IT DOES

  1. Searches the manual with LaTeX escaping normalised away, so an
     identifier is found whether it is written `foo_bar`, `foo\\_bar`, or
     wrapped in \\texttt{}.
  2. Flags STALE references: paths the manual names that do not exist on
     disk any more. This is the direction that silently rots -- a file gets
     renamed or archived and the manual keeps describing it.
  3. Given changed files (from git, or named on the command line), lists
     which of them the manual mentions and where, so the review set is
     produced mechanically instead of from memory.

  It SELF-TESTS before reporting anything (see `_self_test`): if its own
  LaTeX-aware search cannot find a known-escaped string in the manual, it
  aborts loudly instead of reporting a reassuring zero. A check that cannot
  come back negative is decoration -- that is exactly the defect this
  script was written in response to.

USAGE

    python3 shortcuts/doc_check.py                # audit + review uncommitted changes
    python3 shortcuts/doc_check.py --all          # audit + review every published file
    python3 shortcuts/doc_check.py FILE [FILE...] # review specific files
    python3 shortcuts/doc_check.py --find NAME    # LaTeX-aware search for one identifier

Exit code is 1 if stale references were found, else 0.
"""

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
MANUAL = os.path.join(ROOT, "overleaf", "technical.tex")

# The ROS2 manual, added 2026-09-05. Until then this script read technical.tex
# ONLY, so a change touching just the ROS2 tree produced a clean run that meant
# nothing -- the same reassuring zero this script exists to eliminate,
# reintroduced by scope rather than by escaping.
MANUAL_ROS2 = os.path.join(ROOT, "raspberry5-carolus-ros2", "technical-ros2.tex")
MANUALS = [MANUAL, MANUAL_ROS2]

# Top-level directories whose contents the manual may legitimately name.
# A path starting with anything else (/opt/ros, ~/.ssh, /home/ubuntu on the
# Pi, a URL) is not ours to check for existence and is skipped.
REPO_DIRS = ("carolus_ws/", "shortcuts/", "overleaf/",
             "raspberry5-carolus-ros2/")

FILE_EXT = (".py", ".sh", ".launch", ".yaml", ".yml", ".txt", ".xml",
            ".cpp", ".hpp", ".h", ".tex", ".pdf", ".md", ".json")


def delatex(line):
    """Strip the LaTeX escaping that makes a naive grep silently fail.

    Only the escapes that appear in identifiers are undone; the goal is a
    string an ordinary search can match, not valid plain text.
    """
    out = line
    out = re.sub(r"\\texttt\{", "{", out)
    out = re.sub(r"\\(_|%|&|\#|\$|\{|\})", r"\1", out)
    out = out.replace("\\textbackslash", "\\")
    return out


def manual_lines(path=MANUAL):
    with open(path, encoding="utf-8") as f:
        return f.read().split("\n")


# One canary per manual: an identifier present in escaped form (proves
# delatex() runs) and absent in literal form (proves a raw \lstlisting
# match isn't silently doing the work instead -- the exact way the first
# version of this self-test passed while proving nothing, see _self_test).
CANARIES = {
    MANUAL: "cam_view_helper.py",
    MANUAL_ROS2: "carolus_node_ros2.cpp",
}


def find(term, lines=None):
    """LaTeX-aware search. Returns [(lineno, text), ...]."""
    lines = lines if lines is not None else manual_lines()
    needle = delatex(term).lower()
    hits = []
    for i, raw in enumerate(lines, 1):
        if needle in delatex(raw).lower():
            hits.append((i, raw.strip()))
    return hits


def _self_test(lines, path=MANUAL):
    """Prove the search works, for THIS manual, before trusting any zero it
    reports for it.

    The canary is an identifier that is certainly in the manual AND is
    certainly escaped there, so it fails exactly when the escaping logic
    breaks -- which is the bug this script exists to prevent.

    Picking the canary is itself easy to get wrong, and did get it wrong on
    the first pass: `rm_cam_beacon.py` also appears VERBATIM inside
    \\lstlisting code blocks (which are not LaTeX-escaped at all), so even a
    completely disabled delatex() still found it there and the self-test
    reported OK while proving nothing. Confirmed by deliberately breaking
    delatex() into a no-op and re-running this test: it passed anyway. Each
    canary in CANARIES appears only inside \\texttt{...} prose, never in a
    raw listing -- checked by grepping its manual for both the escaped and
    literal forms and confirming exactly one of the two occurs.

    2026-09-05: this ran against MANUAL only for a full day after the ROS2
    manual was added to every OTHER check in this script -- `--find` and the
    stale-reference sweep both kept silently searching `overleaf/technical.tex`
    alone and reporting a confirmed-real absence for terms that were only ever
    going to be in `technical-ros2.tex`. One canary per manual now, so a gap
    like that fails loudly instead of passing on the wrong file.
    """
    raw_text = "\n".join(lines)
    canary = CANARIES.get(path, CANARIES[MANUAL])
    rel = os.path.relpath(path, ROOT)
    escaped_present = canary.replace("_", "\\_") in raw_text
    found = bool(find(canary, lines))
    if not escaped_present:
        print(f"SELF-TEST INCONCLUSIVE ({rel}): the canary '{canary}' is no "
              "longer escaped in this manual. Pick a different canary rather "
              "than trusting this run.", file=sys.stderr)
        return False
    if not found:
        print(f"SELF-TEST FAILED ({rel}): the LaTeX-aware search cannot find "
              f"'{canary}' even though the manual contains its escaped form. "
              "Every 'not mentioned' result for this file would be "
              "meaningless. Fix delatex() before using this output.",
              file=sys.stderr)
        sys.exit(2)
    naive = canary in raw_text
    print(f"self-test OK  ({rel}, canary '{canary}': LaTeX-aware finds it, "
          f"naive grep {'also would' if naive else 'would NOT'} -- "
          f"{'no' if naive else 'this is the trap this script removes'})")
    return True


def referenced_paths(lines):
    """Repo-relative paths the manual names -- fully qualified only.

    A bare filename (`patch.sh`, `target.yaml`, `dji.json`, ...) was tried
    first and dropped after testing against the real manual: it flagged
    adb_en.sh, dji.json, patch.sh, subscribe.launch and target.yaml as
    "stale" when every one of them is a real, correct reference to a file
    that lives in the S1 rooting toolkit or Kalibr -- external tools this
    manual legitimately describes without vendoring. A generic filename
    cannot tell "our file was deleted" from "their file we never had", so
    it is not attempted. A path that starts with one of REPO_DIRS is
    unambiguous -- it can only mean this repository -- and that is the
    only shape checked here.
    """
    found = {}
    for i, raw in enumerate(lines, 1):
        flat = delatex(raw)
        for m in re.finditer(r"[A-Za-z0-9_./*-]+", flat):
            tok = m.group(0).strip(".,;:)")
            if not (tok.startswith(REPO_DIRS) and not tok.endswith("/")):
                continue
            # A naming TEMPLATE, not a literal path: YYYY-MM-DD-style
            # placeholders or a glob (session-*.log) describe a pattern the
            # code generates at runtime, not a file that should exist right
            # now. Real example: shortcuts/logs/session-YYYY-MM-DD-HH-MM-SS.log.
            if re.search(r"YYYY|MM-DD|\*", tok):
                continue
            found.setdefault(tok, []).append(i)
    return found


def exists_in_repo(token):
    """True if the manual's fully-qualified reference resolves on disk."""
    return os.path.exists(os.path.join(ROOT, token))


def changed_files():
    try:
        out = subprocess.run(
            ["git", "-C", ROOT, "status", "--porcelain"],
            capture_output=True, text=True, check=True).stdout
    except Exception:
        return []
    files = []
    for line in out.split("\n"):
        if len(line) > 3:
            files.append(line[3:].strip().strip('"'))
    return files


def published_files():
    try:
        out = subprocess.run(["git", "-C", ROOT, "ls-files"],
                             capture_output=True, text=True, check=True).stdout
        return [f for f in out.split("\n") if f.strip()]
    except Exception:
        return []


def bug_reference_check():
    """Both manuals forbid BUG-XXX labels: a reader with only the repository
    cannot resolve them.

    This has broken twice -- seven references removed 2026-08-25, thirteen more
    found 2026-09-05 -- and both times while writing up a real bug, which is
    exactly when reaching for its number is most automatic. A rule that depends
    on remembering it at the one moment it is hardest to remember needs a
    mechanical check, not better intentions.

    Returns the number of offending references (0 is the only acceptable value).
    """
    total = 0
    print("\n--- bug numbers leaked into the manuals (must be zero) ---")
    for man in MANUALS:
        if not os.path.exists(man):
            continue
        hits = []
        with open(man, encoding="utf-8") as f:
            for i, raw in enumerate(f.read().split("\n"), 1):
                for m in re.finditer(r"BUG-\d{3}", raw):
                    hits.append((i, m.group(0), raw.strip()[:110]))
        total += len(hits)
        rel = os.path.relpath(man, ROOT)
        if hits:
            print(f"  FAIL  {rel}: {len(hits)} reference(s)")
            for ln, tag, txt in hits:
                print(f"          line {ln}: {tag} -- {txt}")
        else:
            print(f"  ok    {rel}")
    if total:
        print("  Keep the content, drop the label: say 'corrected 2026-09-04'")
        print("  or 'known and deliberately unfixed' instead of the number.")
    return total


def existing_manuals():
    found = [m for m in MANUALS if os.path.exists(m)]
    if not found:
        print(f"no manual found (looked for {MANUALS})", file=sys.stderr)
        sys.exit(2)
    return found


def main():
    args = sys.argv[1:]
    manuals = existing_manuals()
    manual_lines_by_path = {m: manual_lines(m) for m in manuals}

    if args and args[0] == "--find":
        if len(args) < 2:
            print("usage: doc_check.py --find NAME", file=sys.stderr)
            return 2
        for m in manuals:
            _self_test(manual_lines_by_path[m], m)
        print(f"\n'{args[1]}' -- reference(s) across {len(manuals)} manual(s):")
        any_hit = False
        for m in manuals:
            hits = find(args[1], manual_lines_by_path[m])
            for ln, txt in hits:
                any_hit = True
                print(f"  {m}:{ln}: {txt[:140]}")
        if not any_hit:
            print("  (none in any manual -- and the self-tests above say "
                  "that is a real absence, not a broken search)")
        return 0

    print("== doc_check.py -- technical manuals vs the code they document ==\n")
    for m in manuals:
        _self_test(manual_lines_by_path[m], m)

    # ---- 1. stale references, per manual ------------------------------------
    print("\n--- references the manuals make that no longer exist ---")
    stale = 0
    for m in manuals:
        rel = os.path.relpath(m, ROOT)
        for tok, linenos in sorted(referenced_paths(manual_lines_by_path[m]).items()):
            if not exists_in_repo(tok):
                stale += 1
                where = ", ".join(f"line {n}" for n in linenos[:4])
                print(f"  STALE  {rel}: {tok}   ({where})")
    if stale == 0:
        print("  none -- every path either manual names resolves on disk")

    # ---- 2. changed files the manuals talk about ----------------------------
    if args and args[0] == "--all":
        targets, label = published_files(), "every published file"
    elif args:
        targets, label = args, "the files you named"
    else:
        targets, label = changed_files(), "your uncommitted changes"

    print(f"\n--- do the manuals document {label}? ---")
    if not targets:
        print("  nothing to review")
    else:
        any_hit = False
        for f in targets:
            name = os.path.basename(f)
            for m in manuals:
                hits = find(name, manual_lines_by_path[m])
                if hits:
                    any_hit = True
                    where = ", ".join(str(n) for n, _ in hits[:6])
                    rel = os.path.relpath(m, ROOT)
                    print(f"  REVIEW  {f}\n            mentioned in {rel} at line(s) {where}")
        if not any_hit:
            print("  none of them are mentioned in either manual "
                  "(the self-tests above confirm the search works)")

    leaked = bug_reference_check()

    print("\nReminder: 'mentioned' is not 'correct'. Open each line above and "
          "either update it or state that it still holds.")
    return 1 if (stale or leaked) else 0


if __name__ == "__main__":
    sys.exit(main())
