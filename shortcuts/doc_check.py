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

# Top-level directories whose contents the manual may legitimately name.
# A path starting with anything else (/opt/ros, ~/.ssh, /home/ubuntu on the
# Pi, a URL) is not ours to check for existence and is skipped.
REPO_DIRS = ("carolus_ws/", "shortcuts/", "overleaf/")

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


def manual_lines():
    with open(MANUAL, encoding="utf-8") as f:
        return f.read().split("\n")


def find(term, lines=None):
    """LaTeX-aware search. Returns [(lineno, text), ...]."""
    lines = lines if lines is not None else manual_lines()
    needle = delatex(term).lower()
    hits = []
    for i, raw in enumerate(lines, 1):
        if needle in delatex(raw).lower():
            hits.append((i, raw.strip()))
    return hits


def _self_test(lines):
    """Prove the search works before trusting any zero it reports.

    The canary is an identifier that is certainly in the manual AND is
    certainly escaped there, so it fails exactly when the escaping logic
    breaks -- which is the bug this script exists to prevent.

    Picking the canary is itself easy to get wrong, and did get it wrong on
    the first pass: `rm_cam_beacon.py` also appears VERBATIM inside
    \\lstlisting code blocks (which are not LaTeX-escaped at all), so even a
    completely disabled delatex() still found it there and the self-test
    reported OK while proving nothing. Confirmed by deliberately breaking
    delatex() into a no-op and re-running this test: it passed anyway. The
    canary below appears only inside \\texttt{...} prose, never in a raw
    listing -- checked by grepping the manual for both its escaped and
    literal forms and confirming exactly one of the two occurs.
    """
    raw_text = "\n".join(lines)
    canary = "cam_view_helper.py"
    escaped_present = "cam\\_view\\_helper.py" in raw_text
    found = bool(find(canary, lines))
    if not escaped_present:
        print("SELF-TEST INCONCLUSIVE: the canary is no longer escaped in the "
              "manual. Pick a different canary rather than trusting this run.",
              file=sys.stderr)
        return False
    if not found:
        print("SELF-TEST FAILED: the LaTeX-aware search cannot find "
              f"'{canary}' even though the manual contains its escaped form. "
              "Every 'not mentioned' result below would be meaningless. "
              "Fix delatex() before using this output.", file=sys.stderr)
        sys.exit(2)
    naive = canary in raw_text
    print(f"self-test OK  (canary '{canary}': LaTeX-aware finds it, "
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


def main():
    args = sys.argv[1:]
    if not os.path.exists(MANUAL):
        print(f"manual not found: {MANUAL}", file=sys.stderr)
        return 2
    lines = manual_lines()

    if args and args[0] == "--find":
        if len(args) < 2:
            print("usage: doc_check.py --find NAME", file=sys.stderr)
            return 2
        _self_test(lines)
        hits = find(args[1], lines)
        print(f"\n'{args[1]}' -- {len(hits)} reference(s) in the manual:")
        for ln, txt in hits:
            print(f"  {MANUAL}:{ln}: {txt[:140]}")
        if not hits:
            print("  (none -- and the self-test above says that is a real "
                  "absence, not a broken search)")
        return 0

    print("== doc_check.py -- technical manual vs the code it documents ==\n")
    _self_test(lines)

    # ---- 1. stale references ------------------------------------------------
    print("\n--- references the manual makes that no longer exist ---")
    stale = 0
    for tok, linenos in sorted(referenced_paths(lines).items()):
        if not exists_in_repo(tok):
            stale += 1
            where = ", ".join(f"line {n}" for n in linenos[:4])
            print(f"  STALE  {tok}   ({where})")
    if stale == 0:
        print("  none -- every path the manual names resolves on disk")

    # ---- 2. changed files the manual talks about ---------------------------
    if args and args[0] == "--all":
        targets, label = published_files(), "every published file"
    elif args:
        targets, label = args, "the files you named"
    else:
        targets, label = changed_files(), "your uncommitted changes"

    print(f"\n--- does the manual document {label}? ---")
    if not targets:
        print("  nothing to review")
    else:
        any_hit = False
        for f in targets:
            name = os.path.basename(f)
            hits = find(name, lines)
            if hits:
                any_hit = True
                where = ", ".join(str(n) for n, _ in hits[:6])
                print(f"  REVIEW  {f}\n            mentioned at line(s) {where}")
        if not any_hit:
            print("  none of them are mentioned in the manual "
                  "(self-test above confirms the search works)")

    print("\nReminder: 'mentioned' is not 'correct'. Open each line above and "
          "either update it or state that it still holds.")
    return 1 if stale else 0


if __name__ == "__main__":
    sys.exit(main())
