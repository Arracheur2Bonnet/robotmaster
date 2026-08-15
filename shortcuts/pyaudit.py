#!/usr/bin/env python3
"""AST audit of the project's Python files.

Written because pyflakes/ruff/vulture are NOT installed here (the first check
for them was itself broken -- `python3 -m x --version | head -1 && echo OK`
prints OK unconditionally, since head succeeds on empty input). Rather than
trust another tool that might not be there, this walks the AST directly.

Reports, per file:
  IMPORT    an imported name never referenced anywhere else in the file
  DEADFN    a module-level def/class never referenced in its own file
            (informational only -- ROS callbacks and entry points are
            referenced from outside, so this needs human judgement)
  SILENT    `except: pass` / `except Exception: pass` -- the exact pattern
            behind BUG-103 and BUG-104 on this project (a failure that
            reports success)
  BARE      a bare `except:` (also catches KeyboardInterrupt/SystemExit)
  MUT       a mutable default argument
"""
import ast
import sys
from pathlib import Path


def audit(path):
    src = path.read_text(encoding="utf-8")
    tree = ast.parse(src, filename=str(path))
    out = []

    # every Name/Attribute-root actually used
    used = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Name):
            used.add(n.id)
        elif isinstance(n, ast.Attribute):
            r = n
            while isinstance(r, ast.Attribute):
                r = r.value
            if isinstance(r, ast.Name):
                used.add(r.id)
    # decorators / string annotations can hide uses; also scan raw text as a
    # deliberately conservative fallback so we under-report rather than
    # recommend deleting something live.
    for n in ast.walk(tree):
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            for a in n.names:
                name = (a.asname or a.name).split(".")[0]
                if name == "*":
                    continue
                if name not in used and src.count(name) <= 1:
                    out.append((n.lineno, "IMPORT", f"'{name}' imported, never used"))

    toplevel = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    for n in toplevel:
        if n.name.startswith("_") or n.name in ("main",):
            continue
        if src.count(n.name) <= 1:
            out.append((n.lineno, "DEADFN", f"'{n.name}' defined, never referenced in this file"))

    for n in ast.walk(tree):
        if isinstance(n, ast.ExceptHandler):
            body_is_pass = len(n.body) == 1 and isinstance(n.body[0], ast.Pass)
            if n.type is None:
                out.append((n.lineno, "BARE", "bare 'except:' catches SystemExit/KeyboardInterrupt too"))
            if body_is_pass:
                out.append((n.lineno, "SILENT", "except ...: pass -- failure swallowed, reports success"))
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for d in n.args.defaults + [k for k in n.args.kw_defaults if k]:
                if isinstance(d, (ast.List, ast.Dict, ast.Set)):
                    out.append((n.lineno, "MUT", f"{n.name}(): mutable default argument"))
    return sorted(out)


def main():
    files = [Path(p) for p in sys.argv[1:]]
    totals = {}
    for f in files:
        try:
            res = audit(f)
        except SyntaxError as e:
            print(f"\n{f}:  SYNTAX ERROR {e}")
            continue
        if not res:
            continue
        print(f"\n--- {f} ---")
        for lineno, kind, msg in res:
            print(f"  {lineno:>5}  {kind:<7} {msg}")
            totals[kind] = totals.get(kind, 0) + 1
    print("\n=== totals ===")
    for k in sorted(totals):
        print(f"  {k:<8} {totals[k]}")
    if not totals:
        print("  clean")


if __name__ == "__main__":
    main()
