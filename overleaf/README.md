# Overleaf report

English-language LaTeX report for the project supervisor. This is a
**skeleton only** — section headers and TODO placeholders, content to be
written in a later pass from the material in `research-log/` (kept outside
this repository, see the project root notes).

## Status

First full draft written 2026-07-24 (abstract, setup, architecture,
methodology F3-F5 + ground-truth protocol, results-to-date, next steps,
selected lessons learned, references). Not yet reviewed by the supervisor
or compiled/proofread on Overleaf.

## Source material

Content will be synthesized from `research-log/journal.md` (the engineering
log kept throughout the project) and the other `research-log/*.md` files as
needed — not a re-edit of the original documents left by the previous
intern. Confirmed direction, 2026-07-24.

## Usage

Copy `main.tex` (and any files added under here later) into an Overleaf
project and compile there — compilation is not done from this repository.

## `technical.tex` — the complete technical manual

A second, separate document from `main.tex`: not a progress report, a
**self-sufficient A-to-Z setup and operation manual** (power-on through
autonomous docking), requiring no other document, inherited tutorial, or
vendor PDF to follow. First full draft written 2026-08-03, out of the
`research-log/23-inventaire-doc-sources.md` document-triage pass (38 source
documents read/dispositioned) and the `08-guide-demarrage.md` coherence
audit the same day.

**Structure:** a single file, `technical.tex` — 10 chapters (introduction,
powering on, rooting the S1, Raspberry Pi setup, network (RNDIS), the
camera/robot bridge, camera calibration, building and launching Carolus
(including the full current `testcarolus.launch`), the operator GUI,
autonomous docking), consolidated into one file on 2026-08-03 (originally
split across a `chapters/` folder for the first draft; merged back down to
one file to keep this folder's file count low while the manual is still a
single, unreviewed deliverable).

**Content rule, deliberately different from `main.tex`:** every program and
parameter value shown is the **current, corrected** version — no
bug/ADR narrative, no "here's what used to be wrong." That history lives in
`research-log/journal.md` only, per this project's single-source-of-truth
rule; this manual exists so a reader never needs it.

**Audited 2026-08-03** against 10 self-generated verification questions
(beginner-completion, matching-configuration, correct operation order,
provenance honesty, etc.) — found and fixed 7 real gaps, the largest being
that the manual never explained how to get this project's own source code
onto the Pi/lab PC in the first place (now §"Getting this project's code
onto..." in the Raspberry Pi and Carolus-build chapters).

**Usage:** copy `technical.tex` (a single self-contained file, no other
files in this folder needed for it) into an Overleaf project and compile
there.

**Compiled successfully 2026-08-03** (`pdflatex`, two passes, after the user
installed `texlive-latex-base`/`texlive-latex-recommended`/`texlive-fonts-recommended`
locally) — `technical.pdf`, 36 pages, no errors, no undefined references,
only 7 benign "Overfull \hbox" warnings (long `\texttt{}` ROS topic/package
names that can't hyphenate mid-identifier). **Not yet done:** proofread by
a human, and not yet reviewed by the supervisor.
