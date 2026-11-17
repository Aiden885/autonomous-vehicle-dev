# APA Principle Figures Implementation Plan

**Goal:** Extend the approved chapter-six illustrated style to the remaining explanatory chapters, using the right visual form for each principle.

**Scope:** Chapters 2-5 and 7-13; clarify the chapter 6 figure relationship. Retain existing chapter structure and chapter 14 onwards byte-for-byte. No model or algorithm modifications.

**Architecture:** Reuse the local SVG/PNG drawing and text-bound verification utilities. Pair each figure with its source references, classification and a concise explanation. Numerical/geometric examples are pedagogical, not claimed runtime results.

**Tech Stack:** Python 3.8, XML, SVG, Pango/Rsvg/Cairo; already available locally.

## Tasks

- [x] Back up the current document and review source/model evidence.
- [x] Draw 15 figures covering overall coordination, coordinates/time, environment, slot memory, geometry/sweep, search/SAT, candidate selection, control, braking, scheduling and delay memory.
- [x] Check geometric identities, illustrative data, source mappings and text bounds; render and inspect every figure.
- [x] Embed figures with explicit cross-references and captions; clarify figure 6-1 versus 6-4.
- [x] Verify headings, links, appendix preservation and original model/MATLAB hashes; update WORKLOG.

## Verification Result

15 new diagrams, 19 figures embedded in total. All figure references resolve; shape bounds cover 1,295 checked polygon/polyline vertices. Original chapter headings and anchors retained; chapter 14 onwards byte-identical. All 190 original model/MATLAB hashes match. Teaching calculations are tested independently of the original algorithm; no simulation or physical validation is claimed. Detailed evidence: `backups/apa_principle_figures_20261113/checks.json` and the two asset directories' `source_evidence.json` files.

## Correctness Boundaries

- Responsibility diagrams are not literal wire maps or guaranteed invocation order.
- Coordinate and vehicle dimensions in examples must not silently become project interface conventions or calibration.
- Search illustrations are not claims of actual search trees, optimality or full collision-free feasibility.
- Pure pursuit bearing angle differs from heading error.
- Nominal scheduling is conditional on the original base invocation; do not fabricate measured phases.
- All figures must identify the relationship to earlier figures and actual modules.
