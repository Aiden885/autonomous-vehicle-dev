# APA State Illustrations Implementation Plan

> Execution: native, within this session. Documentation only; no algorithm, model or canvas changes.

**Goal:** Add a readable, source-checked illustrated sample to chapter 6 of the existing APA explanation.

**Architecture:** Read the original SLX Stateflow Chart 17000; validate state containment and selected transitions before producing static SVG/PNG diagrams. Embed PNGs in the existing Markdown, retaining SVG sources and machine-readable evidence.

**Tech Stack:** Python, XML, SVG, locally available Rsvg/Cairo; no additional installation.

**Spec:** User-approved hierarchy + transition + phase illustration sample; figures must explain rather than replace prose. Real model relationships and conceptual examples must be clearly separated.

## Constraints And Review Focus

- Preserve all major chapters and chapter 14 onwards byte-for-byte.
- Source every real state and arrow from the original model, including composite/default-entry paths.
- Do not treat a guard summary as a complete sufficient condition or suppress transition priority caveats.
- Draw parent-level exits at the parent level, not as invented direct child transitions.
- Label the phase example as conceptual, without fabricated time or measurement values.
- Keep text legible and arrows unambiguous; inspect rendered diagrams.

## Tasks

- [x] Back up the current document; extract source evidence and validate relationships.
- [x] Generate four diagrams: state containment, one normal park-in route, selected wait/protection transitions, task/maneuver coordination.
- [x] Insert diagrams with concise reading notes into chapter 6; retain other chapters.
- [x] Verify sources, figure references, text bounds and preservation; inspect rendered figures and update WORKLOG.

## Results

Checked 23 runtime states and 20 source transitions, including default-entry paths and the parent-level exit. All four SVG/PNG pairs rendered and inspected; text bounds passed. Only chapter 6 changed; all 190 model/MATLAB source hashes match the baseline. No simulation or physical validation claimed. Evidence and document backup: `backups/apa_state_diagrams_20261113/`.
