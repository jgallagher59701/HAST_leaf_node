---
description: Investigate an area of the existing codebase read-only, check it against the requirement docs, and save findings
argument-hint: <module/area/question — e.g. "how does auth work" or "src/billing">
---

Investigate: $ARGUMENTS

If `$ARGUMENTS` is empty, ask what to look at — a directory, a subsystem, or a
specific question.

## Steps

1. **Explore read-only.** Map the files and modules actually involved: structure, key
   abstractions, control/data flow, external dependencies, anything that stands out as
   a risk area (untested, tightly coupled, doing more than its name suggests). Stay
   descriptive — report what's there, and only editorialize about quality if asked to.

2. **Cross-check against `docs/requirements/` and `docs/constraints/`.** For any
   FR/NFR/UC/IC that plausibly relates to this area, note whether the code appears to
   satisfy it. Phrase mismatches as **observations to verify**, not settled fact —
   your read of the code or the docs' currency could both be wrong. If something looks
   like drift (docs say one thing, code does another), flag it explicitly rather than
   quietly assuming either side is the correct one.

3. **Summarize conversationally first.** The point of a deep dive is usually an
   immediate answer, not a file.

4. **Save findings to `docs/deep-dives/<slug>.md`**, dated, with the question that
   prompted it at the top (see `docs/deep-dives/README.md` for why). Confirm before
   overwriting an existing file with the same slug — a later deep dive on the same
   area might be meant to update it or might be a different question that deserves its
   own name.

5. **Don't edit code or the requirement docs.** If the investigation surfaces a bug,
   hand off to `/fix-bug` with what you found rather than continuing to dig here. If it
   surfaces a gap or drift worth formalizing, hand off to `/new-requirement`. If it
   surfaces mechanical cleanup or maintenance work — dead code, stale compile-time
   directives, out-of-date dependencies, missing test coverage — hand off to
   `/plan-task`. If it was investigation ahead of building something new, hand off to
   `/plan-feature`.
