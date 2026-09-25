<!-- CONTRACT: scoped engineering/maintenance work that isn't new capability (FR/NFR/
     UC) and isn't corrective (BUG) — dependency bumps, removing dead code or stale
     compile-time directives, retrofitting tests onto code that predates them,
     build/tooling changes. The common thread: the "right" outcome is usually
     mechanical and completeness matters (did every instance get handled?), not
     open-ended design. Delete the TASK-EXAMPLE section once you've seen the shape. -->

# Task Log

Convention: `TASK-###`, sequential, never renumbered. Status: `Open` → `In Progress`
→ `Done` (or `Won't Do` / `Superseded by TASK-0XX`). Category is one of: Dependency
Update, Code Cleanup, Test Coverage, Build/Tooling, Documentation (add categories as
needed).

---

## TASK-EXAMPLE — Add unit tests for the legacy `parser/` module

*(delete this section once you've seen the shape)*

**Category:** Test Coverage
**Status:** Plan Ready
**Created:** YYYY-MM-DD
**Related requirements/constraints:** None found — `parser/` predates FR/NFR/UC
tracking in this repo
**Plan:** `plans/task-parser-test-coverage-plan.md`

**Scope:** Every public function in `parser/`, tested against the fixtures already
in `parser/testdata/`. Does not include refactoring the module itself, even where
the tests reveal awkward internal structure — that's a separate task if pursued.

**Motivation:** `parser/` has zero test coverage and has caused two regressions in
the last quarter that tests would have caught before merge.

---

## TASK-001 — Extract Arduino-free logic layer out of `src/leaf_node.cc`

**Category:** Code Cleanup
**Status:** Plan Ready
**Created:** 2026-09-22
**Related requirements/constraints:** IC-001 (drives the `Arduino.h`-free logic
layer / thin hardware-adapter split called for in `CLAUDE.md`); see also
`docs/deep-dives/leaf-node-cc.md` (2026-09-18), which already flagged that this
file has no separate hardware-adapter layer and zero native test coverage, and
`docs/deep-dives/soil-sensor-common.md`, which confirms `lib/soil_sensor_common`
already complies and can serve as the pattern to follow.
**Plan:** `plans/task-leaf-node-logic-extraction-plan.md`

**Scope:** Extract the identifiable pure/algorithmic pieces embedded in
`src/leaf_node.cc` and `src/get_battery_voltage.cc` — log-filename rollover
(keeping the existing `DataNN.csv` behavior, including silent reuse of
`Data99.csv` once names are exhausted, exactly as-is), compile-time epoch
parsing, the time-sync delta/threshold decision, the ADC-counts-to-voltage
conversion formula, and the periodic time-request-cadence check — into a new
`Arduino.h`-free pair of project files, `src/leaf_node_logic.cc` /
`include/leaf_node_logic.h` (following the existing `src/`+`include/`
convention already used by `blink`/`get_battery_voltage` — not a new library,
not a separate repo), each backed by an `env:native` Unity test. `leaf_node.cc`
and `get_battery_voltage.cc` are updated to call the extracted functions
instead of inlining the logic; they keep `Arduino.h` and remain the thin
hardware-adapter/orchestrator layer. Does not include re-architecting the
hardware-sequencing functions that have no separable logic (`sleep_node()`,
`send_message()`, `receive_message()`, SD/RTC/radio init sequencing), fixing
any of the behavioral findings already logged in the `leaf-node-cc.md` deep
dive, or touching `src/blink.cc` / `lib/soil_sensor_common`, both of which are
already compliant. A related open item — `env:native` currently has no
`build_src_filter`, so it would try to compile the Arduino-dependent files in
`src/` alongside the new logic file — is flagged in the plan for a decision
before implementation starts.

**Motivation:** `src/leaf_node.cc` is a 977-line single file that includes
`Arduino.h` and mixes hardware access with sequencing/decision logic throughout,
in direct violation of the `CLAUDE.md` rule highlighted for this task. It also
currently has zero native unit test coverage for any of its logic, which the
`Arduino.h` dependency makes structurally impossible under `env:native` today.

---

<!-- Add new tasks via /plan-task, or by hand — keep the section format: metadata
     lines, then Scope / Motivation. Unlike a bug, a task doesn't need a symptom or
     reproduction — it needs a clear boundary of what's included and what isn't,
     since "remove all the X" quietly becoming "remove most of the X" is the
     characteristic failure mode here. -->
