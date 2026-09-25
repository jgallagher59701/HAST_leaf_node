# Task Plan: Extract Arduino-free logic layer out of `src/leaf_node.cc`

**Task:** TASK-001
**Status:** Draft
**Created:** 2026-09-22

## Summary

`src/leaf_node.cc` includes `Arduino.h` and mixes hardware access with
business/decision logic in one 977-line file, which violates the `CLAUDE.md`
rule to keep algorithmic logic in an `Arduino.h`-free layer and confine
`Arduino.h`/`Serial`/`Wire`/`SPI`/register access to a thin hardware-adapter
layer. This task pulls the pure, register-independent pieces of that file (and
of `src/get_battery_voltage.cc`) into a new `Arduino.h`-free translation unit,
`src/leaf_node_logic.cc` / `include/leaf_node_logic.h` — following this
project's existing `src/`+`include/` convention (as used by `blink`/
`get_battery_voltage`), not a separate PlatformIO library — each backed by a
native Unity test, and leaves `leaf_node.cc` as the hardware-adapter/
orchestrator that calls into it.

## Motivation

Per `docs/deep-dives/leaf-node-cc.md` (2026-09-18), `leaf_node.cc` is a
single-file Arduino sketch with no separation between hardware access and
logic, and `env:native`'s `test_filter = native_*` currently has nothing to
match for it. `lib/soil_sensor_common` already follows the `Arduino.h`-free
rule (its include is commented out in both header and source — confirmed in
`docs/deep-dives/soil-sensor-common.md`), but that directory holds code that
lives in its own separate GitHub repo (`lib/soil_sensor_common/.git`) — not the
pattern to copy structurally. The new logic here is leaf-node-specific and not
shared with `HAST_lora_main`, so it belongs alongside `blink`/
`get_battery_voltage` in `src/`+`include/`, as a plain project file, not a new
library or repo.

## Related requirements / constraints

- **IC-001** — Firmware targets a RAM-constrained ARM M0 board; its
  "how" column explicitly cites this rule ("drives the `Arduino.h`-free logic
  layer / thin hardware-adapter split in `CLAUDE.md`"). This task is a direct
  compliance action for IC-001, not new capability.
- No FR/NFR/UC changes — no user-visible behavior is intended to change.

## Scope

**In scope** — six extractions, each: (a) a pure function moved to
`src/leaf_node_logic.cc` / `include/leaf_node_logic.h`, (b) a native Unity test
for it, (c) the call site in `src/leaf_node.cc` or
`src/get_battery_voltage.cc` updated to use it:

1. **Log-filename rollover** (`get_new_log_filename()` in `leaf_node.cc`,
   currently lines ~301–326): the numeric-suffix bump/wrap algorithm
   (`00` → `01` → … → `99`) operates purely on the in-memory `file_name`
   buffer and a base-name length; only `sd.exists()` is a hardware call.
   Extract the bump step itself, e.g. `bool bump_log_file_name(char *name,
   uint8_t base_name_size)`, returning `false` once the suffix is already
   `99` (names exhausted) and `true` otherwise, with no side effect on `name`
   in the `false` case. **Preserve current behavior exactly**: the adapter's
   `while (sd.exists(name))` loop keeps calling this each iteration and, on
   `false`, keeps setting `status |= SD_NO_MORE_NAMES` and keeps returning
   `BASE_NAME99.csv` (silent reuse) rather than refusing to log — per this
   task's instructions, this exhausted-name fallback is being preserved as-is,
   not revisited here.
2. **Compile-time epoch parsing** (`get_epoch()`, lines ~266–284): already has
   no Arduino dependency (`<string.h>`/`<time.h>` only) — move as-is into
   `src/leaf_node_logic.cc`, add the test it never had.
3. **Time-sync delta/threshold decision** (`update_time()`, lines ~547–563):
   split the pure decision — given `main_node_time` and the node's current
   epoch, compute the delta and decide whether it exceeds the 1-second
   threshold — from the hardware actions (`rtc.getEpoch()`, `rtc.setEpoch()`).
   Signature like `bool should_resync_time(uint32_t main_node_time,
   uint32_t local_time, int32_t *out_delta)`; `leaf_node.cc` reads the RTC,
   calls this, and calls `rtc.setEpoch()` only if it returns true.
4. **ADC-counts-to-voltage conversion** (`get_battery_voltage()` in
   `src/get_battery_voltage.cc`, lines ~26–54): split the averaging/scaling
   formula (`voltage = 4.46 * (avg / ADC_MAX_VALUE)`, `+ VOLTAGE_OFFSET`,
   `* 100`) from the `analogRead()` sampling loop. Signature like
   `int adc_average_to_centivolts(long raw_sum, int n_samples)`; the adapter
   keeps the `analogRead()` loop and calls this with the accumulated sum.
5. **Time-request cadence check** (`loop()`, line ~932:
   `message % TIME_REQUEST_SAMPLE_PERIOD == 0`): trivial but currently
   inline and untested. Extract `bool should_request_time(uint32_t
   message_count, uint32_t period)`.
6. **Status-byte low-nibble clear** (`loop()`, line ~914:
   `status = status & 0xF0`): extract `uint8_t clear_transient_status(uint8_t
   status)` documenting the "high nibble = persistent boot errors, low nibble =
   per-cycle errors" convention that's currently only in a comment.

For each, the native test covers at minimum: the normal case, the boundary
condition called out above (99→exhausted, exactly 1s delta, period boundaries,
0xFF input), and one clearly-invalid input where the function's contract says
what should happen (e.g. `n_samples == 0`).

**Out of scope:**

- Hardware-sequencing functions with no separable decision logic:
  `shutdown_sd_card()`, `wake_up_sd_card()`, `sleep_node()`, `send_message()`,
  `receive_message()`, `radio_silence()`, `write_header()`, `log_data()`,
  `debug_log()`, `init_state_pins()`/`clear_state_pins()`/`set_state_pin()`,
  and all of `setup()`'s hardware-init sequencing. These stay in
  `leaf_node.cc` as the hardware-adapter layer; if a future pass finds
  separable logic inside them, that's a new task, not scope creep here.
- `src/blink.cc` — already a pure `digitalWrite`/`delay` sequencer with no
  extractable logic.
- `lib/soil_sensor_common/*` — already `Arduino.h`-free; not touched.
- Any behavior change. If an extraction reveals what looks like a bug (e.g.
  an off-by-one in the rollover wrap, or the broadcast-vs-ack question already
  flagged in `docs/deep-dives/leaf-node-cc.md`), note it and stop — file a
  `BUG-###` via `/fix-bug` rather than fixing it inline in this task.
- Splitting `leaf_node.cc` itself into multiple adapter files, or introducing
  a formal C++ interface/abstract class for the hardware adapter. The rule
  requires logic to live outside `Arduino.h`-including code, not a specific
  file layout for the adapter side; a single orchestrator file that no longer
  contains business logic satisfies it.
- `get_battery_voltage_setup()` (ADC reference/resolution configuration) —
  pure hardware register setup, nothing to extract.

## Approach

1. **Scaffold the files.** Create `include/leaf_node_logic.h` and
   `src/leaf_node_logic.cc`, matching the existing `include/`+`src/` pairing
   used by `blink`/`get_battery_voltage` — plain project files, no
   `library.json`, no separate repo. Add the `build_src_filter` to
   `[env:native]` in `platformio.ini` (see below) excluding
   `leaf_node.cc`/`blink.cc`/`get_battery_voltage.cc` from the native build,
   before moving any real code, then confirm `env:native` builds and runs one
   placeholder test (`test/native_leaf_node_logic/test_placeholder.cc`) — this
   isolates "the build wiring works" from "the extraction is correct."
2. **Extract one function at a time**, in the order listed above (simplest/
   already-portable first: epoch parsing, then cadence check and status
   nibble, then the rollover algorithm, then the time-sync split, then the
   battery-voltage split — saving the two that require splitting an existing
   function for last). For each:
   - Write the native test first against the *current* inline behavior
     (as a spec of what must not change).
   - Move/adapt the code into `src/leaf_node_logic.cc` /
     `include/leaf_node_logic.h`.
   - Update the call site in `src/leaf_node.cc` or
     `src/get_battery_voltage.cc` to call it.
   - Run `pio test -e native` (must pass) and `pio run -e zeroUSB` (must still
     build) before moving to the next function.
3. **Completeness check.** Before/after each extraction, `grep -n` the
   function name in `src/leaf_node.cc` / `src/get_battery_voltage.cc` to
   confirm the inline logic was removed, not duplicated. At the end, the six
   items above should each appear exactly once, in `src/leaf_node_logic.cc`,
   with a corresponding call site in `src/`.
4. **Final pass:** confirm `src/leaf_node.cc` and `src/get_battery_voltage.cc`
   still legitimately need `Arduino.h` (they do — they retain the hardware
   calls) and that `include/leaf_node_logic.h` / `src/leaf_node_logic.cc` have
   no `Arduino.h` include, matching the `lib/soil_sensor_common` precedent for
   the include discipline itself (not its repo/directory structure).

## Verification

- `pio test -e native` passes, including the six new test files.
- `pio run -e zeroUSB` (and `-e debugZeroUSB`) still build clean.
- `pio check` runs clean on changed files, or any new warning is justified
  inline per `CLAUDE.md`.
- Manual diff review of `src/leaf_node.cc` / `src/get_battery_voltage.cc`
  confirms each extracted block was replaced with a call, not left duplicated.
- `grep -l "Arduino.h" src/leaf_node_logic.cc include/leaf_node_logic.h`
  returns nothing.
- On-target smoke test (`pio run -e zeroUSB -t upload` + one wake/sleep/LoRa
  cycle observed over serial with `DEBUG=1`) to catch anything a native test
  can't, since this task has no access to the real ADC/RTC/SD/radio.

## Risks

- **Silent behavior change disguised as refactor.** The rollover algorithm and
  the time-sync threshold both have edge cases (99→exhausted, exactly-1-second
  delta) that are easy to get subtly wrong when re-typing them into a new
  function signature. Mitigated by writing the test against current behavior
  *before* moving the code (step 2). Same risk applies more broadly any time a
  single dual-purpose function is split into a pure part and a hardware part.
- **`data` naming collision.** `leaf_node.cc:900` already declares a local
  `data_message_t data;` in `loop()`; if new logic-layer parameters are also
  named `data`, keep them scoped to the new functions to avoid confusing
  reads during review (not a compile risk, just a review-clarity one).
- **`env:native`'s `build_src_filter` silently going stale.** If a future
  change adds another Arduino-dependent file to `src/` without adding it to
  the filter's exclusion list, `env:native` breaks the next time anyone runs
  `pio test -e native` (loud, not silent) — annoying but not a correctness
  risk, since a broken native build can't pass CI/local checks unnoticed.

## Open questions

- **Resolved:** logic code lives in `src/leaf_node_logic.cc` /
  `include/leaf_node_logic.h` as plain project files — no `lib/`, no separate
  git repo. `lib/` is reserved for code (like `soil_sensor_common`) that lives
  in its own external repo; this logic doesn't.
- **Resolved:** the `DataNN.csv` naming behavior, including silent reuse of
  `BASE_NAME99.csv` once names are exhausted, is preserved exactly as-is —
  not revisited by this task.
- **Resolved:** native build wiring uses a `build_src_filter` on
  `[env:native]` (see below) — confirmed 2026-09-23.

### `env:native` will try to compile all of `src/` — resolved via `build_src_filter`

`[env:native]` in `platformio.ini` sets `test_build_src = yes`, and there's
currently no `build_src_filter` on that environment. Read literally, that
means PlatformIO will attempt to compile *every* file in `src/` — including
`leaf_node.cc`, `blink.cc`, and `get_battery_voltage.cc` — as part of the
native test build, alongside whatever's under `test/`. Those three files
include `Arduino.h`, which doesn't exist for `platform = native`, so the
native build would fail to compile the moment it reaches them.

This is a latent, pre-existing gap, not something this task introduces —
`test/` is currently empty, so `env:native` has never actually been exercised
with real code, and the comment already in `platformio.ini` ("must add the
`UNIT_TEST` guard around `setup()` and `loop()`") suggests it was anticipated
but never finished. It only becomes a live blocker now because this task's
entire point is to finally add native tests, and putting the new logic file
directly in `src/` — as asked — puts it in the same directory PlatformIO
would try to build wholesale.

**Decision (confirmed 2026-09-23):** add a `build_src_filter` to
`[env:native]` excluding the Arduino-dependent files by name, e.g.:

```ini
[env:native]
platform = native
build_src_filter = +<*> -<leaf_node.cc> -<blink.cc> -<get_battery_voltage.cc>
```

so only `leaf_node_logic.cc` (and any future Arduino-free `src/` file) builds
under native. This is a single, self-contained change to `[env:native]` in
`platformio.ini` and doesn't touch any hardware-adapter file. It needs one
more filter entry each time a new Arduino-dependent file is added to `src/` —
a small, visible maintenance cost, and step 1 below should call this filter
out explicitly so it isn't forgotten on the next new `src/` file.
