# Plan: SD-card debug log

**Status:** In Progress (implemented in `src/leaf_node.cc`/`include/debug.h`; builds
clean for `env:zeroUSB` with debug flags both on and off; not yet verified on
hardware, so `FR-009`'s status is left as the user's call to bump to `Implemented`)
**Created:** 2026-09-20

## Summary

Add an append-only, timestamped debug log file on the SD card that captures the leaf
node's diagnostic output — information useful for understanding program flow/timing
during testing, without the execution-halting cost of a debug probe — when debug
mode is enabled. `loop()`-time USB-serial output is dropped entirely in favor of
this; `setup()`-time serial output is kept, under debug-probe-safe initialization.
FR-008's LoRa-routing clause turns out to describe a different, unrelated behavior
(real-time *error* reporting to the main node, not debug tracing) that this plan
leaves functionally untouched, renaming only the function that does it to remove the
naming collision with the new "debug" concept.

## Requirements traced

- FR-009 — Leaf node writes debug diagnostics to a log file on the SD card when
  debug mode is enabled
- FR-010 — Leaf node reports hardware/SD-card errors to the main node over LoRa when
  they occur. Newly added to cover `lora_debug()`'s actual behavior, which FR-008 had
  miscategorized as "debug diagnostics." This plan keeps that behavior unchanged,
  renaming only the function (Phase 2).
- FR-008 — marked `Superseded by FR-009`. This plan finds that only its
  serial-routing clause is actually replaced by FR-009; its LoRa-routing clause is
  now covered by FR-010 instead, since on inspection it reports genuine SD-card/
  hardware errors to the main node, not debug diagnostics.
- UC-001 — main flow step 4 (SD logging) and the "Debug mode enabled" alternate
  flow are both touched by this change

## Constraints considered

- **IC-001** (SAMD21, ~32 KB RAM, no dynamic allocation after `setup()`): the new
  debug-log write path must reuse fixed-size buffers, the same way
  `data_message_to_string`'s static `decoded_string` buffers and `log_data()`'s
  stack-based `error_info[256]` already do. No `String`, no `new`/`malloc`.
- **IC-003** (battery-only, unattended field deployment): IC-003 itself has no
  overhead threshold to satisfy — it's the boundary condition (no mains power) that
  makes SD-write time/power relevant at all, feeding the actual measurable targets
  in NFR-001/NFR-002 (>1yr / >2yr battery life). Given that, the debug log should add
  no SD-write activity when debug mode is *off* — the default, field configuration.
  Since the mechanism is gated behind a compile-time flag exactly like today's
  `DEBUG`/`LORA_DEBUG`, a non-debug build should see zero additional SD activity.
  This needs to hold in practice, not just in principle — see Phase 3. (Confirmed
  post-implementation: `env:zeroUSB` RAM/flash usage is byte-identical with the new
  code present but its flags off — evidence the default build carries no added SD
  activity, not a measurement of actual field battery life against NFR-001/002.)
  This plan does not add a way to enable debug mode at runtime, which would
  conflict with IC-003's unattended-deployment premise (no path for a technician to
  toggle it without physically reflashing or re-provisioning the node).
- **IC-004** (no radio besides LoRa to the main node): `lora_debug()` (renamed under
  FR-010, not removed) is the correct use of the single LoRa link for genuine error
  reporting; moving actual debug tracing off of LoRa and onto the SD log (FR-009)
  keeps that link free for the primary reliable-datagram protocol and FR-010's error
  reports, rather than competing with them. Consistent with IC-004's intent, not a
  conflict.

No constraint is violated by this plan.

## Phases

### Phase 1 — Add the debug log file and a `debug_log()` write function

**Goal:** A single, fixed-name SD file (e.g. `Debug.log`, opened `O_APPEND` —
*not* the numbered-rollover scheme `DataNN.csv` uses) that diagnostic output can be
appended to across boots.
**Satisfies:** FR-009

Steps:
1. Add a fixed debug-log filename constant (e.g. `#define DEBUG_LOG_FILE_NAME
   "Debug.log"`). No unused-filename search, no per-boot rollover — every boot
   appends to the same file, unlike `get_new_log_filename()`'s `DataNN.csv` scheme.
2. Add `void debug_log(const char *msg)` as its own function, distinct from
   `log_data()`: they write to different files for different purposes
   (`log_data()` → the numbered sensor-data CSV; `debug_log()` → the single
   append-only debug file), and `debug_log()` is expected to gain call-site-specific
   behavior over time (e.g. timestamps — see step 3) that `log_data()` shouldn't
   carry. Structurally it still follows `log_data()`'s open/write/close-per-call
   pattern (safe across the standby/wake cycle, since neither file is held open
   during sleep).
3. `debug_log()` prepends a date/time stamp to each line, read from the RTC (already
   running and synced per FR-005) — e.g. the same `%d/%d/%dT%d:%d:%d` format
   `setup()` already builds into `date_str` (`src/leaf_node.cc:718`), into a fixed
   stack buffer, followed by the message. This is what makes the log useful for
   understanding timing/ordering during testing without a debug probe.
4. Gated on `DEBUG_LOG` (via `IO_LOG(...)`, distinct from the setup()-time
   `SERIAL_DIAG`/`IO(...)` flag — see Phase 3): a non-debug build should never open
   or write this file.

**Risks:** A second SD file means a second point of write failure per cycle. Needs
its own status bit or reuse of `SD_FILE_ENTRY_WRITE_ERROR` — minor, not blocking.

### Phase 2 — Rename `lora_debug()` to remove the naming collision with the new debug log

**Goal:** Keep `lora_debug()`'s actual behavior — reporting real errors (e.g. an SD
write failure) to the main node over LoRa — unchanged; only its name changes, since
"debug" now collides with the unrelated concept this plan introduces.
**Satisfies:** FR-010 (clarifies naming only; behavior is unchanged) — see
correction below for why this isn't FR-009

**Correction from initial draft:** the `lora_debug()` call inside `log_data()`'s
SD-write-failure branch is not a debug message and is out of scope for FR-008/FR-009
— it is real-time error reporting to the main node, which this plan does not change.
Treating its removal as blocked on "can we safely write to the SD card that just
failed" was a red herring: nothing about this plan requires touching that call site's
behavior, only its name.

Steps:
1. Rename `lora_debug()` to something that reflects what it actually does, e.g.
   `report_error_to_main_node()` (exact name: open question below — not load-bearing,
   easy to pick at implementation time).
2. Update its one call site in `log_data()` accordingly. No behavior change.
3. `LORA_DEBUG` (the build flag gating it) can keep its name or be renamed to match;
   low stakes either way since it's a single local `#define`, not a shared constant.

**Risks:** None — pure rename, no behavior change, single call site.

### Phase 3 — Redirect `loop()` diagnostics to the debug log; drop Serial except in `setup()`

**Goal:** All `IO(Serial...)` call sites in `loop()` (and the helper functions it
calls, e.g. `send_message`/`receive_message`) move to `debug_log()`. Serial output
is dropped entirely there. `setup()`-time `IO(Serial...)` calls are kept, but only
under a debug-probe-safe serial initialization — not the assumption that a USB host
is listening.
**Satisfies:** FR-009 (supersedes FR-008's serial-routing clause for `loop()`;
narrows, rather than fully replaces, it for `setup()`)

Steps:
1. Repoint every `IO(Serial...)` call site inside `loop()` (state prints, message
   contents, elapsed-time output, the `time_response` switch's default case, etc.)
   at `debug_log(...)`.
2. Keep `IO(Serial...)` call sites inside `setup()` as Serial output, but require
   they only run under the existing bounded, non-blocking connection sequence
   already present at `src/leaf_node.cc:700-706`: `Serial.begin()`, then a
   *bounded* `while (!Serial)` loop capped at `SERIAL_CONNECT_TRIES` iterations of
   `SERIAL_CONNECT_INTERVAL` ms, never an unbounded wait. This is the "debug-probe
   safe" property: `setup()` must complete (and eventually reach `USBDevice.detach()`
   under `STANDBY_MODE`) whether or not a USB terminal is actually attached — e.g.
   when running under the `env:debugZeroUSB` SWD/jlink target
   (`platformio.ini:77-88`), which attaches via SWD, not USB-serial, and must not
   hang waiting on a serial connection that will never come. No behavior change here
   versus today — this phase confirms/keeps the existing pattern rather than
   introducing a new one, and is called out explicitly so it isn't accidentally
   deleted while dropping `loop()`-time serial.
3. Update the boot-error `blink()`/status-bit path (`SHT31_BEGIN_FAIL`,
   `SD_BEGIN_FAIL`, etc.) to also write to the debug log where the SD card is
   confirmed working (i.e. not the `SD_BEGIN_FAIL` case itself) — in addition to,
   not instead of, the `setup()`-time Serial output.

**Risks:** Call-site volume (~30 sites total, split between `setup()` and `loop()`)
makes this the largest phase; low individual risk since each `loop()` site is a
mechanical destination swap. Main risk is accidentally weakening the bounded-wait
Serial init while touching this code — step 2 is a "leave alone" instruction, not a
rewrite, precisely to avoid that.

### Phase 4 — Update comments/build-flag documentation

**Goal:** `src/leaf_node.cc`'s debug-flag comment block (lines 35-38) reflects the
new behavior, naming `SERIAL_DIAG` (renamed from `DEBUG`), `DEBUG_LOG`, and
`LORA_ERROR_REPORT` (the flag behind `report_error_to_main_node()`). `platformio.ini`
also passes these as build flags (`-D SERIAL_DIAG=0`, etc.) and has already been
updated to the new name; the in-file `#ifndef`/`#define` block is only the
fallback default when a flag isn't supplied at build time.
**Satisfies:** FR-009

Steps:
1. Update the comment at `src/leaf_node.cc:35-38` describing what the (possibly
   renamed) debug flag now does.

**Risks:** None — documentation only.

## Open questions

Both naming questions below are resolved:

- **What should `debug_log()`'s siblings be called?** `debug_log()` and `IO_LOG()`
  keep their names. The `setup()`-only serial-diagnostics flag is renamed `DEBUG` →
  `SERIAL_DIAG` (its macro `IO()` keeps its name); `lora_debug()`'s flag renamed to
  `LORA_ERROR_REPORT` to match its FR-010 rename below. The three are now distinct
  by name as well as behavior: `debug_log()`/`DEBUG_LOG` → SD log, `SERIAL_DIAG`/
  `IO()` → Serial at boot, `LORA_ERROR_REPORT` → main node over LoRa.
- **What exact name replaces `lora_debug()`?** `report_error_to_main_node()`.

## Follow-up (outside this plan)

- **Done:** FR-010 ("Leaf node reports hardware/SD-card errors to the main node over
  LoRa when they occur") was added via `/new-requirement` to own the
  `lora_debug()` behavior FR-008 had miscategorized as debug diagnostics. Phase 2 now
  satisfies FR-010, not FR-009.

## Out of scope

- Any change to the data-sample CSV log (`DataNN.csv`, FR-002) — untouched by this
  plan.
- A runtime (non-compile-time) way to toggle debug mode in the field — would need
  its own requirement and conflicts with IC-003's unattended-deployment premise
  unless carefully designed; not assumed here.
- Changes to `HAST_lora_main` (the main node) — FR-008/FR-009 are leaf-node-only
  requirements; the main node's own debug/logging behavior (`TFTDisplay`, etc.) is
  untouched.
- `lib/soil_sensor_common` changes — this plan only touches `src/leaf_node.cc`.
