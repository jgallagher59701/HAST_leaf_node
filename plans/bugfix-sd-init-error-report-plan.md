# Bugfix Plan: SD card init failure is never reported to the main node over LoRa

**Bug:** BUG-003
**Status:** Draft
**Created:** 2026-09-24

## Summary

When `sd.begin()` fails in `setup()`, the leaf node sets `status |= SD_CARD_INIT_ERROR`
but never calls `report_error_to_main_node()` for it — the dedicated LoRa error-report
path is structurally unreachable for this specific failure. This plan adds a single,
one-time report call at the one point in `setup()` where both "SD failed" and "LoRa is
up" are simultaneously known.

## Root cause

`log_data()` (`src/leaf_node.cc:384-419`) — called once per `loop()` iteration to
write the periodic sensor-data CSV — starts with:

```cpp
if (status & SD_CARD_INIT_ERROR) {
    return;
}
```

(lines 386-388), which returns before ever reaching the `SD_FILE_ENTRY_WRITE_ERROR`
branch (lines 406-414) that calls `report_error_to_main_node()`. That branch is only
reachable for a *write* failure on a card that initialized successfully — never for
the init failure itself, which is latched once in `setup()` at line 793
(`status |= SD_CARD_INIT_ERROR`) and deliberately never cleared (it's in the high
nybble of `status`, preserved across `loop()`'s `status = status & 0xF0` reset).

This early return is correct and should stay: `write_header()` and `log_data()` must
not attempt SD operations on a card that never initialized. The gap is that nothing
else ever reports the failure over LoRa.

The failure can't simply be reported at the point it's detected (`setup()`,
line 789-793) either: SD init runs *before* LoRa init in `setup()`
(`rf95_manager.init()` is at line 808), so the radio manager isn't ready yet at line
793. Calling `report_error_to_main_node()` there would call `sendtoWait()` on an
uninitialized `rf95_manager`. Confirmed by reading `setup()`'s ordering directly — no
further investigation needed; this is a narrow, localized ordering issue, not a
subsystem-level one.

## Related requirements

- FR-010 — "Leaf node reports hardware/SD-card errors to the main node over LoRa when
  they occur" (status `Proposed`). An SD init failure is exactly this kind of
  hardware error; today it's silently absorbed into the periodic status byte instead
  of producing the dedicated report FR-010 describes.

## Fix approach

Add one new call site in `setup()`, immediately after the existing LoRa
init/frequency/power/bandwidth/spreading-factor/coding-rate configuration block
completes (after line 838, before the next section of `setup()`):

```cpp
if (status & SD_CARD_INIT_ERROR) {
    report_error_to_main_node("SD_BEGIN_FAIL", MAIN_NODE_ADDRESS);
}
```

This is the first point after LoRa hardware setup where sending is possible, and it
runs exactly once (setup() runs once per boot), matching FR-010's "when they occur"
without repeating the same report every `loop()` cycle. No existing call site,
gating, or early-return behavior changes — `log_data()`/`write_header()` keep
returning early on `SD_CARD_INIT_ERROR` exactly as before; this only adds the missing
report, it doesn't touch the SD-write-avoidance logic that's already correct.

Narrow fix, not a broader one: this does not reorder SD/LoRa init in `setup()` (which
would be a larger, riskier change touching working code with no reported problem of
its own) and does not add a report for `RFM95_INIT_ERROR` (LoRa init/frequency
failure) — reporting a LoRa problem over LoRa is nonsensical, and those paths already
have their own handling (`error_blink`/`blink` + `debug_log()` where SD is confirmed
up, per the sd-debug-log plan).

## Regression risk

- Single new call site in `setup()`, which runs once per boot — no `loop()`-path or
  timing-sensitive code touched.
- `report_error_to_main_node()` itself is unchanged; it's already gated by
  `#if LORA && LORA_ERROR_REPORT` internally, so a build with `LORA_ERROR_REPORT=0`
  sees no behavior change at all (the call becomes a no-op, consistent with today).
- If LoRa *also* failed to init (`RFM95_INIT_ERROR` also set), `sendtoWait()` on a
  non-functional radio will simply fail and its return value is already ignored at
  every other call site of `report_error_to_main_node()` (e.g. `log_data()`,
  line 413) — this call follows the same existing pattern, not a new one.
- No existing test covers this path: `setup()` lives directly in the monolithic
  `src/leaf_node.cc` sketch (includes `Arduino.h`), not in `lib/` — the same
  architectural gap the separate `TASK-###` logic-extraction effort
  (`plans/task-leaf-node-logic-extraction-plan.md`) is tracking. A native unit test
  isn't practical here until that extraction happens; this fix relies on hardware
  verification instead, consistent with how BUG-001/BUG-002 in this same file are
  verified.

## Verification

1. Re-run the reproduction from `docs/bugs/BUG-LOG.md` BUG-003: disconnect/remove the
   SD card before power-on so `sd.begin()` fails, then observe the main node's radio
   traffic.
2. Confirm a dedicated `report_error_to_main_node()` message (e.g. `"SD_BEGIN_FAIL"`)
   now arrives at the main node once, at boot — in addition to (not instead of) the
   `0x20` status bit still showing up in the regular periodic data message.
3. Confirm the normal case (SD card present, `sd.begin()` succeeds) is unaffected: no
   report is sent, `status & SD_CARD_INIT_ERROR` is 0.
4. Confirm the combined-failure case (SD *and* LoRa both fail to init) doesn't hang or
   crash `setup()` — the report call should simply fail silently, same as any other
   `report_error_to_main_node()` call today.
5. `pio run -e zeroUSB` builds clean before/after.

## Out of scope

- Reordering SD/LoRa init in `setup()` — would remove the underlying ordering
  constraint entirely but is a larger, unrelated-risk change with no problem of its
  own driving it right now.
- Reporting `RFM95_INIT_ERROR`/`RFM95_SET_FREQ_FAIL` over LoRa — self-defeating by
  definition (the report channel itself is what failed); already handled by
  `blink()`/`error_blink()` + `debug_log()` where applicable.
- Extracting `setup()`/`loop()` logic out of `src/leaf_node.cc` into a testable,
  `Arduino.h`-free layer — tracked separately by the logic-extraction task, not this
  bugfix.
