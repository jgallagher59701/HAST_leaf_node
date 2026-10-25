# Bugfix Plan: Boot diagnostics after `USBDevice.detach()` are silently suppressed when `SERIAL_DIAG=1`

**Bug:** BUG-004
**Status:** Draft
**Created:** 2026-10-25

## Summary

`setup()` in `src/leaf_node.cc` detaches the USB serial device whenever
`STANDBY_MODE=1` (the default), without regard for whether `SERIAL_DIAG=1` is also
set. Because several `IO(Serial...)` diagnostic calls run later in the same `setup()`
call, turning diagnostics on (`SERIAL_DIAG=1`) is exactly what causes some of those
diagnostics to be silently dropped. The fix makes the detach conditional on
`SERIAL_DIAG` being off, matching the stale comment that already documents this as
the original intent.

## Root cause

Code was added to setup() after the call to detach(), but detach() should be the 
laster things called before loop().

## Related requirements

- FR-008 ("Leaf node supports a debug mode that can route diagnostics over USB serial
  or over LoRa", status `Superseded by FR-009`) — this is the exact mechanism FR-008
  describes. FR-008 is superseded, not deprecated/removed, and the code (`SERIAL_DIAG`,
  `IO()`) is still live, so it's still a legitimate thing to fix rather than delete.
  No requirement specifies the detach/diagnostics interaction directly — that gap is
  being closed by this fix, not by a new requirement, since it's a straightforward
  correction to match already-documented intent (the stale comment), not a new policy
  decision.

## Fix approach

Narrow, two-part fix scoped to the exact interaction that's broken:

1. Move the new code before the call to detach().

## Regression risk

- `USBDevice.detach()` at line 931 is the only call site affected. Nothing else in
  `src/leaf_node.cc` depends on USB being detached immediately at that point in
  `setup()` — the surrounding code (time-request send/receive, "Setup complete." log)
  only *writes* to `Serial`, it doesn't branch on USB attach state.
- Behavior changes only when `SERIAL_DIAG=1` (not the default in either shipped
  environment — `platformio.ini`'s `[common_env_data]` sets `SERIAL_DIAG=0`). With
  `SERIAL_DIAG=0` (the normal, shipped configuration), `USB_DETACH_AFTER_SETUP`
  evaluates identically to the current `STANDBY_MODE` check, so production firmware
  behavior is unchanged.
- With `SERIAL_DIAG=1`, USB now stays attached through the rest of `setup()` instead
  of detaching early. This trades a small amount of power draw during `setup()` for
  working diagnostics — acceptable since `SERIAL_DIAG=1` is a debug-only
  configuration, never shipped.
- `src/leaf_node.cc` is excluded from `env:native`
  (`platformio.ini`'s `build_src_filter`, since it includes `Arduino.h`), so this
  change can't be exercised by the existing native leaf-node-logic tests — those cover
  `leaf_node_logic.cc`, not `leaf_node.cc`'s `setup()`/`loop()` directly.

## Verification

1. **New native test** (regression guard for the actual logic bug): add
   `test/test_native_debug_macros/` that includes `include/debug.h` under each of the
   four `(STANDBY_MODE, SERIAL_DIAG)` combinations and asserts
   `USB_DETACH_AFTER_SETUP` evaluates to `1` only when `STANDBY_MODE=1 &&
   SERIAL_DIAG=0`. This is the part of the bug that's actually unit-testable —
   `debug.h` has no `Arduino.h` dependency, unlike `leaf_node.cc` itself. Run via
   `pio test -e native`.
2. **Hardware verification** (re-run BUG-004's reproduction steps, since the
   `setup()` interaction itself isn't natively testable):
   - Build and flash `env:zeroUSB` with `SERIAL_DIAG=1` (override the default) and the
     default `STANDBY_MODE=1`.
   - Open a serial monitor during boot.
   - Confirm diagnostic output continues past the former detach point: the
     time-request/response branch's output (or `"Unexpected response type: ..."` if no
     main node is present to respond) and the final `"Setup complete."` line now
     appear, where they previously did not.
   - Build and flash with `SERIAL_DIAG=0` (the shipped default) and confirm no change
     in behavior (USB still detaches at the same point, since
     `USB_DETACH_AFTER_SETUP` reduces to the old condition).
3. `pio check -e zeroUSB` and `pio check -e debugZeroUSB` stay clean.

## Out of scope

- The pre-existing, separately-documented limitation that `SerialUSB` doesn't work
  after the board wakes from `rtc.standbyMode()` (comment at
  `src/leaf_node.cc:929-930`) — this affects diagnostics *after* the first sleep/wake
  cycle in `loop()`, not the `setup()`-time diagnostics this bug is about. Worth its
  own bug/task if diagnostics during `loop()` are ever needed.
- Any change to `rtc.standbyMode()` gating (lines 468, 726) to also respect
  `SERIAL_DIAG` — these aren't part of the reported symptom (diagnostics suppressed
  during `setup()`) and changing them would affect real sleep/power behavior, a larger
  change than this bug warrants.
- Reclassifying or updating FR-008's `Superseded by FR-009` status — that's a
  requirements-doc decision independent of this code fix.
