# Bugfix Plan: Boot diagnostics after `USBDevice.detach()` are silently suppressed when `SERIAL_DIAG=1`

**Bug:** BUG-004
**Status:** Done
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

Reorder rather than re-gate: `USBDevice.detach()` (and the `BOOT_SAFETY_DELAY` yield
before it) is moved to the very end of `setup()`, after every remaining `IO(Serial...)`
diagnostic call — including a new one added at the same time, printing the RTC
date/time once it's updated from the main node's time response. The `#if
STANDBY_MODE` guard on the detach itself is unchanged: USB still always detaches when
`STANDBY_MODE=1`, regardless of `SERIAL_DIAG`. What changes is that detach now happens
*after* all diagnostics have had a chance to print, not before. This satisfies both
goals at once — standby/power behavior is untouched, and `SERIAL_DIAG=1` diagnostics
are no longer cut off — without introducing a new macro or conditional.

The one leftover rough edge: the inline comment on the `#if STANDBY_MODE` line
(`// !SERIAL_DIAG jhrg 6/17/23`) still implies the detach is conditioned on
`SERIAL_DIAG`, which was true of the *intent* but still isn't true of the *code* —
the guard is still `STANDBY_MODE` alone. Left as-is since it's cosmetic (the actual
bug — diagnostics after detach — is fixed by the reordering) and not part of what was
asked; worth a follow-up comment cleanup if it causes confusion again.

## Regression risk

- The only code moved is the `BOOT_SAFETY_DELAY` yield and the `#if STANDBY_MODE`
  `USBDevice.detach()` block, now running after the time-request/response exchange
  and the final `"Setup complete."` log instead of before them. Nothing between the
  old and new detach location branches on USB attach state, so moving it doesn't skip
  or duplicate any logic — confirmed by reading the intervening code (time
  request/response handling, status bit, loop() start).
- Behavior is unchanged in the shipped configuration (`SERIAL_DIAG=0`,
  `STANDBY_MODE=1`, both shipped environments' default): the `IO()` macro
  (`include/debug.h`) compiles to nothing when `SERIAL_DIAG=0`, so moving `IO()` call
  sites around has no runtime effect, and `USBDevice.detach()` still fires
  unconditionally under `STANDBY_MODE=1` — just a few lines/one LoRa round-trip later
  in `setup()`. Verified via `pio run -e zeroUSB` (default flags): builds clean,
  unchanged RAM/flash usage.
- With `SERIAL_DIAG=1` (debug-only, never shipped), the detach now happens slightly
  later in wall-clock terms — after the time-request LoRa round trip — rather than
  before it. That round trip already happens whether or not USB is attached, so this
  doesn't change LoRa timing or behavior, only when the USB detach relative to it
  occurs.
- `src/leaf_node.cc` is excluded from `env:native` (`platformio.ini`'s
  `build_src_filter`, since it includes `Arduino.h`), so this reorder can't be and
  isn't exercised by the existing native test suites (`test_native_leaf_node_logic`,
  `test_native_messages_join`, `test_native_slot_schedule`) — all three cover other
  files/logic and are unaffected, confirmed by running them (see Verification).

## Verification

1. `pio test -e native` — all 29 existing test cases pass unchanged; none of them
   touch `leaf_node.cc`'s `setup()`/`loop()`, so this is a no-regression check on
   unrelated logic, not a test of this fix itself.
2. `pio run -e zeroUSB` and `pio run -e debugZeroUSB` — both build clean with the
   shipped default flags (`SERIAL_DIAG=0`). RAM/flash usage unchanged
   (6752 B / 64856 B on zeroUSB) from before the fix.
3. `pio run -e zeroUSB -e debugZeroUSB` rebuilt with `SERIAL_DIAG=1` (temporarily
   overriding `platformio.ini`'s default to exercise the previously-broken path) —
   both build clean (flash usage grows by ~1 KB, matching the new RTC-print `IO()`
   block compiling in).
4. `pio check -e zeroUSB` under `SERIAL_DIAG=1` — passes with 0 high-severity
   findings; nothing new on the lines touched by this fix (pre-existing
   style/C-style-cast findings elsewhere in the file are unrelated and unchanged).
5. **Hardware verification — done.** User confirmed on an actual board that
   `setup()`-time diagnostics (time-request/response branch, `"Setup complete."`)
   now visibly print over USB serial under `SERIAL_DIAG=1` before `USBDevice.detach()`
   runs.

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
