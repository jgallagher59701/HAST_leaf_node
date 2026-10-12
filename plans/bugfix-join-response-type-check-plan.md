# Bugfix Plan: `parse_join_response` checks for the wrong `MessageType`

**Bug:** BUG-002
**Status:** Done, fix applied and verified 2026-10-11 (BUG-002's own status stays
`Fix Planned` until the user confirms it in practice)
**Created:** 2026-09-19

## Summary

`parse_join_response` (`lib/soil_sensor_common/messages.cc:151`) validates an
incoming `join_response_t` against `join_request` instead of `join_response`. The
fix is a one-line correction to compare against the right enum value.

## Root cause

Line 151 reads `if (data->type != join_request) return false;`. It should read
`if (data->type != join_response) return false;`. This is almost certainly a
copy-paste artifact from a sibling parse function — every other `parse_*` function
in this file (`parse_join_request`, `parse_time_request`, `parse_time_response`,
`parse_text_message`, `parse_data_message`) correctly checks its own message's type
constant; `parse_join_response` is the one exception.

## Related requirements

None. No `FR`/`NFR`/`UC`/`IC` documents the join/handshake process at all — see
`docs/bugs/BUG-LOG.md` (BUG-002) and `docs/deep-dives/soil-sensor-common.md`. This is
worth resolving separately via `/new-requirement` (formally document the join
protocol) or by deprecating it explicitly if it's not meant to be revived — not
bundled into this bugfix, since that's a scope decision for the user, not a
byproduct of a one-line fix.

## Fix approach

Change `messages.cc:151` from `data->type != join_request` to
`data->type != join_response`. Narrow, single-line fix — there's no broader design
gap here, just a wrong constant. No other function in this file has the same defect
(verified by re-reading every `parse_*` function during the deep-dive). **Applied by
the user directly** (not by this session's assistant work) on 2026-10-11.

**Scope grew by one more line to make the above actually testable.**
`messages.h:18` had `#undef SUPPORT_JOIN` unconditionally, before any `#ifdef` check
— added in the same commit that introduced the `SUPPORT_JOIN` switch
(`3209975`, "Add SUPPORT_JOIN compile-time switch"). That's almost certainly the same
class of authoring slip as BUG-002 itself: an unconditional `#undef` defeats its own
switch, since it wipes out any `-D SUPPORT_JOIN` build flag before the `#ifdef`s ever
see it — there is no way, with that line present, to compile the join code into
*anything*, by build flag or by `#define` before `#include`. Removed the line (not
replaced with a conditional default, since "undefined" is already the correct
default absent an explicit override). Confirmed with the user before making this
change, since the original plan scoped it out ("Out of scope" below, now superseded
for just this one line).

## Regression risk

Effectively none for shipped firmware: `SUPPORT_JOIN` is still not defined by either
`HAST_leaf_node/platformio.ini`'s `zeroUSB`/`debugZeroUSB` environments or (as far as
this repo can see) `HAST_lora_main`'s, so the `#ifdef SUPPORT_JOIN` blocks still don't
compile into either shipped firmware — confirmed by rebuilding both
`HAST_leaf_node` board environments clean after the `messages.h` change. The only
place `SUPPORT_JOIN` is now defined is `HAST_leaf_node`'s `[env:native]`
`build_flags`, added specifically so the new native test (below) can exercise this
code; that environment has no other consumer. `MessageType`'s other values are
explicit integer literals (not auto-incremented), so enabling `SUPPORT_JOIN` can't
renumber or collide with any message type the rest of the native test suite uses.

## Verification

1. **Done.** Re-ran reproduction step 3 from `docs/bugs/BUG-LOG.md` as an actual
   native test (`test/test_native_messages_join/test_messages_join.cc`), not just by
   inspection: `parse_join_response()` on a correctly-typed `join_response_t` (built
   by `build_join_response()`) returns `true` and populates `node`/`time`; called on
   a `join_request_t`-typed buffer reinterpreted as a `join_response_t *`, it returns
   `false` and leaves the out-params untouched. Also covers null out-params and a
   `parse_join_request()` round-trip (the sibling function, as a control case
   confirming the `SUPPORT_JOIN` wiring itself works independent of the bug).
2. `pio test -e native`: 29/29 pass (4 new + the 25 pre-existing from the
   node-time-slots/TASK-001 work). `pio run -e zeroUSB`/`-e debugZeroUSB`: both build
   clean. `pio check -e zeroUSB`: passes, no new findings.
3. **Incidental finding, left alone:** enabling `messages.cc` under `env:native` for
   the first time (it was never reachable there before) surfaced 12 pre-existing
   `-Wformat` warnings (`%lu` used with `uint32_t` arguments, which matches on the
   embedded target's ABI but not the 64-bit host's) across `messages.cc` and
   `data_packet.cc` — unrelated to this bug, confirmed with the user not to fix here.
   Worth its own `TASK-###` if the shared library is meant to be host-testable more
   broadly.

## Out of scope

- Deciding whether the join/handshake feature should be formally adopted (documented
  via `/new-requirement`) or removed as dead code — that's a separate decision the
  user should make, not something this one-line fix should force. (Making
  `SUPPORT_JOIN` *overridable* so it could be tested, above, isn't the same decision
  as adopting the feature — it's still off by default everywhere.)
- Fixing the 12 pre-existing `-Wformat` warnings noted above, or adding `HAST_lora_main`-side
  coverage for `parse_join_response` (its own `test/native/test_messages/
  join_messages_test.cc` doesn't exercise it either, per the original investigation) —
  both left for a separate `TASK-###` if wanted.
- Any change to `data_packet.h`/`data_packet.cc` (the legacy `packet_t` protocol).
  Confirmed during the `soil_sensor_common` deep-dive that `HAST_lora_main`'s
  `TFTDisplay.cc` still actively calls `parse_data_packet()` — that code is in use by
  the main node and is untouched by this fix.
