# Bugfix Plan: `parse_join_response` checks for the wrong `MessageType`

**Bug:** BUG-002
**Status:** Draft
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
(verified by re-reading every `parse_*` function during the deep-dive).

## Regression risk

Effectively none: the entire `SUPPORT_JOIN` block, including this function, is
compiled out in both `HAST_leaf_node` and `HAST_lora_main` today (`messages.h:18`
`#undef SUPPORT_JOIN`, not redefined by either `platformio.ini`). The only code that
calls `parse_join_response` is inside the same `#ifdef SUPPORT_JOIN` guard, so no
currently-compiled firmware behavior changes. The main-node test
(`HAST_lora_main/test/native/test_messages/join_messages_test.cc`) exercises
`build_join_request`/`parse_join_request`/`join_request_to_string`, not
`parse_join_response`, so it doesn't currently cover this function either — and that
test still can't compile without `SUPPORT_JOIN` defined, independent of this fix.

## Verification

1. Re-run reproduction step 3 from `docs/bugs/BUG-LOG.md`: with the fix applied,
   `parse_join_response()` called on a correctly-typed `join_response_t` (as built by
   `build_join_response()`) should return `true` and populate `node`/`time`; called on
   a `join_request_t`-typed buffer, it should return `false`.
2. This can only be exercised by compiling with `SUPPORT_JOIN` defined (e.g. a
   temporary native build flag), since the function isn't reachable otherwise — add a
   native unit test for `parse_join_response` (new coverage; none exists today) if/when
   `SUPPORT_JOIN` is formally revived and documented. Until then, verification is by
   code inspection only, consistent with the fact that this path doesn't compile into
   either shipped firmware.

## Out of scope

- Deciding whether the join/handshake feature should be formally adopted (documented
  via `/new-requirement`) or removed as dead code — that's a separate decision the
  user should make, not something this one-line fix should force.
- Any change to `data_packet.h`/`data_packet.cc` (the legacy `packet_t` protocol).
  Confirmed during the `soil_sensor_common` deep-dive that `HAST_lora_main`'s
  `TFTDisplay.cc` still actively calls `parse_data_packet()` — that code is in use by
  the main node and is untouched by this fix.
