# Bugfix Plan: LoRa data/time messages sent as unacknowledged broadcasts instead of to the main node

**Bug:** BUG-001
**Status:** Draft
**Created:** 2026-09-19

## Summary

The leaf node sends its data messages and time requests to `RH_BROADCAST_ADDRESS`
instead of `MAIN_NODE_ADDRESS` (`0`), so RadioHead's reliable-datagram
acknowledgment/retry logic never engages. This fix addresses those sends to the main
node directly so FR-003's "reliable (acknowledged) datagram" behavior actually takes
effect.

## Root cause

`send_message()` (`src/leaf_node.cc:452-467`) and its three call sites — the initial
time request in `setup()`, the periodic time request in `loop()`, and the per-sample
data message in `loop()` — all pass `RH_BROADCAST_ADDRESS` as the destination.
`RHReliableDatagram::sendtoWait()` (vendored at
`.pio/libdeps/zeroUSB/RadioHead/RHReliableDatagram.cpp:75-77`) explicitly skips
acknowledgment and retry for broadcast addresses and returns `true` unconditionally
after a single send. The `if (to == RH_BROADCAST_ADDRESS) { waitPacketSent(...) }`
fallback in `send_message()` only confirms the packet left the radio, not that it was
received. This has been the design since the initial commit (`git log -S
RH_BROADCAST_ADDRESS -- src/leaf_node.cc`), not a recent regression. Full trace in
`docs/deep-dives/leaf-node-cc.md`.

## Related requirements

- FR-003 — "reliable (acknowledged) datagram" is violated: broadcast sends skip
  ACK/retry entirely, so nothing about the current transmission is actually reliable.

Not formally linked on the bug entry, but worth noting: NFR-005 (≥99% of
transmissions delivered) currently has no mechanism in the code that could measure or
enforce it either way — this fix makes drops *detectable* (via `status`), but does
not by itself add any counting/telemetry toward that target. See Out of scope.

## Fix approach

1. In `setup()`, change the initial time-request send to target `MAIN_NODE_ADDRESS`
   instead of `RH_BROADCAST_ADDRESS`.
2. In `loop()`, change the per-sample data-message send to target
   `MAIN_NODE_ADDRESS`.
3. In `loop()`, change the periodic time-request send (inside the
   `message % TIME_REQUEST_SAMPLE_PERIOD == 0` block) to target `MAIN_NODE_ADDRESS`.
4. No change needed to `send_message()`'s existing
   `if (!rf95_manager.sendtoWait(...)) status |= RFM95_SEND_ERROR;` path — once sends
   are addressed rather than broadcast, `sendtoWait()` will actually wait for an ACK
   and retry up to `setRetries(2)` times before failing, so this existing status-flag
   logic becomes meaningful for the first time.
5. Remove the `if (to == RH_BROADCAST_ADDRESS) { rf95_manager.waitPacketSent(...) }`
   block in `send_message()` — after step 1-3, nothing calls `send_message()` with a
   broadcast address, so this becomes dead code. Grep the file first to confirm no
   other call site relies on it before deleting.

**Why this scope:** `MAIN_NODE_ADDRESS` is already a known, fixed compile-time
constant, and the codebase already assumes a single, statically-addressed main node
everywhere else (the only mechanism that would need dynamic discovery — a
join/EUI-based handshake — is explicitly gated behind the disabled `SUPPORT_JOIN`
feature). Switching these three sends from broadcast to addressed is a narrow,
localized change that makes the existing reliable-datagram machinery behave as
FR-003 already describes it. Redesigning toward multi-main-node support or a
join/handshake protocol is unrelated to this bug and out of scope.

## Regression risk

- Addressed sends now genuinely retry (up to 2 retries at a 400 ms timeout each,
  i.e. up to ~1.2 s of retry time per send) before `sendtoWait()` reports failure —
  this adds up to a few hundred ms of worst-case duration to `setup()`/`loop()`
  compared to today's single-shot broadcast. Each leaf node already budgets roughly a
  5 s transmit window per IC-005's math; this should fit, but worth confirming
  against the main node's actual receive-window duration once that code is reviewed
  (main node lives in a separate repo, `HAST_lora_main`).
- `receive_message()` still doesn't validate the sender's address (a pre-existing,
  separate observation from the deep dive) — replies will now legitimately come from
  `MAIN_NODE_ADDRESS`, so this fix doesn't require touching that, but the gap remains
  open.
- No existing automated test covers this path (`docs/deep-dives/leaf-node-cc.md`
  notes there are no native tests for this file, and the on-air send path isn't
  practical to unit-test without hardware or a mock radio).
- Deleting the dead broadcast-branch in `send_message()` should be safe since this
  fix eliminates its only call pattern — confirm via grep that no other file in this
  repo calls `send_message()` with `RH_BROADCAST_ADDRESS` before removing it.

## Verification

- Static check: grep `src/leaf_node.cc` for `RH_BROADCAST_ADDRESS` and confirm no
  `send_message()` call site still uses it (reproduction steps 1-3 in BUG-001 should
  no longer apply).
- On-hardware test: flash a leaf node and a main node; during one sample cycle, take
  the main node's receiver out of range (or power it off); confirm the leaf node's
  `status` byte now picks up `RFM95_SEND_ERROR` after retries are exhausted — this
  was previously impossible to trigger, since broadcast sends always reported
  success regardless of whether anything received them.
- On-hardware test: with both nodes in normal range, confirm the main node still
  receives data messages and time responses correctly now that they're addressed
  rather than broadcast (verify the main node's own datagram-manager address matches
  `MAIN_NODE_ADDRESS`/`0` — that's in `HAST_lora_main`, not this repo).
- No existing automated test guards this path. If this class of bug recurs, consider
  a `TASK-###` to add a native test around `send_message()`'s status-flag logic
  against a mocked/faked RadioHead call.

## Out of scope

- Sender-address validation in `receive_message()` (the unchecked `from` parameter)
  — a separate, lower-severity observation from the deep dive, not part of this fix.
- Any telemetry/counting toward measuring NFR-005's ≥99% delivery target over time —
  this fix makes drops detectable per-transmission via `status`, but adds no logging
  or aggregation. A separate feature if NFR-005 needs active monitoring.
- Multi-main-node addressing or a join/handshake protocol (`SUPPORT_JOIN` is already
  disabled and unrelated to this bug).
- Collision avoidance / CAD arbitration across up to 25 simultaneously-waking leaf
  nodes (separate deep-dive observation tied to IC-005) — unrelated to this
  addressing bug and not touched by this fix.
