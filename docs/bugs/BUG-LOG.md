<!-- CONTRACT: deviations from intended behavior, tracked from report through root
     cause to fix. Not a duplicate of docs/requirements/ — a bug means something
     already documented (or implicitly assumed) isn't holding true in practice.
     Delete the BUG-EXAMPLE section once you've seen the shape. -->

# Bug Log

Convention: `BUG-###`, sequential, never renumbered. Status: `Open` → `Investigating`
→ `Fix Planned` → `Fixed` → (or `Won't Fix` / `Duplicate of BUG-0XX`). **Only the user
sets a bug to `Fixed`** — that's a claim about verified deployed behavior, not
something to self-report from having written a fix plan.

---

## BUG-001 — LoRa data/time messages sent as unacknowledged broadcasts instead of to the main node

**Severity:** Medium
**Status:** Fix Planned
**Reported:** 2026-09-19
**Related requirements:** FR-003 (violates the "reliable (acknowledged) datagram" part)
**Fix plan:** `plans/bugfix-lora-broadcast-address-plan.md`

**Reproduction steps:**
1. In `src/leaf_node.cc`, note `MAIN_NODE_ADDRESS` is defined as `0` (the main node's
   address), but every call to `send_message()` — the initial and periodic time
   requests, and the per-sample data message — passes `RH_BROADCAST_ADDRESS` as the
   destination instead.
2. In the vendored RadioHead library
   (`.pio/libdeps/zeroUSB/RadioHead/RHReliableDatagram.cpp:75-77`),
   `RHReliableDatagram::sendtoWait()` sends once and returns `true` unconditionally
   whenever `address == RH_BROADCAST_ADDRESS`, skipping the acknowledgment wait and
   retry loop entirely (`// Never wait for ACKS to broadcasts`).
3. Because of this, no LoRa transmission from the leaf node — whether or not it was
   actually received by the main node — will ever cause `sendtoWait()` to return
   `false`, so `send_message()` never sets `status |= RFM95_SEND_ERROR` for these
   sends, and `rf95_manager.setRetries(2)` has no effect on them.

**Expected behavior:** Per FR-003, the leaf node's readings should be delivered to
the main node using a reliable, acknowledged datagram — addressed to
`MAIN_NODE_ADDRESS`, retried on a missing ACK, with a detectable failure status when
delivery ultimately fails.

**Actual behavior:** Readings and time requests are sent as LoRa broadcasts: the
transmission is attempted exactly once, with no ACK wait and no retry, and the leaf
node has no way to detect a dropped transmission via `status`.

**Root cause:** `send_message()` (`src/leaf_node.cc:452-467`) and its callers in
`setup()` and `loop()` pass `RH_BROADCAST_ADDRESS` as the destination for both the
time-request and data-message sends, instead of the already-known `MAIN_NODE_ADDRESS`
constant. `RHReliableDatagram::sendtoWait()` explicitly special-cases broadcast
addresses to skip acknowledgment and retry (see reproduction step 2) — this is
documented, intentional RadioHead library behavior, not a library bug. The extra
`if (to == RH_BROADCAST_ADDRESS) { rf95_manager.waitPacketSent(...) }` block in
`send_message()` only waits for the packet to leave the radio, which is not the same
as confirming it was received. Confirmed via
`git log -S RH_BROADCAST_ADDRESS -- src/leaf_node.cc` that broadcast addressing has
been present since the initial commit — long-standing design, not a recent
regression. Full investigation: `docs/deep-dives/leaf-node-cc.md`.

---

<!-- Add new bugs via /fix-bug, or by hand — keep the section format: metadata lines,
     then Reproduction / Expected / Actual / Root Cause. A bug with no reproduction
     steps yet is a symptom report, not a bug entry — get concrete steps before
     assigning an ID if at all possible. -->
