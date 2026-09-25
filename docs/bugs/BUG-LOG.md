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

## BUG-002 — `parse_join_response` checks for the wrong `MessageType`

**Severity:** Low
**Status:** Fix Planned
**Reported:** 2026-09-19
**Related requirements:** None found. No `FR`/`NFR`/`UC`/`IC` documents a join/handshake
process at all — the entire `SUPPORT_JOIN` feature (join request/response messages,
node registration by EUI) exists in `lib/soil_sensor_common` with no requirement ever
having been written for it, in either direction (not documented as planned, not
documented as deprecated).

**Reproduction steps:**
1. In `lib/soil_sensor_common/messages.cc:150-160`, `parse_join_response` is defined
   to unpack a `join_response_t` message.
2. Its guard on line 151 reads `if (data->type != join_request) return false;` — it
   checks the message against `join_request`, not `join_response`.
3. Call `parse_join_response()` with a `join_response_t` whose `type` field is
   correctly set to `join_response` (as `build_join_response()`, messages.cc:143-147,
   sets it): the guard's condition (`join_response != join_request`) is true, so the
   function returns `false` and refuses to parse a validly-typed join response.
   Conversely, a buffer mistakenly holding a `join_request_t` (type `join_request`)
   would pass this check and be misparsed as a join response.
4. This code path is currently unreachable in both `HAST_leaf_node` and
   `HAST_lora_main`: `messages.h:18` does `#undef SUPPORT_JOIN` unconditionally, and
   neither repo's `platformio.ini` redefines it, so none of the `#ifdef SUPPORT_JOIN`
   code compiles into either firmware today. It does, however, block
   `HAST_lora_main/test/native/test_messages/join_messages_test.cc` from compiling
   whenever that guard is lifted, since that test exercises the join API.

**Expected behavior:** `parse_join_response` should validate that the message it's
given is actually a `join_response_t` by checking `data->type != join_response`, and
return `true`/populate its out-parameters only for a correctly-typed join response.

**Actual behavior:** It checks against `join_request` instead, so it rejects valid
join responses and would silently accept a mistyped buffer holding a join request.

**Root cause:** Copy-paste from a sibling parse function (e.g. `parse_join_request`
or `parse_time_request`, which correctly check their own type) without updating the
enum value being compared against. Confirmed by direct code inspection
(`lib/soil_sensor_common/messages.cc:150-160`); no further investigation needed — this
is a narrow, single-line defect, not a subsystem-level issue. Full context on the
surrounding library: `docs/deep-dives/soil-sensor-common.md`.

**Fix plan:** `plans/bugfix-join-response-type-check-plan.md`

---

## BUG-003 — SD card init failure is never reported to the main node over LoRa

**Severity:** Low
**Status:** Fix Planned
**Reported:** 2026-09-24
**Fix plan:** `plans/bugfix-sd-init-error-report-plan.md`
**Related requirements:** FR-010 ("Leaf node reports hardware/SD-card errors to the
main node over LoRa when they occur", status `Proposed`) — an SD init failure is
exactly this kind of hardware error, yet no dedicated LoRa report is ever sent for it.

**Reproduction steps:**
1. Cause `sd.begin(SD_CS)` to fail in `setup()` (e.g. disconnect/remove the SD card
   before power-on) — `src/leaf_node.cc:789-793` then runs `blink(STATUS_LED,
   SD_BEGIN_FAIL, ERROR_TIMES)` and sets `status |= SD_CARD_INIT_ERROR` (`0x20`).
2. Let the node continue running normally through `loop()`.
3. Observe the main node's radio traffic from this leaf node over several cycles.

**Expected behavior:** Per FR-010, a hardware/SD-card error like this should produce
a dedicated LoRa error-report message to the main node (via
`report_error_to_main_node()`).

**Actual behavior:** No dedicated error-report message is ever sent. The `0x20` bit
does show up in the main node's regular periodic data message (`build_data_message()`
→ `send_message()` in `loop()`, unconditional every cycle), but that's the ordinary
telemetry channel, not FR-010's error-report path — confirmed via hardware testing by
the user (2026-09-24): status byte arrives correctly, but the message
`report_error_to_main_node()` would send never shows up.

**Root cause:** `log_data()` (`src/leaf_node.cc:384-419`), called once per `loop()`
iteration, returns immediately at line 386-388 whenever `status & SD_CARD_INIT_ERROR`
is set — before ever reaching the `SD_FILE_ENTRY_WRITE_ERROR` branch (lines 406-414)
that calls `report_error_to_main_node()`. That branch is therefore only reachable for
a *write* failure on a card that initialized successfully (e.g. card removed
mid-run, card full) — never for the init failure itself, which is set once in
`setup()` at line 793.
Moving the report call to the SD-init-failure site itself isn't a drop-in fix: at
that point in `setup()` (line 789-793), `rf95_manager.init()` hasn't run yet — LoRa
init happens later, at line 808 — so the radio isn't up yet and
`report_error_to_main_node()` would call `sendtoWait()` on an uninitialized radio
manager. There's no existing point in `setup()`, before this bug, where both "SD
failed" and "LoRa is up" are simultaneously known.

---

<!-- Add new bugs via /fix-bug, or by hand — keep the section format: metadata lines,
     then Reproduction / Expected / Actual / Root Cause. A bug with no reproduction
     steps yet is a symptom report, not a bug entry — get concrete steps before
     assigning an ID if at all possible. -->
