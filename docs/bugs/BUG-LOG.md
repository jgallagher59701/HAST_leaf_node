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
**Status:** Fixed
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

**Work Log**

**2026-10-11** — "BUG-002. But note that I made the one-line file on messages.cc line 151. You need to do the verification/tests part."
User applied the root-cause fix directly (`messages.cc:151`,
`data->type != join_request` -> `data->type != join_response`). Verification
needed real test coverage, not just inspection, which required fixing a second,
related issue: `messages.h:18`'s unconditional `#undef SUPPORT_JOIN` blocked
*any* way of enabling the join code, by build flag or local `#define` - the
same class of authoring slip as this bug itself. Confirmed with the user before
expanding scope to cover it; removed the line (default-undefined is already
correct) and added `-D SUPPORT_JOIN` to `HAST_leaf_node`'s `[env:native]` only
(not the shipped board environments) plus a new native test suite,
`test/test_native_messages_join/`, covering the exact reproduction-step-3
scenario plus a null-out-param case and a `parse_join_request` control case.
`pio test -e native` passes 29/29; `pio run -e zeroUSB`/`-e debugZeroUSB` and
`pio check -e zeroUSB` all stay clean. Incidentally surfaced 12 pre-existing
`-Wformat` warnings in `messages.cc`/`data_packet.cc` (first time that code has
ever compiled for the native host) - left alone, out of scope, flagged for a
possible future `TASK-###`. Status stays `Fix Planned`, not `Fixed` - only the
user confirms that.

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

## BUG-004 — Boot diagnostics after `USBDevice.detach()` are silently suppressed when `SERIAL_DIAG=1`

**Severity:** High
**Status:** Fix Planned
**Reported:** 2026-10-25
**Fix plan:** `plans/bugfix-serial-debug-after-usb-detach-plan.md`
**Related requirements:** FR-008 ("Leaf node supports a debug mode that can route
diagnostics over USB serial or over LoRa", status `Superseded by FR-009`) — this is
the exact USB-serial diagnostics mechanism (`SERIAL_DIAG` / the `IO()` macro in
`include/debug.h`) FR-008 describes. No requirement documents what should happen to
that mechanism once `USBDevice.detach()` runs, in either direction.

**Reproduction steps:**
1. Build `env:zeroUSB` (or `env:debugZeroUSB`) with the default
   `-D STANDBY_MODE=1` and `-D SERIAL_DIAG=1` (override the common
   `SERIAL_DIAG=0` flag in `platformio.ini`/`[common_env_data]`) to enable
   setup()-time USB serial diagnostics.
2. Flash the board and open a serial monitor during boot.
3. Observe `setup()` running in `src/leaf_node.cc`: diagnostics up through the LoRa
   init block (through line 915, "Initializing LORA... Done.") print normally.
4. At `src/leaf_node.cc:931`, `USBDevice.detach()` runs (guarded only by
   `#if STANDBY_MODE`, which is on by default) and the serial port goes away.
5. Immediately after, `setup()` sends the time request and still runs `IO()`-wrapped
   `Serial` calls at `src/leaf_node.cc:954-955` (unexpected time-response branch) and
   `:961` (`"Setup complete."`) — these compile in whenever `SERIAL_DIAG=1`,
   regardless of whether USB was just detached.

**Expected behavior:** When `SERIAL_DIAG=1`, all setup()-time diagnostic output
should reach the serial monitor, per FR-008's intent of routing diagnostics over USB
serial during debug mode.

**Actual behavior:** Diagnostics for the time-request/response exchange and the
final "Setup complete." message are silently dropped — the lines after
`USBDevice.detach()` write to a USB serial peripheral that's already been detached,
with no visible crash or hang on the board; they just vanish. This defeats the
specific purpose of turning `SERIAL_DIAG` on.

**Root cause:** `src/leaf_node.cc:926` guards `USBDevice.detach()` with `#if
STANDBY_MODE` only — the inline comment on that same line, `// !SERIAL_DIAG jhrg
6/17/23`, indicates the original intent was for the detach to also be skipped when
`SERIAL_DIAG` is on, but the actual preprocessor condition was never updated to say
`#if STANDBY_MODE && !SERIAL_DIAG`. Both shipped environments
(`env:zeroUSB`, `env:debugZeroUSB` in `platformio.ini`) default to `STANDBY_MODE=1`,
and `SERIAL_DIAG` is the flag developers flip to `1` specifically to get boot
diagnostics — so the one time a developer turns diagnostics on is exactly the time
`USBDevice.detach()` still fires and cuts them off mid-`setup()`. Confirmed by direct
inspection of `src/leaf_node.cc:926-961` and `include/debug.h`'s `IO()` macro
(compiles to a real `Serial` call whenever `SERIAL_DIAG` is non-zero, with no
awareness of USB attach state). This is a narrow, localized defect — no deep-dive
needed.

**Work Log**

**2026-10-25 00:00** — "During startup, when SERIAL_DEBUG is on, the call to
USBDevice.detach() turns off the serial port. However there is code that follows the
detach() call that uses the serial port when SERIAL_DEBUG is true."
Traced the report (user's "SERIAL_DEBUG" is actually `SERIAL_DIAG`) to
`src/leaf_node.cc:926`: `USBDevice.detach()` is guarded only by `#if STANDBY_MODE`,
while a stale trailing comment (`// !SERIAL_DIAG jhrg 6/17/23`) shows the condition
was originally meant to also exclude `SERIAL_DIAG`. Confirmed via code inspection
(no deep-dive needed — single, localized defect) that `IO()`-wrapped `Serial` calls
at lines 954-955 and 961 still compile in under `SERIAL_DIAG=1` and run after the
detach. User confirmed severity (High) and the observed symptom (some boot
diagnostics silently suppressed, no crash/hang) before logging. Related requirement:
FR-008 (superseded by FR-009 but still the live mechanism this bug affects). Wrote
fix plan to `plans/bugfix-serial-debug-after-usb-detach-plan.md`; status set to
`Fix Planned`.

---

<!-- Add new bugs via /fix-bug, or by hand — keep the section format: metadata lines,
     then Reproduction / Expected / Actual / Root Cause. A bug with no reproduction
     steps yet is a symptom report, not a bug entry — get concrete steps before
     assigning an ID if at all possible. -->
