# Deep dive: `lib/soil_sensor_common`

**Date:** 2026-09-19
**Question:** Investigate `lib/soil_sensor_common` — structure, usage, and how it
holds up against the requirement docs.

## Structure

A git submodule (own `.git`, pinned to commit `ed09d76fae9bdb0f1bd0379dc5d0563f66110e9b`
in both this repo and `HAST_lora_main`, so the two are currently byte-identical — no
drift between leaf and main node copies). No `Arduino.h` dependency (`#include
<Arduino.h>` is commented out in both headers) — pure `cstdint`/`cstdio`/`cstring`,
consistent with `CLAUDE.md`'s hardware-adapter-layer convention.

Two parallel protocols live here:

- **`data_packet.h`/`.cc`** — a 21-byte `packet_t` struct (message#, time, battery,
  last-tx-duration, temp, humidity, status, node), no type tag, with
  `build_data_packet`/`parse_data_packet`/`data_packet_to_string`.
- **`messages.h`/`.cc`** — a tagged-union-style protocol: `MessageType` enum
  (`time_request=3`, `time_response=4`, `data_message=10`, `text=11`,
  `data_packet=12`, `error=255`) plus per-type structs (`time_request_t`,
  `time_response_t`, `text_t`, `data_message_t`) each with their own
  build/parse/to-string trio. `data_message_t` duplicates `packet_t`'s payload
  fields but adds `type` and reorders them.
- **Join handshake** (`join_request_t`/`join_response_t`, EUI-based node
  registration) exists in `messages.h`/`.cc` but is compiled out unconditionally —
  `messages.h:18` does `#undef SUPPORT_JOIN` and neither repo's `platformio.ini`
  redefines it.

## Usage

**HAST_leaf_node** (`src/leaf_node.cc` only): uses `build_time_request`/
`parse_time_response`/`MessageType` for clock sync (initial + periodic
re-sync every `TIME_REQUEST_SAMPLE_PERIOD` samples), and `build_data_message`/
`data_message_to_string` for the per-sample sensor reading sent over LoRa and
logged to SD. `packet_t` appears only in a stale comment (`// packet_t data;`,
line 851) — the older `data_packet.h` API (`build_data_packet`, `parse_data_packet`,
`data_packet_to_string`) and `parse_data_message`/`build_time_response`/
`parse_time_request` are unused in this repo. No native unit test exercises this
library here — `test/` has only an empty `test/m_0` directory, contrary to
`CLAUDE.md`'s "new logic-layer code isn't done without a native test" convention
(though this library predates that convention; the gap is in leaf-node-side test
coverage, not the library itself).

**HAST_lora_main** (main node firmware, in the other working directory):
`src/main-node.cc` dispatches on `get_message_type()` and handles `data_message`
(via `data_message_to_string`), `time_request` (via `parse_time_request` /
`build_time_response`), and an empty, `#if SUPPORT_JOIN`-guarded `join_request`
case. `include/TFTDisplay.h`/`src/TFTDisplay.cc` still actively support **both**
the legacy `packet_t` path (`parse_data_packet`) and the current `data_message_t`
path (`parse_data_message`), plus `parse_time_request`.

## Findings

### `parse_join_response` checks the wrong message type

`messages.cc:151` — `parse_join_response` checks `data->type != join_request`; it
should almost certainly check `join_response`. This is currently unreachable (the
whole join feature is compiled out), but would misbehave immediately if
`SUPPORT_JOIN` were ever re-enabled without this being caught.

### Join feature has no requirement backing it, in either direction

No `FR`/`NFR`/`UC`/`IC` describes a join/handshake process, yet `join_request_t`/
`join_response_t`, the disabled `SUPPORT_JOIN` code paths, and a main-node test
(`HAST_lora_main/test/native/test_messages/join_messages_test.cc`, which calls
`build_join_request`/`parse_join_request`/`join_request_to_string` — all inside
`#ifdef SUPPORT_JOIN` blocks) all exist. Since `SUPPORT_JOIN` is unconditionally
`#undef`'d and never redefined by either `platformio.ini`, **that main-node test
file currently cannot compile.** This reads as a half-migrated feature: either it
was intended and never documented, or it's dead code left over from before static
node addressing (`FR-007`) was adopted, and should be formally deprecated rather
than silently carried forward.

### `data_packet` (the packet_t protocol) is legacy, not documented as such

`packet_t`/`data_packet.h` looks like an earlier wire format that `data_message_t`
superseded (same fields, plus a type tag and reordering). The main node's
`TFTDisplay.cc` still supports both paths; the leaf node only ever builds
`data_message_t`. `MessageType::data_packet` (value 12) exists in the enum but
`get_message_type_string`/`is_valid_message_type` don't handle it — falls through
to `"unknown"`/`false`. No requirement or decision doc marks `packet_t` as
superseded (`ADR-###` would be the natural place per `CLAUDE.md`'s ID conventions).

### No requirement covers the LoRa payload size constraint

Nothing in `docs/requirements/` or `docs/constraints/` references
`RH_RF95_MAX_MESSAGE_LEN` (251 bytes, `messages.h:15`) or bounds message/struct
size against it, despite `text_t`'s `TEXT_BUF_LEN` being explicitly sized off that
constant. `data_message_t`/`packet_t` are well under the limit today, but there's
no documented guardrail if fields are added later.

## Requirements checked against

- **FR-003** (reliable/acknowledged datagram transmission) — matches the
  `data_message_t`/`send_message()` flow.
- **FR-005** (clock sync via time request/response) — matches
  `build_time_request`/`parse_time_response`.
- **FR-006** (battery voltage reported alongside sensor data) — matches
  `data_message_t.battery`.
- **FR-007** (unique, compile-time node address) — matches `NODE_ADDRESS` passed
  into `build_data_message`/`build_time_request`.
- **IC-005** (bounds node count) — related to node addressing but doesn't touch
  message/payload size.
