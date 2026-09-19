# Deep dive: `src/leaf_node.cc`

**Date:** 2026-09-18
**Question:** Investigate `src/leaf_node.cc` — how it's structured, what it does, and
how it holds up against the requirement docs.

## Structure

Single-file Arduino sketch, includes `Arduino.h` directly and mixes hardware access
(`digitalWrite`, `SPI`, `Serial`, RadioHead, SdFat, RTCZero) with the sequencing logic
in one place — there is no separate hardware-adapter layer for this file, unlike what
`CLAUDE.md`'s conventions ask for going forward. Depends on `lib/soil_sensor_common`
(`data_message_t`, `time_request_t`/`time_response_t`, build/parse/to-string helpers),
`include/blink.h`, `include/debug.h`, `include/get_battery_voltage.h`.

`setup()`: initializes pins (blanket `INPUT_PULLUP` sweep, then overrides for the
pins actually used), the RTC (seeded from `__DATE__`/`__TIME__` at compile time), the
SHT30D sensor, the SD card (creates/opens the next unused `DataNN.csv`, writes a
header), and the RF95 radio (frequency/bandwidth/spreading factor/coding
rate/retries/timeout), then sends an initial time request and applies the response
before falling into `loop()`.

`loop()`: reads temperature, humidity, and battery voltage; builds a `data_message_t`;
transmits it over LoRa; logs it to the SD card; every `TIME_REQUEST_SAMPLE_PERIOD`
samples, re-syncs the clock from the main node; then calls `sleep_node()`, which
powers down the radio and SD card and puts the MCU into RTC-driven standby until the
next scheduled wake (top-of-hour by default, or once-per-minute in test mode).

## Finding: data transmissions and time requests are sent as unacknowledged broadcasts

Every `send_message()` call in this file (both `time_request_t` in `setup()`/`loop()`
and the `data_message_t` in `loop()`) targets `RH_BROADCAST_ADDRESS`, not
`MAIN_NODE_ADDRESS` (which is a known constant, `0`).

Checked against the vendored library
(`.pio/libdeps/zeroUSB/RadioHead/RHReliableDatagram.cpp:75-77`):

```cpp
// Never wait for ACKS to broadcasts:
if (address == RH_BROADCAST_ADDRESS)
    return true;
```

`RHReliableDatagram::sendtoWait()` sends once and returns `true` unconditionally for a
broadcast address — no ACK wait, no retry, regardless of `setRetries()`. Confirmed via
`git log -S RH_BROADCAST_ADDRESS` that this has been the design since the initial
commit, not a recent regression.

**This appears to contradict:**
- **FR-003** ("Leaf node transmits each reading to the main node over LoRa using a
  reliable (acknowledged) datagram") — the transmission is neither acknowledged nor
  retried in practice, despite using `RHReliableDatagram` and configuring retries.
- **NFR-005** (≥99% of LoRa transmissions delivered) — there is currently no mechanism
  in this code that could detect or count a dropped data message; `status` will never
  pick up `RFM95_SEND_ERROR` for a broadcast send, since `sendtoWait()` can't fail for
  one.

This is an observation to verify, not a settled bug report — it's possible broadcast
was chosen deliberately (e.g., to avoid hardcoding the main node's address, or because
addressed sends were tried and abandoned for some reason not visible in this file) and
the requirement docs are the side that needs adjusting instead. Recommend
`/fix-bug` if the intent was reliable, addressed delivery; `/new-requirement` or an
edit to FR-003/NFR-005 if broadcast-and-hope is the accepted design.

## Other observations

- **`receive_message()` ignores the sender.** It captures `from` via
  `recvfromAck(rf95_buf, &len, &from)` but never checks it against
  `MAIN_NODE_ADDRESS`. A reply from any address would currently be accepted as a
  valid time response. Low practical risk with a single main node, but worth
  knowing if that topology ever changes.

- **Diagnostic code paths are compiled out entirely in production builds.**
  `DEBUG` and `LORA_DEBUG` are hardcoded to `0` at the top of the file (not exposed
  via `platformio.ini` build flags the way `NODE_ADDRESS`/`FREQUENCY`/etc. are), so
  the `IO(...)`-wrapped code and `LORA_DEBUG` blocks never build under normal
  conditions. One `IO(...)` invocation (setup(), the RTC-echo block) spans ~10 lines
  and multiple statements as a single macro argument — it's correct as written, but
  fragile to edit. Since none of this compiles under `DEBUG=0`, regressions in it
  (e.g., the "address vs. value" bug a recent commit fixed) can go unnoticed for a
  long time.

- **No native unit test coverage.** `env:native`'s `test_filter = native_*` has
  nothing to match — `test/m_0` is empty (`save/test/native_0` has one test file for
  unrelated `standby_time` code, not wired into the active `test/` tree). Consistent
  with this file not separating Arduino-dependent code from testable logic.

- **Fleet-wide wake time, no visible collision-avoidance call.** `WAKE_UP_MINUTE` /
  `WAKE_UP_SECOND` come from shared `platformio.ini` build flags — every leaf node
  built from the same config wakes at the same instant. IC-005's 25-node budget
  (25 × 5 s ≤ ~120 s main-node awake time) assumes each node's transmit window is
  effectively serialized/arbitrated, but this file has no explicit `waitCAD()` call
  or per-node time-division scheduling; `rf95.setCADTimeout(...)` only sets a timeout
  value for channel-activity detection, it doesn't by itself invoke CAD before a send
  in this code. Worth checking whether collision avoidance lives in the main node,
  in RadioHead's driver internals, or isn't handled yet.

- **`platformio.ini.bak`** at the repo root is a stale leftover targeting a
  completely different board (`espressif8266`/`nodemcuv2`) — unrelated to this file,
  noted only because it was seen in passing while checking build-flag history.
