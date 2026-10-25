# HAST_leaf_node

Code for the HAST leaf node — the node uses LoRa to send data to a main node.

## What it does

Each leaf node is a battery-powered field sensor built around a RocketScream
Mini Ultra Pro V3 (SAMD21 / ARM M0). Once per hour (or once per minute in test
builds) the node wakes from standby, then:

1. Reads temperature and humidity from an SHT31-D sensor.
2. Appends the reading to a CSV log file on its SD card.
3. Sends the same reading to a main node over LoRa, in an assigned time slot
   so multiple leaf nodes sharing a channel don't collide.
4. Periodically asks the main node for the current time and resyncs its RTC
   if its own clock has drifted.
5. Goes back into low-power standby until its next scheduled wake time.

Each leaf node is identified to the main node by a one-byte node number, which
is included in every message it sends. The main node uses that number to tell
one leaf node's readings apart from another's.

## Loading the code

This is a PlatformIO project. Open it in PlatformIO (CLI or the VS Code/CLion
extension) and build/upload the `zeroUSB` environment (`debugZeroUSB` for a
JLink debug build) to a RocketScream Mini Ultra Pro V3.

### Setting the node number

**Every physical node must be built with a unique node number before it is
deployed.** There is currently no runtime configuration mode — the node number
is a compile-time constant, `NODE_ADDRESS`, set in `platformio.ini`:

```ini
[common_env_data]
build_flags =
    ...
    -D NODE_ADDRESS=11
    ...
```

Before flashing a node, edit this value to the number assigned to that node,
then build and upload. Rules:

- Valid range is 1–15 (`IC-005`); 0 is reserved for the main node
  (`MAIN_NODE_ADDRESS`). A build fails at compile time (`static_assert` in
  `src/leaf_node.cc`) if `NODE_ADDRESS` is out of range or if its transmit
  slot doesn't fit within the hour.
- Node numbers below 3 are reserved for the two hand-built units, which wire
  SD card power to a different pin (11) than the PCB-based units (9) —
  `src/leaf_node.cc` picks the correct pin automatically based on
  `NODE_ADDRESS`, but only if the number correctly identifies which board it
  is.
- The node's LoRa transmit slot (which minute/second within the hour it's
  allowed to send) is derived from `NODE_ADDRESS` together with
  `SLOT_SCHEDULE` and `GROUPED_START_S` — see `include/slot_schedule.h`.
  Changing `NODE_ADDRESS` changes when that node transmits.

Other build-time settings worth knowing about in `platformio.ini`:

| Flag | Purpose |
|---|---|
| `NODE_ADDRESS` | This node's one-byte identifier (see above) |
| `FREQUENCY` | LoRa center frequency in MHz |
| `STANDBY_MODE` | 1 = sleep between samples (normal/deployed use), 0 = busy-wait |
| `SAMPLE_ONCE_PER_MINUTE` / `SAMPLE_SECOND` | Test mode: sample once a minute instead of once an hour |
| `SERIAL_DIAG` | 1 = emit setup()-time diagnostics over USB serial |
| `DEBUG_LOG` | 1 = write loop()-time diagnostics to the SD card debug log |
| `LORA_ERROR_REPORT` | 1 = also report hardware/SD errors to the main node over LoRa (requires a main node that handles them) |

## Error codes

### Boot-time errors (LED blink codes)

If a hardware component fails to initialize during `setup()`, the node
flashes the onboard status LED (pin `STATUS_LED`) to signal which component
failed. Each blink cycle is one long flash (1s) followed by *N* short
flashes, where *N* is the error code below:

| Code | Meaning |
|---|---|
| 2 | `SHT31_BEGIN_FAIL` — temperature/humidity sensor failed to initialize |
| 3 | `SD_BEGIN_FAIL` — SD card failed to initialize |
| 4 | `SD_WRITE_HEADER_FAIL` — could not write the CSV header to the log file |
| 5 | `RFM95_INIT_FAIL` — LoRa radio failed to initialize |
| 6 | `RFM95_SET_FREQ_FAIL` — LoRa radio initialized but could not be set to `FREQUENCY` |
| 7 | `SD_LOG_FILE_NAME_FAIL` — could not find an unused log file name on the SD card |

Most of these repeat the blink cycle 3 times (`ERROR_TIMES`) and then let boot
continue in a degraded state, since a dead sensor or disconnected SD card
shouldn't by itself stop the node from at least trying to report data. The
exception is `RFM95_INIT_FAIL`, which blinks forever — without a working
radio, the node has no way to report anything, so it halts there rather than
continue.

### Runtime status byte

Every reading the node sends to the main node also carries a one-byte status
field summarizing the node's health. It's a bitmask, so more than one
condition can be set at once:

| Bit | Meaning | Persists across cycles? |
|---|---|---|
| `0x01` | `RFM95_SEND_ERROR` — LoRa transmit failed | No — cleared each loop |
| `0x02` | `RFM95_NO_REPLY` — main node didn't acknowledge | No — cleared each loop |
| `0x04` | `SD_FILE_ENTRY_WRITE_ERROR` — couldn't append this reading to the SD log | No — cleared each loop |
| `0x08` | `SD_CARD_WAKEUP_ERROR` — SD card didn't come back after waking from standby | No — cleared each loop |
| `0x10` | `SD_NO_MORE_NAMES` — ran out of `DataNN.csv` names; logging to `Data99.csv` | Yes — set once at boot |
| `0x20` | `SD_CARD_INIT_ERROR` — SD card failed to initialize at boot | Yes — set once at boot |
| `0x40` | `RFM95_INIT_ERROR` — LoRa radio failed to initialize at boot | Yes — set once at boot |
| `0x80` | `SHT_31_INIT_ERROR` — temperature/humidity sensor failed to initialize at boot | Yes — set once at boot |

`0x00` means no errors (`STATUS_OK`). The low nibble (per-cycle errors) is
cleared at the start of every cycle; the high nibble (boot-time errors)
persists for the life of the boot so the main node can tell a node is running
in a degraded state even on cycles where nothing new went wrong.

If `LORA_ERROR_REPORT` is enabled, hardware/SD errors are also sent to the
main node as a human-readable text message in addition to being reflected in
the status byte.
