# Plan: Node-address-based transmit time slots

**Status:** Draft
**Created:** 2026-09-27

## Summary

Each leaf node derives its hourly wake/transmit time from its `NODE_ADDRESS` so that
up to 15 leaf nodes share the main node in non-overlapping 5 s slots: first the
staggered layout (node N at minute N−1, second 0 — FR-011), later, after hardware
testing, the grouped layout (all slots in one 75 s window — FR-012). The staggered
phase doubles as a measurement campaign: it records how far leaf-node clocks drift
and how long each LoRa exchange actually takes, so that the guard bands for the
grouped layout, and a revised NFR-006, rest on data rather than on estimates. The
goal is a main node that can sleep for most of the hour.

## Requirements traced

- FR-011 — staggered slot schedule (v1): phases 1, 2, 3, 5.
- FR-012 — grouped slot schedule (v2): phases 1 (computation only) and 6 (enablement).
  Priority is still `TBD`; phase 6 is gated on it.
- FR-004 — the scheduled RTC wake becomes the slot start rather than a hand-set
  `WAKE_UP_MINUTE`/`WAKE_UP_SECOND`: phase 2.
- FR-005 — time sync has to fit inside the slot, and its results are the drift
  measurement: phases 3, 4.
- FR-007 — the slot is a function of the compile-time node address; the address range
  is narrowed to 1–15: phase 1.
- FR-003 — the slot has to hold the acknowledged exchange, including retries: phase 3.
  Depends on BUG-001 (see Phase 0).
- NFR-006 — measured in phase 4/5; revised afterwards via `/new-requirement` (phase 5
  output, not done by this plan).
- NFR-005 — deadline enforcement (phase 3) must not drop more than the delivery
  target allows; measured in phase 5.
- NFR-001 / NFR-002 — the slot deadline puts a bound on the leaf node's awake time per
  cycle (phase 3); nothing here should lengthen it.
- UC-001 — steps 1, 5, 6 as revised 2026-09-27.

## Constraints considered

- **IC-001** (SAMD21, ~32 KB RAM) — the slot arithmetic and deadline checks go in a
  new `Arduino.h`-free unit with fixed-width types, no allocation, and native Unity
  tests (the `src/`+`include/` convention from TASK-001). `leaf_node.cc` only calls it.
- **IC-002** (US ISM band) — no change to frequency or modem settings. **Flag, not
  caused by this plan but made visible by it:** at SF10/125 kHz, one data frame is
  ~0.4 s of airtime, which is right at the 400 ms per-channel dwell limit that FCC
  15.247 applies to frequency-hopping systems in 902–928 MHz, and the node uses one
  fixed channel. Whether the current radio setup is compliant has not been reviewed
  anywhere in `docs/`. It deserves its own look; this plan doesn't change it either way.
- **IC-003** (battery only) — the leaf node still wakes once per hour. The deadline
  in phase 3 limits how long it can stay awake when the link is bad.
- **IC-004** (LoRa only) — all timing information travels over the existing
  reliable-datagram messages. **No change to `lib/soil_sensor_common` message
  formats** in this plan: that library is its own repo shared with `HAST_lora_main`.
- **IC-005** (≤ 15 leaf nodes, 5 s slots, 120 s main-node budget) — enforced at build
  time: a `static_assert` rejects `NODE_ADDRESS` outside 1–15. Grouped layout uses 75 s
  of the 120 s budget.
- IC-006, IC-007, IC-008 — hardware/enclosure/BOM; not affected.

**Tension to decide (FR-011 vs FR-005 / FR-003):** FR-011 says the whole LoRa
exchange has to finish within 5 s. Estimated worst case with the current
`setRetries(2)` / `setTimeout(400)`: a data message with 3 attempts takes ~3.5 s, and a
time sync adds ~0.7–1 s (or up to ~3.5 s if it also retries), before any guard band.
Staying inside the slot therefore means that, on a bad cycle, something gets cut
short: fewer retries, or a time sync skipped until the next cycle. Phase 3 proposes
the latter. That is a real change in behavior against FR-005's "periodically" and it
needs your decision (open question 1).

## Phases

### Phase 0 — Prerequisite: BUG-001 (acknowledged sends)

**Goal:** Data and time-request sends go to `MAIN_NODE_ADDRESS`, so ACK/retry actually
happens.
**Satisfies:** FR-003 (via `plans/bugfix-lora-broadcast-address-plan.md`)

Not new work in this plan. It comes first because every timing number in phases 3–5
depends on whether ACKs and retries happen. Measuring slot duration while sends are
still fire-and-forget broadcasts would produce a guard band that's too small.

### Phase 1 — Slot schedule in the logic layer

**Goal:** A pure function maps a node address and schedule to a slot start (seconds
past the hour), with native tests.
**Satisfies:** FR-011, FR-012, FR-007, IC-005

Steps:
1. Add `include/slot_schedule.h` / `src/slot_schedule.cc` (no `Arduino.h`):
   - `enum class SlotSchedule : uint8_t { Staggered, Grouped };`
   - `constexpr` slot constants: `SLOT_LENGTH_S = 5`, `MAX_LEAF_NODES = 15`.
   - `bool slot_start_s(uint8_t node, SlotSchedule s, uint16_t grouped_start_s,
     uint16_t *start_s)` — Staggered: `(node−1)·60`; Grouped:
     `grouped_start_s + 5·(node−1)`. Returns `false` if the node is outside 1–15 or the
     slot would cross the end of the hour.
   - `void slot_to_mmss(uint16_t start_s, uint8_t *mm, uint8_t *ss)` for the RTC alarm.
2. Build flags (in `leaf_node.cc` with `#ifndef` defaults, same pattern as today):
   `SLOT_SCHEDULE` (default Staggered), `GROUPED_START_S` (open question 3), and
   `SLOT_LEAD_GUARD_MS` (phase 3).
3. `static_assert(NODE_ADDRESS >= 1 && NODE_ADDRESS <= 15)` in `leaf_node.cc`. The
   current `platformio.ini` value (5) already passes.
4. Native test `test/native_slot_schedule/`: nodes 1, 12, 13, 15 in both layouts; 0 and
   16 are rejected; a grouped start that would run past :59:59 is rejected; mm/ss
   conversion.

**Risks:** Low. The main node must use the same numbers; if it grows its own copy,
the two can drift apart (open question 4).

### Phase 2 — Wake at the slot

**Goal:** `sleep_node()` sets its hourly alarm from the slot, not from hand-set flags.
**Satisfies:** FR-004, FR-011, UC-001 step 1

Steps:
1. In `sleep_node()` (`src/leaf_node.cc` ~L640–645), replace
   `setAlarmTime(0, WAKE_UP_MINUTE, WAKE_UP_SECOND)` with the mm:ss from
   `slot_start_s()`/`slot_to_mmss()`, still `MATCH_MMSS`.
2. Remove `WAKE_UP_MINUTE`/`WAKE_UP_SECOND` from `leaf_node.cc` and `platformio.ini`.
   Add an `#error` if either is still defined, so an old build config fails loudly
   instead of being silently ignored.
3. Leave `SAMPLE_ONCE_PER_MINUTE` test mode unchanged for now (open question 5).
4. Check the wake → start-of-TX latency: in `loop()` the data message is built and
   sent right after waking. Confirm on hardware (state pins already exist:
   `STATE_1`) that this stays well under 100 ms with `DEBUG_LOG=0`. Also measure it
   with `DEBUG_LOG=1`, since the `IO_LOG` SD writes before the send may add delay.

**Risks:** The slot start depends on the leaf RTC being set correctly at boot. Today
`setup()` seeds it from `__DATE__/__TIME__` and then asks the main node for the time.
If that first sync fails, the node sleeps into the wrong slot until a later sync
succeeds. Staggered layout keeps nodes ~1 min apart, so this can't cause collisions in
v1, only missed windows.

### Phase 3 — Keep the exchange inside the slot

**Goal:** Every LoRa exchange of a cycle finishes before the slot ends; overruns are
recorded.
**Satisfies:** FR-011, FR-003, FR-005, NFR-001, NFR-005

Steps:
1. Reorder `loop()` so both LoRa exchanges happen back to back at the start of the
   slot: data message, then (when due) the time request, then `log_data()` to SD.
   Today the SD write sits between them (`src/leaf_node.cc` ~L928–930).
2. Add a pure helper to `slot_schedule`:
   `bool fits_in_slot(uint32_t elapsed_ms, uint32_t needed_ms)` against
   `SLOT_LENGTH_S·1000 − trailing guard`. Before the time request, `loop()` checks
   the time elapsed since waking (`millis()`). If the request won't fit, it's
   deferred to the next cycle instead of dropped (pending decision on open question 1).
3. Leading guard: `SLOT_LEAD_GUARD_MS` holds off the first send after waking, so that a
   leaf running slightly *fast* doesn't transmit before the main node is listening.
   Provisional default is 0 in v1, since the main node is always awake today. Its
   real value comes from phase 5.
4. Log slot use on every cycle: elapsed ms at the end of the exchange, and whether the
   time request was deferred. This goes to the SD data log, not only the debug log.
   `last_tx_duration` already sends the data-exchange time to the main node one cycle
   late; keep it.
5. Native tests for `fits_in_slot()` edges.

**Risks:** Deferring syncs lets drift grow between corrections; phase 4 measures how
much. Reducing `setRetries` would also make it fit, but costs delivery (NFR-005), so
this plan doesn't propose it.

### Phase 4 — Drift and sync instrumentation

**Goal:** Every successful sync records how far the clock drifted since the last one,
in a form that can be converted to ppm.
**Satisfies:** NFR-006, FR-005

Steps:
1. In `update_time()` (`src/leaf_node.cc` ~L545), write a record to the SD data log
   on every sync: main time, local time, `delta`, seconds since the last sync, and the
   node's current temperature (drift depends strongly on temperature). Today the
   delta goes only to the debug log.
2. Measurement build: always apply the correction, instead of only when `|delta| > 1`.
   Then each recorded delta covers exactly one sync interval and isn't carried over
   from the previous one. Whether the `> 1` threshold stays in production is decided
   in phase 5.
3. Make `TIME_REQUEST_SAMPLE_PERIOD` a documented test knob. A longer period gives a
   finer ppm estimate from whole-second deltas (±1 s over 24 h ≈ ±12 ppm), and a
   shorter one shows the residual error right after a sync.
4. Known limit, recorded rather than fixed: the correction resolves only whole
   seconds, and the ~0.4 s the response spends in the air is not compensated. So the
   residual error after a sync is up to ~1–1.5 s no matter how small the drift is.
   Sub-second sync is a separate decision (open question 2).

**Risks:** Leaf-side records only show drift at each sync. Knowing where a packet lands
within its slot needs the main node to log when it received each message, which is a
`HAST_lora_main` change (open question 4).

### Phase 5 — v1 hardware run and sizing

**Goal:** Turn the measurements into numbers for guard bands, the sync period, and a
revised NFR-006.
**Satisfies:** FR-011 (verification), NFR-006, NFR-005

Steps:
1. Deploy N staggered leaf nodes (N = TBD) for a TBD duration, over as wide a
   temperature range as practical. The expected worst case is cold soil, where these
   crystals run slowest.
2. From the phase 3/4 logs: drift in ppm vs. temperature, worst-case exchange
   duration, deferred-sync rate, residual error after a sync.
3. Derive: leading/trailing guard (ms), the time-request period needed to stay inside
   the guard, and whether 5 s slots hold in the grouped layout.
4. Outputs, as separate steps that go back through the requirement process:
   `/new-requirement` to revise NFR-006 and to add a slot-timing-accuracy NFR; an ADR
   recording the guard band and sync period and why.

**Risks:** A test in mild weather will underestimate cold-soil drift.

### Phase 6 — Enable the grouped layout (v2)

**Goal:** Switch `SLOT_SCHEDULE` to Grouped with the guard bands from phase 5.
**Satisfies:** FR-012

Gated on: phase 5 outputs, FR-012 priority set, `GROUPED_START_S` decided, and a main
node that sleeps outside the 75 s window (`HAST_lora_main`).

Steps:
1. Set `GROUPED_START_S` and the guard values; rebuild all nodes. Slots are now back
   to back, so drift causes **collisions**, not only missed windows.
2. Re-run a shorter hardware test focused on adjacent-slot collisions (NFR-005).

## Open questions

1. **Blocks phase 3.** When the data exchange runs long and the time request won't fit
   in the slot, should the sync be deferred to the next cycle (proposed), should the
   node reduce retries, or is overrunning the slot acceptable in v1 (slots are a
   minute apart)?
2. **Doesn't block v1; may block v2.** Is whole-second sync accurate enough, or do we
   need sub-second sync? That could mean compensating for time in the air, or the main
   node answering on a second boundary. This depends on phase 5 data.
3. **Blocks phase 6.** Which second of the hour is `GROUPED_START_S`? FR-012 leaves it
   open.
4. **Blocks phase 4's slot-offset measurement and phase 6.** The main node side (logging
   receive time, sleeping between slots, sharing the slot constants) lives in
   `HAST_lora_main`. Does it get its own plan there, and should the slot constants
   live in `lib/soil_sensor_common` (the shared repo) so the two nodes can't disagree?
5. **Doesn't block.** What should `SAMPLE_ONCE_PER_MINUTE` test mode do? Today it
   ignores node number. 15 × 5 s doesn't fit in a minute, so a per-minute
   version of the slot layout isn't possible as-is.
6. **Blocks phase 5.** How many nodes are available for the v1 run, and for how long?

## Out of scope

- Main-node firmware (`HAST_lora_main`): listen windows, sleep, receive-time logging.
- Changes to message formats in `lib/soil_sensor_common`.
- Automatic drift-rate correction on the leaf node, or the main node pushing time
  corrections. These are possible follow-ups if phase 5 shows whole-second syncs
  aren't enough.
- Changes to the radio modem or frequency, including the IC-002 dwell-time question
  above.
- Revising NFR-006 itself. That's a phase 5 output through `/new-requirement`, not an
  edit this plan makes.

## Work Log

**2026-09-27 20:41** — "Update UC-001. Let's plan before updating NFR-006 to see how much that will drift. IC-005 needs to be updated; drop it down to 15 leaf-nodes and fix the math." → `/plan-feature` for FR-011/FR-012 plus drift measurement.
Drafted the plan against FR-011/FR-012 (added this session) and the revised UC-001 and
IC-005. The main finding is that a worst-case exchange (3 data attempts + time sync)
doesn't fit in 5 s with current retry settings. So the plan adds a slot deadline that
defers the time sync, flagged as a decision against FR-005. NFR-006 is deliberately
left unchanged: the staggered v1 is run as a measurement campaign (drift per sync,
exchange duration) whose results feed a later NFR-006 revision and an ADR.
BUG-001 is a prerequisite because timing without ACKs would understate slot use.
