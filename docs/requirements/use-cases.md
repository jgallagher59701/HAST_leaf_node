<!-- CONTRACT: actor-driven flows through the system. Each use case is what ties
     functional requirements together into something a real user actually does.
     Delete the _EXAMPLE_ use case once you've seen the shape. -->

# Use Cases

Convention: `UC-###`. Each use case follows the template below. Link related `FR-###`
/ `NFR-###` IDs at the bottom so `/trace` can find them in both directions.

---

## UC-001 — Leaf node completes a scheduled measurement and report cycle

**Actor(s):** Leaf node (autonomous, unattended); Main node (receiver)

**Trigger:** RTC standby timer wakes the leaf node at its configured schedule (default:
top of the hour; once-per-minute in test mode)

**Preconditions:** Leaf node is deployed in the field, powered by battery, with SD
card and LoRa transceiver present and functioning; main node is reachable over LoRa

**Main flow:**
1. Leaf node wakes from standby (FR-004)
2. Leaf node reads temperature and humidity from the sensor (FR-001)
3. Leaf node reads its battery voltage (FR-006)
4. Leaf node logs the reading to the SD card (FR-002)
5. Leaf node transmits the reading to the main node as a reliable LoRa datagram
   (FR-003), tagged with its node address (FR-007)
6. Leaf node periodically synchronizes its clock from the main node via a time
   request/response exchange (FR-005)
7. Leaf node returns to standby until the next scheduled wake

**Alternate / exception flows:**
- LoRa transmission is not acknowledged → reading remains on the SD card
  (FR-002 already satisfied independent of delivery); node proceeds to standby per
  NFR-005's dropped-transmission allowance
- Debug mode enabled → diagnostics are additionally routed over USB serial or LoRa
  (FR-008) instead of, or alongside, normal operation

**Postconditions:** Reading exists on the SD card; reading has been transmitted to
the main node (subject to the delivery-reliability target); leaf node is back in
standby, drawing minimal power until the next wake

**Related requirements:** FR-001, FR-002, FR-003, FR-004, FR-005, FR-006, FR-007,
FR-008, NFR-001, NFR-002, NFR-003, NFR-004, NFR-005, NFR-006, IC-001, IC-002, IC-003,
IC-004

---

<!-- Copy the template above for each new use case. Sequential UC-### heading,
     same subsection order, so /trace can parse it. -->
