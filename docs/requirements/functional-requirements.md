<!-- CONTRACT: what the system must DO. One row per requirement. IDs are permanent —
     never renumbered or reused. Dropped requirements get Status: Deprecated /
     Superseded, not deleted. Delete the _EXAMPLE_ row once you've seen the shape. -->

# Functional Requirements

Convention: `FR-###`, zero-padded, assigned sequentially by next-highest-number.
Priority: `Must` / `Should` / `Could` (MoSCoW). Status: `Proposed` / `Approved` /
`Implemented` / `Deprecated` / `Superseded by FR-0XX`.

| ID | Requirement | Priority | Related Use Cases | Status |
|---|---|---|---|---|
| FR-001 | Leaf node reads temperature and humidity from the SHT30/31 sensor each wake cycle | Must | UC-001 | Implemented |
| FR-002 | Leaf node logs each reading locally to the SD card | Must | UC-001 | Implemented |
| FR-003 | Leaf node transmits each reading to the main node over LoRa using a reliable (acknowledged) datagram | Must | UC-001 | Implemented |
| FR-004 | Leaf node wakes on a scheduled interval via RTC standby/sleep (default: top of the hour; a once-per-minute mode exists for testing) | Must | UC-001 | Implemented |
| FR-005 | Leaf node synchronizes its clock from the main node via a time request/response exchange | Should | UC-001 | Implemented |
| FR-006 | Leaf node reports battery voltage alongside sensor data | Should | UC-001 | Implemented |
| FR-007 | Each leaf node has a unique, compile-time node address distinguishing it from other leaf nodes and from the main node | Must | UC-001 | Implemented |
| FR-008 | Leaf node supports a debug mode that can route diagnostics over USB serial or over LoRa | Could | | Implemented |

<!-- Add new rows via /new-requirement, or by hand — keep the table format. Each
     requirement should be a single testable statement, not a paragraph. If it needs
     a paragraph, it's probably a use case (docs/requirements/use-cases.md) with this
     as one of its acceptance criteria. -->
