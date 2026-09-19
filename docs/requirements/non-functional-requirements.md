<!-- CONTRACT: quality attributes the system must meet, with a measurable target where
     possible. "Fast" and "user-friendly" are not requirements — a number or a
     testable condition is. Delete the _EXAMPLE_ row once you've seen the shape. -->

# Non-Functional Requirements

Convention: `NFR-###`. Category is one of: Performance, Security, Reliability,
Availability, Scalability, Usability, Maintainability, Compliance, Observability
(add categories as needed — keep the list here in sync). Priority: `Must` / `Should`
/ `Could`. Status as in functional-requirements.md.

| ID | Category | Requirement | Measurable Target | Priority | Status |
|---|---|---|---|---|---|
| NFR-001 | Reliability | Leaf node operates on its battery, unserviced, in the field | > 1 year | Must | Proposed |
| NFR-002 | Reliability | Leaf node operates on its battery, unserviced, in the field (stretch target) | > 2 years | Should | Proposed |
| NFR-003 | Performance | LoRa link reaches the main node when the leaf node is above ground | >= 1,000 ft | Should | Proposed |
| NFR-004 | Performance | LoRa link reaches the main node when the leaf node is buried | >= 50 ft | Should | Proposed |
| NFR-005 | Reliability | LoRa transmissions are successfully delivered to the main node | >= 99% delivered (<= 1% dropped) | Should | Proposed |
| NFR-006 | Performance | Leaf node clock drift between time-synchronization requests to the main node stays bounded | < 2 s | Should | Proposed |

<!-- If you genuinely can't state a measurable target yet, write TBD in that column
     rather than a vague phrase — TBD is honest and searchable; "reasonably fast"
     just looks finished. -->
