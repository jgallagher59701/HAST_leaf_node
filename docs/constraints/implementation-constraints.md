<!-- CONTRACT: things that constrain HOW the system can be built, independent of
     what it must do. These are boundaries, not preferences — if a plan would cross
     one, that's a flag, not a quiet redesign. Delete the _EXAMPLE_ row once you've
     seen the shape. -->

# Implementation Constraints

Convention: `IC-###`. Category is one of: Technology/Platform, Infrastructure/
Deployment, Regulatory/Compliance, Organizational/Team, Third-Party/Integration,
Budget/Timeline (add categories as needed).

| ID | Category | Constraint | Rationale | Impact on design |
|---|---|---|---|---|
| IC-001 | Technology/Platform | Firmware must run on the RocketScream Mini Ultra Pro V3 (SAMD21, ARM M0, ~32 KB RAM) — currently the only supported board | Fixed hardware target; ARM M0 is RAM-constrained | No dynamic allocation after `setup()`; fixed-size buffers only; drives the `Arduino.h`-free logic layer / thin hardware-adapter split in `CLAUDE.md` |
| IC-002 | Regulatory/Compliance | LoRa radio operation must stay within the US ISM band (902–928 MHz) | FCC Part 15 unlicensed-operation requirement | Frequency/channel plan (`FREQUENCY` build flag, channel spacing) is constrained to this band; deploying outside the US would need re-certification |
| IC-003 | Infrastructure/Deployment | Leaf node is field-deployed with no mains power; battery is the sole power source | In situ soil sensor, unattended remote deployment | Drives the standby/sleep wake-cycle design (FR-004); no continuous listening or polling; informs NFR-001/NFR-002 |
| IC-004 | Technology/Platform | No radio link besides LoRa to the main node (no WiFi/cellular/BLE) | The RocketScream board carries only a LoRa transceiver | All communication with the main node must go through the LoRa reliable-datagram protocol; no fallback network path |
| IC-005 | Infrastructure/Deployment | No more than 25 leaf nodes per main node | Main node's battery budget allows ~2 minutes of awake time per hour; each leaf node needs a ~5 s transmit window, so N leaf nodes require N × 5 s of main-node listening time per cycle (25 × 5 s = 125 s, just under the 120 s budget) | Bounds node-addressing scheme (FR-007) and the main node's wake-window duration/schedule; adding leaf nodes beyond 25 requires either a shorter per-node transmit window or a larger main-node awake budget |
| IC-006 | Infrastructure/Deployment | Burial depth limited to < 3 inches | LoRa signal attenuation through soil; sensor itself can still measure at greater depth via its 18" cable, so this mainly constrains where the *enclosure* sits, not measurement depth | Constrains enclosure placement in the field; does not limit sensor placement depth |
| IC-007 | Infrastructure/Deployment | Enclosure and cable glands used for sensor wiring must be IP68-rated | Device is deployed outdoors/underground and exposed to moisture | Constrains enclosure and cable-gland selection; any wiring changes must preserve the IP68 seal |
| IC-008 | Budget/Timeline | Bill-of-materials cost must stay under $100 per prototype unit | Cost target for prototype-stage leaf nodes | Constrains component selection (sensor, radio module, MCU board, enclosure, cable glands) for any hardware revision |

<!-- A constraint is not the same as a non-functional requirement: an NFR describes
     a quality the system should have (and can trade off); a constraint is a boundary
     condition that plans must work within, full stop. If it's negotiable, it's
     probably an NFR with a priority, not a constraint. -->
