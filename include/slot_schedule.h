//
// Map a leaf node's address to its hourly LoRa transmit slot (FR-011, FR-012).
// Arduino-free so it can be tested on the host (pio test -e native).
//
// Written to C++11 (the SAMD core builds with -std=gnu++11), so the constexpr
// functions are single return expressions.
//
// James Gallagher <jgallagher@opendap.org>
// 9/28/26

#ifndef slot_schedule_h
#define slot_schedule_h

#include <stdint.h>

/// Slot layout: Staggered puts node N at minute N-1, second 0 (FR-011); Grouped
/// puts node N at grouped_start_s + SLOT_LENGTH_S * (N-1) (FR-012).
enum class SlotSchedule : uint8_t { Staggered = 0, Grouped = 1 };

constexpr uint16_t SLOT_LENGTH_S = 5;   ///< Length of one node's slot, seconds (IC-005)
constexpr uint8_t MAX_LEAF_NODES = 15;  ///< Leaf-node addresses are 1..MAX_LEAF_NODES (IC-005)
constexpr uint16_t SECONDS_PER_MINUTE = 60;
constexpr uint16_t SECONDS_PER_HOUR = 3600;

/// Start of a node's slot in seconds past the hour, with no range checking; node must be >= 1.
constexpr uint32_t slot_start_unchecked_s(uint8_t node, SlotSchedule schedule, uint16_t grouped_start_s) {
    return schedule == SlotSchedule::Staggered
               ? (uint32_t)(node - 1) * SECONDS_PER_MINUTE
               : (uint32_t)grouped_start_s + (uint32_t)(node - 1) * SLOT_LENGTH_S;
}

/// True if node is 1..MAX_LEAF_NODES, schedule is known, and the whole slot ends by the end of the hour.
constexpr bool slot_is_valid(uint8_t node, SlotSchedule schedule, uint16_t grouped_start_s) {
    return node >= 1 && node <= MAX_LEAF_NODES
           && (schedule == SlotSchedule::Staggered || schedule == SlotSchedule::Grouped)
           && slot_start_unchecked_s(node, schedule, grouped_start_s) + SLOT_LENGTH_S <= SECONDS_PER_HOUR;
}

/// Set *start_s to the node's slot start, seconds past the hour; false (and *start_s unchanged) if invalid.
bool slot_start_s(uint8_t node, SlotSchedule schedule, uint16_t grouped_start_s, uint16_t *start_s);

/// Split seconds past the hour into minute and second for the RTC alarm; false if start_s >= 3600.
bool slot_to_mmss(uint16_t start_s, uint8_t *mm, uint8_t *ss);

#endif
