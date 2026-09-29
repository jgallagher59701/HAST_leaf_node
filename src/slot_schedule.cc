//
// Map a leaf node's address to its hourly LoRa transmit slot (FR-011, FR-012).
//
// James Gallagher <jgallagher@opendap.org>
// 9/28/26

#include "slot_schedule.h"

bool slot_start_s(uint8_t node, SlotSchedule schedule, uint16_t grouped_start_s, uint16_t *start_s) {
    if (!start_s || !slot_is_valid(node, schedule, grouped_start_s))
        return false;

    *start_s = (uint16_t)slot_start_unchecked_s(node, schedule, grouped_start_s);
    return true;
}

bool slot_to_mmss(uint16_t start_s, uint8_t *mm, uint8_t *ss) {
    if (!mm || !ss || start_s >= SECONDS_PER_HOUR)
        return false;

    *mm = (uint8_t)(start_s / SECONDS_PER_MINUTE);
    *ss = (uint8_t)(start_s % SECONDS_PER_MINUTE);
    return true;
}
