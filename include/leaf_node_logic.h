//
// Arduino-free logic layer extracted out of leaf_node.cc / get_battery_voltage.cc
// (TASK-001, IC-001). No dependency on the Arduino core, so it can be tested on
// the host (pio test -e native).
//
// James Gallagher <jgallagher@opendap.org>
// 10/11/26

#ifndef leaf_node_logic_h
#define leaf_node_logic_h

#include <stdint.h>
#include <time.h>

/// Parse compile-time __DATE__ ("Mmm dd yyyy") and __TIME__ ("hh:mm:ss") strings
/// into a unix epoch (seconds since 1 Jan 1970), to seed the RTC at boot before
/// the first time sync. Uses mktime(), so the result is in the process/device's
/// local time zone, not necessarily UTC - this matches the pre-extraction
/// behavior exactly (unchanged by TASK-001), not a new assumption.
/// @note input must be formatted exactly as the compiler emits __DATE__/__TIME__.
time_t get_epoch(const char *date, const char *time);

/// True every period-th message (message_count % period == 0) - decides whether
/// this cycle also sends a time request. period == 0 always returns false
/// rather than dividing by zero.
bool should_request_time(uint32_t message_count, uint32_t period);

/// Clear the per-cycle error flags (the low nibble: RFM95_SEND_ERROR,
/// RFM95_NO_REPLY, SD_FILE_ENTRY_WRITE_ERROR, SD_CARD_WAKEUP_ERROR) at the
/// start of a cycle, preserving the persistent boot-time error flags in the
/// high nibble (SD_NO_MORE_NAMES, SD_CARD_INIT_ERROR, RFM95_INIT_ERROR,
/// SHT_31_INIT_ERROR).
uint8_t clear_transient_status(uint8_t status);

/// Bump the two-digit numeric suffix at name[base_name_size]/[base_name_size+1]
/// (e.g. "00" -> "01", ..., "08" -> "09" -> "10", ..., "98" -> "99"). Returns
/// false, leaving name unchanged, once the suffix is already "99" (all 100
/// names exhausted); true (with name updated) otherwise. name must point at a
/// buffer at least base_name_size + 2 characters long, with two ASCII digits
/// at that offset.
bool bump_log_file_name(char *name, uint8_t base_name_size);

/// Decide whether the node's clock should be resynced to main_node_time, given
/// its current local_time (both unix epoch seconds). Always sets *out_delta to
/// main_node_time - local_time; returns true (resync) only if the magnitude of
/// that delta exceeds the 1-second threshold.
bool should_resync_time(uint32_t main_node_time, uint32_t local_time, int32_t *out_delta);

/// Convert an accumulated sum of raw ADC counts (raw_sum, over n_samples
/// readings) to centivolts (voltage * 100), for the 1M|1M battery-voltage
/// divider circuit on the 2.23V internal reference (get_battery_voltage.cc's
/// USE_AREF_2V23 path). Returns 0 if n_samples <= 0 (no valid average, avoids a
/// divide-by-zero) rather than an undefined/garbage reading.
int adc_average_to_centivolts(long raw_sum, int n_samples);

#endif
