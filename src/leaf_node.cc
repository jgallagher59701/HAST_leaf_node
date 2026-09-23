//
// Simple lora client. Uses Simple lora server.
//
// Modified from the pro-mini version to work with the Rocket Scream
// Mini Ultra Pro, V3, which includes a LoRa module on the RS board.
//
// Based on LoRa Simple Yun Client by Edwin Chen <support@dragino.com>,
// Dragino Technology Co., Limited
//
// James Gallagher <jgallagher@opendap.org>
// 7/26/20

#include <Arduino.h>

#include <string.h>
#include <time.h>

#include <SPI.h>
#include <Wire.h>

#include <RHDatagram.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <RTCZero.h>
#include <SerialFlash.h>

#include "Adafruit_SHT31.h" // Uses the BusIO library from Adafruit
#include "SdFat.h"

#include "blink.h"
#include "data_packet.h"
#include "get_battery_voltage.h"
#include "messages.h"

// Exclude some parts of the code for debugging. Zero excludes the code.
#define DEBUG 0             // setup()-time diagnostics over USB serial; requires USB (FR-008)
#define DEBUG_LOG 0         // Write loop()-time diagnostics to a debug log file on the SD card (FR-009)
#define LORA_ERROR_REPORT 0 // Report hardware/SD-card errors to the main node over LoRa (FR-010)
#include "debug.h"

#define MAIN_NODE_ADDRESS 0

// Define in the platformio file. jhrg 7/31/21
#ifndef NODE_ADDRESS
#define NODE_ADDRESS 4
#endif

#define Serial SerialUSB // Needed for RS. jhrg 7/26/20
#define SERIAL_CONNECT_TRIES 10
#define SERIAL_CONNECT_INTERVAL 1000 // ms

#ifndef STANDBY_MODE
#define STANDBY_MODE 1
#endif

#define TX_LED 1 // 1 == show the LED during operation, 0 == not
#define SHT30D 1
#define SD 1
#define SPI_SLEEP 1
#define LORA 1

// Pin assignments

#define RFM95_INT 2 // RF95 Interrupt
#define FLASH_CS 4  // CS for 2MB onboard flash on the SPI bus
#define RFM95_CS 5  // RF95 SPI CS

// NB: The two hand-built units have SD_PWR on 11, the PCB uses pin 9
#if NODE_ADDRESS < 3
#define SD_PWR 11 // HIGH == power on SD card; hand built nodes use pin 11 for this
#else
#define SD_PWR 9 // HIGH == power on SD card; hand built nodes use pin 11 for this
#endif
#define SD_CS 10 // CS for the SD card, SPI uses dedicated lines

#define STATUS_LED 13

// These GPIO pins are used for debugging the leaf node state in case
// it crashes/freezes. Could add 17–19 if needed. jhrg 1/1/21
#define STATE_1 3
#define STATE_2 6
#define STATE_3 7
#define STATE_4 8
#define STATE_5 12

#define USE_AREF_2V23 1
#define V_BAT A1 // A5

// Constants

// Channel 0 is 902.3, others are + 200KHz for BW = 125 KHz. There are 64 channels.
// 915.0 MHz is the no-channel nominal freq
// Define in the platformio file. jhrg 7/31/21
#ifndef FREQUENCY
#define FREQUENCY 902.3
#endif

// Use these settings:
#define BANDWIDTH 125000
#define SPREADING_FACTOR 10
#define CODING_RATE 5

// RH_CAD_DEFAULT_TIMEOUT 10seconds

#ifndef STANDBY_INTERVAL_S
#define STANDBY_INTERVAL_S 300 // seconds to wait/sleep before next transmission; only used when STANDBY_MODE is 0
#endif

#ifndef TIME_REQUEST_SAMPLE_PERIOD
#define TIME_REQUEST_SAMPLE_PERIOD 24  // Ask the time once for every N data samples
#endif

// Normal operation: wake once per hour, at WAKE_UP_MINUTE:WAKE_UP_SECOND, using
// the RTC's recurring MATCH_MMSS alarm (matches every hour; day/month/year are
// ignored). Because the match is on minute/second only, it is unaffected by a
// time_request/time_response clock correction applied before the node sleeps -
// unlike an absolute wake-up epoch, this alarm can never end up already in the
// past.
#ifndef WAKE_UP_MINUTE
#define WAKE_UP_MINUTE 0 // minute of the hour to wake up and sample, 0-59
#endif

#ifndef WAKE_UP_SECOND
#define WAKE_UP_SECOND 0 // second of that minute to wake up and sample, 0-59
#endif

// For testing: sample once per minute, at SAMPLE_SECOND, using the RTC's
// recurring MATCH_SS alarm, instead of once per hour.
#ifndef SAMPLE_ONCE_PER_MINUTE
#define SAMPLE_ONCE_PER_MINUTE 0
#endif

#ifndef SAMPLE_SECOND
#define SAMPLE_SECOND 0 // second of each minute to wake up and sample, 0-59; only used when SAMPLE_ONCE_PER_MINUTE is 1
#endif

#define BOOT_SAFETY_DELAY 10000  // 10s

#define WAIT_AVAILABLE 5000   // ms to wait for reply from main node
#define SD_POWER_ON_DELAY 200 // ms

#define ADC_BITS 12
#define ADC_MAX_VALUE 4096

// Log file name.
#define FILE_BASE_NAME "Data"
// Fixed name, appended to across boots - unlike FILE_BASE_NAME's numbered
// DataNN.csv rollover scheme, there is only ever one debug log (FR-009).
#define DEBUG_LOG_FILE_NAME "Debug.log"
// To avoid a race condition when using 'standby mode' this must be >= 2.
#define SD_CARD_WAIT 2 // seconds to wait after last write before power off

// Error codes for setup(). If any of these errors during the boot of the node
// flash the status led 2, 3, ..., n times. The sequence will repeat ERROR_TIMES
// then the node boot will continue. The node status will also be set to include
// some boot error information (see the codes below).
#define SHT31_BEGIN_FAIL 2
#define SD_BEGIN_FAIL 3
#define SD_WRITE_HEADER_FAIL 4
#define RFM95_INIT_FAIL 5
#define RFM95_SET_FREQ_FAIL 6
#define SD_LOG_FILE_NAME_FAIL 7

#define ERROR_TIMES 3

// In the RH Datagram and ReliableDatagram header, we can use the four
// LSB of the 'status' field. However, currently node status is part of
// the data packet.
#define STATUS_OK 0x00

// These errors are reset on every iteration of loop()
#define RFM95_SEND_ERROR 0x01
#define RFM95_NO_REPLY 0x02
#define SD_FILE_ENTRY_WRITE_ERROR 0x04
#define SD_CARD_WAKEUP_ERROR 0x08

// These codes indicate errors at boot time. They are persistent.
#define SD_NO_MORE_NAMES 0x10 // means it will use "Data99.csv"
#define SD_CARD_INIT_ERROR 0x20
#define RFM95_INIT_ERROR 0x40
#define SHT_31_INIT_ERROR 0x80

#if LORA
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// Singleton instance for the reliable datagram manager
RHReliableDatagram rf95_manager(rf95, NODE_ADDRESS);
#endif

unsigned int tx_power = 23; // dBm; 5 to 23 for RF95

// Singleton for the Real Time Clock
RTCZero rtc;

// Temp/humidity sensor. Use the SHT31 code. We actually have the 30-D sensor.
Adafruit_SHT31 sht30d = Adafruit_SHT31();

// Singletons for the SD card objects
SdFat sd; // File system object.

uint8_t status = STATUS_OK;

/** 
 * @brief Call back for the sleep alarm
 */
void alarmMatch() {
    rtc.detachInterrupt();
}

/**
 * @brief Report a hardware/SD-card error to the main node over LoRa (FR-010)
 * @param msg The message; null terminated string
 * @param to Send to this node
 */
void report_error_to_main_node(const char *msg, uint8_t to) {
#if LORA && LORA_ERROR_REPORT
    rf95_manager.sendtoWait((uint8_t *)msg, strlen(msg) + 1, to);
#endif
}

// Set to 1 to use delay(), zero to use yield(). There are problems
// debugging the RocketScream using yield().
#define USE_DELAY 1

/**
   @brief delay that enables background tasks

   Used for debugging and to enable program upload. See setup().

   @note Cannot be used when interrupts are disabled (cf. millis())
*/
void yield(unsigned long ms_delay) {
#if USE_DELAY
    delay(ms_delay);
#else
    unsigned long start = millis();
    while ((millis() - start) < ms_delay)
        yield();
#endif
}

/**
   @brief Get the current epoch from __DATE__ and __TIME__

   This function returns the time in seconds since 1 Jan 1970
   using the string values of the compile-time constants
   __DATE__ and __TIME_. The formats of these are: mmm dd yyyy
   (e.g. "Jan 14 2012") and hh::mm::ss in 24 hour time
   (e.g. "22:29:12")

   @note input must be formatted correctly
   @param data The value of __DATE__ or the equiv
   @param time The value of __TIME__ or the equiv
   @return Seconds since Jan 1, 1970
*/
time_t
get_epoch(const char *date, const char *time) {
    char s_month[5];
    struct tm t = {0};
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

    sscanf(date, "%s %d %d", s_month, &t.tm_mday, &t.tm_year);

    // pointer math
    int month = (strstr(month_names, s_month) - month_names) / 3;

    t.tm_mon = month;
    t.tm_year -= 1900;
    t.tm_isdst = -1;

    sscanf(time, "%d:%d:%d", &t.tm_hour, &t.tm_min, &t.tm_sec);

    return mktime(&t);
}

/// Use get_log_filename() and get_new_log_filename()
char file_name[13] = FILE_BASE_NAME "00.csv";

/**
 * @brief get an unused filename for the new log.
 * 
 * This function returns a pointer to local static storage. 
 * 
 * @note Only call this from setup(), never from loop() and never if
 * the SD library has not been initialized correctly and never after
 * the Radiohead library (RFM95) has been initialized.
 * 
 * @return A pointer to the new file name. Global static storage.
 * @see get_log_filename()
 */
const char *
get_new_log_filename() {
    // If the SD card/library failed to init, don't run this code.

    const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

    // Find an unused file name.
    if (BASE_NAME_SIZE > 6) {
        blink(STATUS_LED, SD_LOG_FILE_NAME_FAIL, ERROR_TIMES);
    }

    // Look for a BASE_NAME00.csv. if all are taken return BASE_NAME99.csv
    while (sd.exists(file_name)) {
        if (file_name[BASE_NAME_SIZE + 1] != '9') {
            file_name[BASE_NAME_SIZE + 1]++;
        } else if (file_name[BASE_NAME_SIZE] != '9') {
            file_name[BASE_NAME_SIZE + 1] = '0';
            file_name[BASE_NAME_SIZE]++;
        } else {
            status |= SD_NO_MORE_NAMES;
            break;
        }
    }

    return file_name;
}

/**
 * @return The log file name
 */
const char *
get_log_filename() {
    return file_name;
}

/**
   @brief Write a header for the new log file.
   @param file_name open/create this file, append if it exists
   @note Never call this if the SD card initialization fails.
*/
void write_header(const char *file_name) {
#if SD
    if (status & SD_CARD_INIT_ERROR) {
        return;
    }

    // disable interrupts
    noInterrupts();

    FatFile file; // Log file.
    if (file.open(file_name, O_WRONLY | O_CREAT | O_APPEND)) {
        file.write("# Start Log\n");
        // getWriteError returns true if there was an error writing OR
        // if the file has been closed (which is not an error...)
        if (file.getWriteError())
            status |= SD_FILE_ENTRY_WRITE_ERROR;
        file.close();
    } else {
        status |= SD_FILE_ENTRY_WRITE_ERROR;
    }

    if (status & SD_FILE_ENTRY_WRITE_ERROR) {
        char error_info[256];
        int error = sd.sdErrorCode();
        // sd.errorPrint(error_info);
        snprintf(error_info, 256, "SD Card error: 0x%02x, node status: 0x%02x.", error, status);

        IO(Serial.println(F("Couldn't write file header")));
        IO(Serial.println(error_info));

        blink(STATUS_LED, SD_WRITE_HEADER_FAIL, ERROR_TIMES);
    }

    // enable interrupts
    interrupts();
#endif
}

/**
   @brief log data
   write data to the log, append a new line
   @param file_name open for append
   @param data write this char string
*/
void log_data(const char *file_name, const char *data) {
#if SD
    if (status & SD_CARD_INIT_ERROR) {
        return;
    }

    // disable interrupts
    noInterrupts();

    FatFile file; // Log file.
    if (file.open(file_name, O_WRONLY | O_CREAT | O_APPEND)) {
        file.write(data);
        file.write('\n');
        // getWriteError returns true if there was an error writing OR
        // if the file has been closed (which is not an error...)
        if (file.getWriteError())
            status |= SD_FILE_ENTRY_WRITE_ERROR;
        file.close();
    } else {
        status |= SD_FILE_ENTRY_WRITE_ERROR;
    }

    if (status & SD_FILE_ENTRY_WRITE_ERROR) {
        char error_info[256];
        int error = sd.sdErrorCode();
        // sd.errorPrint(error_info);
        snprintf(error_info, 256, "SD Card error: 0x%02x, node status: 0x%02x.", error, status);

        interrupts(); // enable interrupts for the rfm95

        report_error_to_main_node(error_info, MAIN_NODE_ADDRESS);
    }

    // enable interrupts
    interrupts();
#endif
}

/**
 * @brief Append a timestamped line to the SD-card debug log (FR-009)
 *
 * For information useful to understand program flow/timing while testing -
 * not needed during normal operation - without the execution-halting cost of
 * a debug probe. Reuses log_data()'s file I/O and error handling (including
 * reporting a write failure to the main node via report_error_to_main_node()),
 * against the single append-only DEBUG_LOG_FILE_NAME file rather than the
 * numbered data-sample log.
 * @param msg Null-terminated diagnostic message; a date/time stamp is prepended
 */
void debug_log(const char *msg) {
    char line[320];
    snprintf(line, sizeof(line), "%d/%d/%dT%d:%d:%d %s", rtc.getMonth(), rtc.getDay(), rtc.getYear(),
             rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), msg);

    log_data(DEBUG_LOG_FILE_NAME, line);
}

/**
 * @brief Shutdown the SD card by cutting power
 * 
 * Wait for SD_CARD_WAIT seconds before cutting the power.
 */
void shutdown_sd_card() {
    // FIXME only do this if the card started. jhrg 9/26/21
#if STANDBY_MODE
    // Wait SD_CARD_WAIT seconds for the SD card to settle.
    rtc.setAlarmEpoch(rtc.getEpoch() + SD_CARD_WAIT);
    rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
    rtc.attachInterrupt(alarmMatch);
    yield(10);
    rtc.standbyMode();
#else
    yield(SD_CARD_WAIT * 1000);
#endif

    digitalWrite(SD_PWR, LOW); // Now, turn off the SD card
}

/**
 * @brief Power on teh SD card and initialize the driver
 */
void wake_up_sd_card() {
    // FIXME Only do this if the SD card was initialized. jhrg 9/26/21
    digitalWrite(SD_PWR, HIGH);

    // Calling this here freezes the RS. jhrg 3/24/21
    // noInterrupts();

    yield(SD_POWER_ON_DELAY);

    // FIXME If the SD card didn't init, don't try to start it here.
    // OR, maybe we should try if it was a transient problem? Probably
    // a problem at boot time is real and should not be ignored. jhrg 9/25/21
    if (!sd.begin(SD_CS)) {
        status |= SD_CARD_WAKEUP_ERROR;
    }

    // See above. interrupts();
}

void send_message(uint8_t *data, uint8_t to, uint32_t size) {
#if LORA
    // This may block for up to CAD_TIMEOUT seconds
    if (!rf95_manager.sendtoWait(data, size, to)) {
        status |= RFM95_SEND_ERROR;
    }

    // This is not needed if the 'TO' address above is a specific node. If
    // RH_BROADCAST_ADDRESS is used, then we should wait
    if (to == RH_BROADCAST_ADDRESS) {
        if (!rf95_manager.waitPacketSent(WAIT_AVAILABLE)) {
            status |= RFM95_SEND_ERROR;
        }
    }
#endif
}

/**
 * Read a message. Each time this is called, the static storage used to hold
 * the message is cleared. This function waits for 5s (see WAIT_AVAILABLE)
 * for a message to appear.
 *
 * If no message is received, the global 'status' is set with the code
 * RFM95_NO_REPLY.
 *
 * @return The message, held in static storage. Returns nullptr if no message
 * was received.
 */
uint8_t *receive_message() {
#if LORA
    // Used to hold any reply from the main node
    static uint8_t rf95_buf[RH_RF95_MAX_MESSAGE_LEN];
    memset(rf95_buf, 0, sizeof(rf95_buf));

    // Now wait for a reply
    uint8_t len = sizeof(rf95_buf);
    uint8_t from;

    // Should be a reply message for us now
    if (rf95_manager.waitAvailableTimeout(WAIT_AVAILABLE)) {
        if (rf95_manager.recvfromAck(rf95_buf, &len, &from)) {
            return rf95_buf;
        } else {
            IO_LOG(debug_log("Message available, but ack failure."));
            status |= RFM95_NO_REPLY;
        }
    } else {
        IO_LOG(debug_log("No message available."));
        status |= RFM95_NO_REPLY;
    }
#endif

    return nullptr;
}

/**
 * Update the node's time using 'main_node_time' if the difference between
 * the two times is greater than one second.
 *
 * @param main_node_time The unix time from the main node
 * @return True if the time was updated, false if not.
 */
bool update_time(uint32_t main_node_time) {
    int32_t delta_time = main_node_time - rtc.getEpoch();

    IO_LOG(
        char msg[128];
        snprintf(msg, sizeof(msg), "Time from main node: %lu, Time from this node: %lu, Delta: %ld",
                 (unsigned long)main_node_time, (unsigned long)rtc.getEpoch(), (long)delta_time);
        debug_log(msg));

    // update the time if the delta is more than a second
    if (abs(delta_time) > 1) {
        rtc.setEpoch(main_node_time);
        return true;
    }

    return false;
}

/**
 * @brief RMF95 sleep mode. Any API call wakes the RMF95 up.
 */
void radio_silence() {
#if LORA
    rf95.sleep(); // Turn off the LoRa
#endif
}

/**
 * @brief Get the temperature from the SHT-30D
 * @note If the SHT30D didn't initialize correctly, this will return zero.
 * @return the temperature * 100 as a 16-bit signed int
 */
int16_t get_temperature() {
#if SHT30D
    return (int16_t)(sht30d.readTemperature() * 100);
#else
    return 0;
#endif
}

/**
 * @brief Get the humidity from the SHT-30-D
 * @return the humidity * 100 as a 16-bit unsigned int
 */
uint16_t get_humidity() {
#if SHT30D
    return (uint16_t)(sht30d.readHumidity() * 100);
#else
    return 0;
#endif
}

/**
 * Enter the sleep mode. Wake up after an interrupt. This
 * handles shutting down the peripherals and puts the RS to
 * sleep with an interrupt handler set to trigger wakeup.
 *
 * The wake-up time is a recurring RTC alarm rather than an absolute
 * epoch: once per hour, at WAKE_UP_MINUTE:WAKE_UP_SECOND, or - when
 * SAMPLE_ONCE_PER_MINUTE is set - once per minute at SAMPLE_SECOND,
 * for testing. See the definitions of those macros above.
 */

void sleep_node() {
#if TX_LED
    digitalWrite(STATUS_LED, LOW);
#endif

#if LORA
    // low-power configuration. There is no corresponding wake up
    // function since the first use of the LoRa module cancels
    // its sleep mode.
    radio_silence();
    IO_LOG(debug_log("Radio silence"));
#endif

    // Log once, before the SD card is powered off: debug_log() needs the card
    // up (and, below, SPI active) to write to it. shutdown_sd_card() cuts SD
    // power internally (after its own settle wait) and SPI.end() drops the
    // bus, so neither a "SD shutdown" nor a "SPI shutdown" message logged
    // *after* those calls could actually reach the debug log - there is no
    // safe point between them to split this into two messages. jhrg 9/21/26
#if SD || SPI_SLEEP
    IO_LOG(debug_log("SD/SPI shutdown"));
#endif

#if SD
    shutdown_sd_card();
#endif

#if SPI_SLEEP
    SPI.end();
#endif

#if STANDBY_MODE
#if SAMPLE_ONCE_PER_MINUTE
    rtc.setAlarmSeconds(SAMPLE_SECOND);
    rtc.enableAlarm(rtc.MATCH_SS);
#else
    rtc.setAlarmTime(0, WAKE_UP_MINUTE, WAKE_UP_SECOND);
    rtc.enableAlarm(rtc.MATCH_MMSS);
#endif
    rtc.attachInterrupt(alarmMatch);
    // 10ms wait here. jhrg 12/5/20
    yield(10);
    // At this point the node will enter sleep and wake up when the alarm is triggered.
    // Execution resumes in alarmMatch() and then the line following the standbyMode()
    // call.
    rtc.standbyMode();
#else
    yield(STANDBY_INTERVAL_S * 1000);
#endif

#if SPI_SLEEP
    SPI.begin();
#endif

#if SD
    wake_up_sd_card();
#endif

    // Symmetric with the shutdown message above: only safe to log once SPI is
    // back and wake_up_sd_card() has re-initialized the card. jhrg 9/21/26
#if SD || SPI_SLEEP
    IO_LOG(debug_log("SPI/SD up"));
#endif

#if TX_LED
    digitalWrite(STATUS_LED, HIGH);
#endif
}

/// Debugging output - set the state of unused GPIOs to track progress
void init_state_pins() {
    pinMode(STATE_1, OUTPUT);
    pinMode(STATE_2, OUTPUT);
    pinMode(STATE_3, OUTPUT);
    pinMode(STATE_4, OUTPUT);
    pinMode(STATE_5, OUTPUT);
}

void clear_state_pins() {
    digitalWrite(STATE_1, LOW);
    digitalWrite(STATE_2, LOW);
    digitalWrite(STATE_3, LOW);
    digitalWrite(STATE_4, LOW);
    digitalWrite(STATE_5, LOW);
}

void set_state_pin(unsigned int pin) {
    digitalWrite(pin, HIGH);
}

void setup() {
    // Blanket pin mode settings
    // Switch unused pins as input and enabled built-in pullup
    for (unsigned int pinNumber = 0; pinNumber < 23; pinNumber++) {
        pinMode(pinNumber, INPUT_PULLUP);
    }

    for (unsigned int pinNumber = 32; pinNumber < 42; pinNumber++) {
        pinMode(pinNumber, INPUT_PULLUP);
    }

    pinMode(25, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    // Debugging pins initialized
    init_state_pins();
    clear_state_pins();

    // Configure the ADC
    get_battery_voltage_setup();

    // RocketScream's built-in flash not used
    SerialFlash.begin(FLASH_CS);
    SerialFlash.sleep();
    // digitalWrite(FLASH_CS, HIGH); // deselect

    // SD card power control (low-side switching)
    pinMode(SD_PWR, OUTPUT);
    digitalWrite(SD_PWR, HIGH); // Power on the card

    // SPI bus control. Deselect both devices explicitly rather than relying
    // on the DOUT bit carried over from the INPUT_PULLUP pass above - each
    // device's own driver (RadioHead, SdFat) only asserts its CS for the
    // duration of its own transactions, so nothing else keeps these pins
    // deselected between transactions. jhrg 9/14/26
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    pinMode(RFM95_CS, OUTPUT);
    digitalWrite(RFM95_CS, HIGH);

    // Only start the Serial interface when DEBUG is 1
    IO(Serial.begin(115200));
    int tries = 0;
    // Wait for serial port to be available
    IO(while ((tries < SERIAL_CONNECT_TRIES) && !Serial) { yield(SERIAL_CONNECT_INTERVAL); ++tries; });

    IO(Serial.println(F("Start LoRa Client")));

    // Initialize the RTC

    rtc.begin(/*reset*/ true);
    rtc.setEpoch(get_epoch(__DATE__, __TIME__));

    IO(
        Serial.print(F("Initial Date, Time: "));
        Serial.print(__DATE__);
        Serial.print(F(", "));
        Serial.println(__TIME__);
        char date_str[32] = {0};
        snprintf(date_str, sizeof(date_str), "%d/%d/%dT%d:%d:%d", rtc.getMonth(), rtc.getDay(), rtc.getYear(),
                 rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
        Serial.print(F("RTC: "));
        Serial.println((const char *)date_str));

    // Initialize the temp/humidity sensor
#if SHT30D
    IO(Serial.print(F("Initializing SHT30D...")));

    if (!sht30d.begin(0x44)) { // Set to 0x45 for alternate i2c addr
        IO(Serial.println(F("Couldn't find SHT30D")));
        blink(STATUS_LED, SHT31_BEGIN_FAIL, ERROR_TIMES);
        digitalWrite(STATUS_LED, HIGH);
        status |= SHT_31_INIT_ERROR;
    }

    // The SHT30D temp/humidity sensor has a heater; turn it off
    sht30d.heater(false);

    IO(Serial.println(F(" Done.")));
#endif // SHT30D

    // Not disabling interrupts here since the RFM 95 is not yet running

#if SD
    // Initialize the SD card
    IO(Serial.print(F("Initializing SD card... ")));
    yield(SD_POWER_ON_DELAY);

    if (!sd.begin(SD_CS)) {
        IO(Serial.println(F("Couldn't init the SD Card")));
        blink(STATUS_LED, SD_BEGIN_FAIL, ERROR_TIMES);
        digitalWrite(STATUS_LED, HIGH);
        status |= SD_CARD_INIT_ERROR;
    } else {
        const char *file_name = get_new_log_filename();
        IO(Serial.print(file_name));

        // Write data header. This will call error_blink() if it fails.
        write_header(file_name);
    }

    IO(Serial.println(F(" Done.")));
#endif

#if LORA
    IO(Serial.print(F("Initializing LORA...")));

    if (!rf95_manager.init()) {
        IO(Serial.println(F("LoRa init failed.")));
        // SD is confirmed working by this point in setup(), unlike SD_BEGIN_FAIL. jhrg 9/21/26
        IO_LOG(debug_log("LoRa init failed."));
        error_blink(STATUS_LED, RFM95_INIT_FAIL);
        digitalWrite(STATUS_LED, HIGH);
        status |= RFM95_INIT_ERROR;
    }

    rf95_manager.setRetries(2); // default is 3
    // the value based on the ACK time (6 Octets == 327ms given SF 10, CR 5, BW 125kHz)
    rf95_manager.setTimeout(400);

    // Setup ISM frequency
    if (!rf95.setFrequency(FREQUENCY)) {
        IO(Serial.println(F("LoRa frequency out of range.")));
        IO_LOG(debug_log("LoRa frequency out of range."));
        blink(STATUS_LED, RFM95_SET_FREQ_FAIL, ERROR_TIMES);
        digitalWrite(STATUS_LED, HIGH);
        status |= RFM95_INIT_ERROR;
    }

    // Setup Power,dBm
    rf95.setTxPower(tx_power);
    // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
    // Lower BandWidth for longer distance.
    rf95.setSignalBandwidth(BANDWIDTH);
    // Setup Spreading Factor (6 ~ 12)
    rf95.setSpreadingFactor(SPREADING_FACTOR);
    // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8)
    rf95.setCodingRate4(CODING_RATE);
    // 10 seconds
    rf95.setCADTimeout(RH_CAD_DEFAULT_TIMEOUT);

    IO(Serial.println(F(" Done.")));
#endif // LORA

    // Because the RS Ultra Pro board's native USB won't work with the standby() mode
    // in the LowPower or RTCZero libraries, the MCU board can easily wind up bricked
    // when using standby(). It will then become impossible to upload new/fixed
    // code. Add a 10s delay here so a coordinated reset/upload will work.
    //
    // 'tries' is the number of times the code tries to init the USB serial object.
    yield(max(0, BOOT_SAFETY_DELAY - tries * SERIAL_CONNECT_INTERVAL));

#if STANDBY_MODE  // !DEBUG jhrg 6/17/23
    // Once past setup(), the USB cannot be used unless DEBUG is on. Then it must
    // be toggled during the sleep period.
    // NB: I cannot get the SerialUSB class to work after the RS has woken from its
    // StandbyMode.
    USBDevice.detach();
#endif

#if LORA
    // send time request
    time_request_t request;
    build_time_request(&request, NODE_ADDRESS);
    send_message((uint8_t *)&request, RH_BROADCAST_ADDRESS, TIME_REQUEST_SIZE);

    // get the time response
    uint8_t *response = receive_message();
    MessageType mt = get_message_type(response);
    switch (mt) {
        case time_response: {
            uint8_t node;
            uint32_t time;
            parse_time_response((time_response_t *)response, &node, &time);

            update_time(time);
            break;
        }

        default: {
            IO(Serial.print(F("Unexpected response type: ")));
            IO(Serial.println(get_message_type_string(mt)));
            status |= RFM95_NO_REPLY;  // We're pretty lean on codes...
        }
    }
#endif

    IO(Serial.println(F("Setup complete.")));
}

void loop() {
    static unsigned long last_tx_time = 0;
    static unsigned long message = 0;

    // The data sent to the main node
    // packet_t data;

    data_message_t data;

    unsigned long sample_time = rtc.getEpoch();

    ++message;

    build_data_message(&data, NODE_ADDRESS, message, sample_time, (uint16_t)get_battery_voltage(), (uint16_t)last_tx_time,
                       get_temperature(), get_humidity(), status);

    clear_state_pins();
    set_state_pin(STATE_1);
    IO_LOG(debug_log("STATE 1"));

    // Preserve the 4 high bits of the status byte - the initialization errors.
    status = status & 0xF0; // clear status low nyble for the next sample interval

#if LORA
    last_tx_time = millis();
    send_message((uint8_t *)&data, RH_BROADCAST_ADDRESS, DATA_MESSAGE_SIZE);
    last_tx_time = millis() - last_tx_time;
#endif

    IO_LOG(
        char msg[300];
        snprintf(msg, sizeof(msg), "Data msg: %s", data_message_to_string((data_message_t *)&data, true));
        debug_log(msg));

    set_state_pin(STATE_2);
    IO_LOG(debug_log("STATE 2"));

    log_data(get_log_filename(), data_message_to_string(&data, false));

    if (message % TIME_REQUEST_SAMPLE_PERIOD == 0) {
        set_state_pin(STATE_3);
        IO_LOG(debug_log("STATE 3"));
#if LORA
        // send time request
        time_request_t request;
        build_time_request(&request, NODE_ADDRESS);
        send_message((uint8_t *)&request, RH_BROADCAST_ADDRESS, TIME_REQUEST_SIZE);

        IO_LOG(debug_log("Sent time request..."));

        // get the time response
        uint8_t *response = receive_message();
        MessageType mt = get_message_type(response);
        if (mt == time_response) {
            uint8_t node;
            uint32_t time;
            parse_time_response((time_response_t *)response, &node, &time);

            IO_LOG(debug_log(time_response_to_string((time_response_t *)response, true)));

            if (update_time(time)) {
                sample_time = time;  // for the elapsed-time debug output below
            }
        } else {
            IO_LOG(debug_log("No response."));
            status |= RFM95_NO_REPLY;  // We're pretty lean on codes...
        }
#endif
    }

    set_state_pin(STATE_4);
    IO_LOG(debug_log("STATE 4"));

    sleep_node();

    set_state_pin(STATE_5);
    IO_LOG(debug_log("STATE 5"));

    // Elapsed-wake-time diagnostic - moved off LoRa (FR-010 is for errors, not
    // this) and onto the debug log (FR-009) where it belongs. jhrg 9/21/26
    IO_LOG(
        char msg[64];
        snprintf(msg, sizeof(msg), "t: %ld, o: %d", (unsigned long)rtc.getEpoch() - sample_time, STANDBY_INTERVAL_S);
        debug_log(msg));
}
