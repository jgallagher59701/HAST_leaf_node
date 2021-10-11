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

// Exclude some parts of the code for debugging. Zero excludes the code.
#define DEBUG 0      // Requires USB; Will not work with STANDBY_MODE
#define LORA_DEBUG 0 // Send debugging info to the main node using lora
#include "debug.h"

#define MAIN_NODE_ADDRESS 0

// Define in the platformio file. jhrg 7/31/21
#ifndef NODE_ADDRESS
#define NODE_ADDRESS 4
#endif

#define Serial SerialUSB // Needed for RS. jhrg 7/26/20
#define SERIAL_CONNECT_TRIES 10
#define SERIAL_CONNECT_INTERVAL 1000 // ms

#define STANDBY_MODE 1 // Use RTC standby mode and not yield()

#if DEBUG && STANDBY_MODE
#undef STANDBY_MODE
#define STANDBY_MODE 0
#endif

// Set to 1 to use delay(), zero to use yield(). There are problems
// debugging the RocketScream using yield().
#define USE_DELAY 1

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

// Will the main node reply to the data packet broadcast?
#define EXPECT_REPLY 1

#ifndef STANDBY_INTERVAL_S
#define STANDBY_INTERVAL_S 300 // seconds to wait/sleep before next transmission
#endif

#define BOOT_SAFTEY_DELAY 10000 // 10s

#define WAIT_AVAILABLE 5000   // ms to wait for reply from main node
#define SD_POWER_ON_DELAY 200 // ms

// Log file name.
#define FILE_BASE_NAME "Data"
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

// TODO: Are these functions that set the SPI bus CS lines HIGH needed?

/**
    @brief RF95 off the SPI bus to enable SD card access
*/
void yield_spi_to_sd() {
    digitalWrite(RFM95_CS, HIGH);
    // digitalWrite(FLASH_CS, HIGH);
    // Setting the SD card SS LOW seems to break things. Let the
    // SdFat library code control when to set SS to LOW.
}

/**
    @brief SD card off the SPI bus to enable RFM95 access
*/
void yield_spi_to_rf95() {
    digitalWrite(SD_CS, HIGH);
    // digitalWrite(FLASH_CS, HIGH);
}

/**
 * @brief Remove everything from the SPI bus
 */
void yield_spi_bus() {
    digitalWrite(SD_CS, HIGH);
    digitalWrite(RFM95_CS, HIGH);
    // FLASH_CS is always off the bus digitalWrite(FLASH_CS, HIGH);
}

/**
 * @brief Send a short message for debugging using the LoRa
 * @param msg The message; null terminated string
 * @param to Send to this node
 */
void lora_debug(const char *msg, uint8_t to) {
#if LORA && LORA_DEBUG
    yield_spi_to_rf95();
    rf95_manager.sendtoWait((uint8_t *)msg, strlen(msg) + 1, to);
#endif
}

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
 * the radiohead library (RFM95) has been initialized.
 * 
 * @return A pointer to the new file name. Global static storage.
 * @see get_log_filename()
 */
const char *
get_new_log_filename() {
    // If the SD card/library failed to init, don't run this code.
    if (status & SD_CARD_INIT_ERROR) {
        return nullptr;
    }

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
   @note Claim the SPI bus
   @note Never call this if the SD card initialization fails.
*/
void write_header(const char *file_name) {
#if SD
    if (status & SD_CARD_INIT_ERROR) {
        return;
    }

    yield_spi_to_sd();

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
   @note Claim the SPI bus (calls yield_spi_to_sd).
*/
void log_data(const char *file_name, const char *data) {
#if SD
    if (status & SD_CARD_INIT_ERROR) {
        return;
    }

    yield_spi_to_sd();

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

        lora_debug(error_info, MAIN_NODE_ADDRESS);
    }

    // enable interrupts
    interrupts();
#endif
}

/**
 * @brief Shutdown the SD card by cutting power
 * 
 * Wait for SD_CARD_WAIT seconds before cutting the power.
 */
void shutdown_sd_card() {
    // FIXME only do this if the card started. jhrg 9/26/21
    if (status & SD_CARD_INIT_ERROR) {
        return;
    }

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
    if (status & SD_CARD_INIT_ERROR) {
        return;
    }

    yield_spi_to_sd();

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

/**
 * @brief Send a data packet.
 * 
 * Send the packet to a node and wait for a reply from that node. If
 * the 'to' address is RH_BROADCAST_ADDRESS, wait until the packet is
 * sent (but not for an ACK).
 * 
 * If an error is detected, set 'status.'
 * 
 * @param data The data packet to send
 * @param to Send to this node. If RH_BROADCAST_ADDRESS, send to all nodes.
 */
void send_data_packet(packet_t &data, uint8_t to) {
#if LORA
    yield_spi_to_rf95();

    // This may block for up to CAD_TIMEOUT seconds
    if (!rf95_manager.sendtoWait((uint8_t *)&data, DATA_PACKET_SIZE, to)) {
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
 * @brief Read the time time code reply from the main node
 * 
 * Once a packet is sent to the main node, expect a reply (even
 * when broadcasting the packet). Read the time code and update
 * the local node's RTC.
 *
 * This must be called within 5000ms of the expected reply.
 *
 * @todo This needs to be changed for configurations where nodes'
 * communication with the main node might overlap.
 */
void read_main_node_reply() {
#if LORA
    yield_spi_to_rf95();

    // Used to hold any reply from the main node
    uint8_t rf95_buf[RH_RF95_MAX_MESSAGE_LEN];

    // Now wait for a reply
    uint8_t len = sizeof(rf95_buf);
    uint8_t from;

    // Should be a reply message for us now
    if (rf95_manager.waitAvailableTimeout(WAIT_AVAILABLE)) {
        if (rf95_manager.recvfromAck(rf95_buf, &len, &from)) {
            uint32_t main_node_time = 0;
            if (len == sizeof(uint32_t)) { // time code?
                memcpy(&main_node_time, rf95_buf, sizeof(uint32_t));
                // cast in abs() needed to resolve ambiguity
                uint32_t delta_time = main_node_time - rtc.getEpoch();
                // update the time if the delta is more than a second
                if (abs(long(delta_time)) > 1) {
                    rtc.setEpoch(main_node_time);
                }
            }
        } else {
            status |= RFM95_NO_REPLY;
        }
    } else {
        status |= RFM95_NO_REPLY;
    }
#endif
}

/**
 * @brief RMF95 sleep mode. Any API call wakes the RMF95 up.
 */
void radio_silence() {
#if LORA
    yield_spi_to_rf95();
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
 * The sleep time period is set by the compile-time constant
 * STANDBY_INTERVAL_S, which due to a time rounding error, is
 * one less than the number of seconds the node will actually
 * sleep.
 *
 * @param sample_time Unix time the last sample was taken.
 */

void sleep_node(unsigned long sample_time) {
#if TX_LED
    digitalWrite(STATUS_LED, LOW);
#endif

#if LORA
    // low-power configuration. There is no corresponding wake up
    // function since the first use of the LoRa module cancels
    // its sleep mode.
    radio_silence();
    IO(Serial.println("Radio silence"));
#endif

#if SD
    shutdown_sd_card();
    IO(Serial.println("SD shutdown"));
#endif

#if SPI_SLEEP
    SPI.end();
    IO(Serial.println("SPI shutdown"));
#endif

    // TODO Fix this so that the times are on the hour.
    //  Use setAlaramTime(h, m, s) and rtc.MATCH_MMSS for every hour or MATCH_SS for
    //  every minute. Update the m and s values using 'time + n mod 60'.
    //  https://www.arduino.cc/en/Reference/RTC

#if STANDBY_MODE
    unsigned long wake_up_time = sample_time + STANDBY_INTERVAL_S;
    if (wake_up_time < rtc.getEpoch() + 2)
        wake_up_time = rtc.getEpoch() + 2;
    rtc.setAlarmEpoch(wake_up_time);
    rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
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
    IO(Serial.println("SPI up"));
#endif

#if SD
    wake_up_sd_card();
    IO(Serial.println("SD card up"));
#endif

#if TX_LED
    digitalWrite(STATUS_LED, HIGH);
#endif
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

    // Configure the ADC
    get_battery_voltage_setup();

    // RocketScream's built-in flash not used
    SerialFlash.begin(FLASH_CS);
    SerialFlash.sleep();
    // digitalWrite(FLASH_CS, HIGH); // deselect

    // SD card power control (low-side switching)
    pinMode(SD_PWR, OUTPUT);
    digitalWrite(SD_PWR, HIGH); // Power on the card

    // SPI bus control
    pinMode(SD_CS, OUTPUT);
    pinMode(RFM95_CS, OUTPUT);

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
        Serial.print(F("Date, Time: "));
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
    if (!sht30d.begin(0x44)) { // Set to 0x45 for alternate i2c addr
        IO(Serial.println(F("Couldn't find SHT30D")));
        blink(STATUS_LED, SHT31_BEGIN_FAIL, ERROR_TIMES);
        digitalWrite(STATUS_LED, HIGH);
        status |= SHT_31_INIT_ERROR;
    }

    // The SHT30D temp/humidity sensor has a heater; turn it off
    sht30d.heater(false);
#else
    status |= SHT_31_INIT_ERROR;
#endif // SHT30D

    // Not disabling interrupts here since the RFM 95 is not yet running

#if SD
    // Initialize the SD card
    yield_spi_to_sd();

    IO(Serial.print(F("Initializing SD card...")));
    yield(SD_POWER_ON_DELAY);

    if (!sd.begin(SD_CS)) {
        IO(Serial.println(F("Couldn't init the SD Card")));
        blink(STATUS_LED, SD_BEGIN_FAIL, ERROR_TIMES);
        digitalWrite(STATUS_LED, HIGH);
        status |= SD_CARD_INIT_ERROR;
    } else {
        const char *file_name = get_new_log_filename();
        IO(Serial.println(file_name));

        // Write data header. This will call error_blink() if it fails.
        write_header(file_name);
    }
#else
    status |= SD_CARD_INIT_ERROR;
#endif

#if LORA
    yield_spi_to_rf95();

    if (!rf95_manager.init()) {
        IO(Serial.println(F("LoRa init failed.")));
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
#else
    status |= RFM95_INIT_ERROR;
#endif // LORA

    // Because the RS Ultra Pro board's native USB won't work with the standby() mode
    // in the LowPower or RTCZero libraries, the MCU board can easily wind up bricked
    // when using standby(). It will then become impossible to upload new/fixed
    // code. Add a 10s delay here so a coordinated reset/upload will work.
    //
    // 'tries' is the number of times the code tries to init the USB serial object.
    yield(max(0, BOOT_SAFTEY_DELAY - tries * SERIAL_CONNECT_INTERVAL));

#if !DEBUG
    // Once past setup(), the USB cannot be used unless DEBUG is on. Then it must
    // be toggled during the sleep period.
    // NB: I cnnot get the SerialUSB class to work after the RS has woken from its
    // StandbyMode.
    USBDevice.detach();
#endif

    // Exit setup() with the status LED lit.
}

void loop() {
    static unsigned long last_tx_time = 0;
    static unsigned long message = 0;

    // The data sent to the main node
    packet_t data;

    unsigned long sample_time = rtc.getEpoch();

    ++message;

// New packet encoding.
// TODO Could drop NODE_ADDRESS and status if using RH Datagrams.
#if 0
    build_data_packet(&data, NODE_ADDRESS, message, sample_time, get_bat_v(), (uint16_t)last_tx_time,
                      get_temperature(), get_humidity(), status);
#else
    build_data_packet(&data, NODE_ADDRESS, message, sample_time, get_battery_voltage(), (uint16_t)last_tx_time,
                      get_temperature(), get_humidity(), status);
#endif

    // Preserve the 4 high bits of the status byte - the initialization errors.
    status = status & 0xF0; // clear status low nyble for the next sample interval

#if LORA
    last_tx_time = millis();
    send_data_packet(data, RH_BROADCAST_ADDRESS);
    last_tx_time = millis() - last_tx_time;

    read_main_node_reply();
#endif

    log_data(get_log_filename(), data_packet_to_string(&data, false));

    sleep_node(sample_time);

#if LORA_DEBUG
    char msg[256];
    snprintf(msg, 256, "t: %ld, o: %d \n", (unsigned long)rtc.getEpoch() - sample_time, STANDBY_INTERVAL_S);
    lora_debug(msg, MAIN_NODE_ADDRESS);
#endif
}
