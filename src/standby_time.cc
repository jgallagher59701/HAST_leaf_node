//
// Functions to set the standby times. 
//
// James Gallagher <jgallagher@opendap.org>
// 10/11/21

#include <Arduino.h>

#include <RTCZero.h>

#include "standby_time.h"

#define STANDBY_MODE 1 // Use RTC standby mode and not delay()

// Defined in platformio.ini
#ifndef NODE_ADDRESS
#define NODE_ADDRESS 10
#endif

#define SECONDS_PER_NODE 4 // s, Each leaf node is given this time window

/** 
 * @brief Call back for the sleep alarm
 */
void alarmMatch() {
    rtc.detachInterrupt();
}

uint8_t sample_intervals[60] = {0};

/**
 * @brief Compute once the sample times for a given interval of minutee per sample
 */
void compute_samples()
{

}

/**
 * @brief Set the standby time based on clock time. Calls stadnbyMode().
 * 
 * @todo fix this so it uses wall time. Now it is a duration. Maybe most
 * of the computation can be done by the compiler?
 * 
 * @param rtc The real time clode object.
 * @param alarm_minute_interval The minute marker for the alarm.
 */
void standby_minutes_interval(RTCZero rtc, uint8_t alarm_minute_interval)
{
    // Get teh current time to make sure the alarm iis in the future.
    uint8_t mm = rtc.getMinutes();
    uint8_t ss = rtc.getSeconds();

    // The alarm time is fixed at 'alarm_minute_interval' points around the
    // clock, with the added offset based on node number to minimze transmission
    // collisions.
    const uint8_t ss_offset = (NODE_ADDRESS * SECONDS_PER_NODE) % 60;
    const uint8_t mm_offset = ss_offset / 60;

    // 23 + 5 --> 28; 28 / 5 = 5; 5 * 5 = 25
    // 56 + 5 --> 61; 60  / 5 = 12; 12 * 5 = 60; mm_offset = 1, mm_alarm = 61, mm_alarm = 1
    // 0 + 5 --> 5; 5 / 5 = 1; 1 * 5 = 5
    uint8_t next_minute_mark = ((mm + alarm_minute_interval) / 5) * alarm_minute_interval;
    uint8_t mm_alarm = next_minute_mark + mm_offset;
    mm_alarm = mm_alarm % 60;

    rtc.setAlarmMinutes(mm_alarm);
    rtc.setAlarmSeconds(ss_offset);

    rtc.enableAlarm(rtc.MATCH_MMSS);

    rtc.attachInterrupt(alarmMatch);

    delay(10);  // 10ms wait here. jhrg 12/5/20
    // At this point the node will enter sleep and wake up when the alarm is triggered.
    // Execution resumes in alarmMatch() and then the line following the standbyMode()
    // call.
    rtc.standbyMode();
}
