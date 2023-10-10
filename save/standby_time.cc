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
#define MINUTES_PER_HOUR 60
#define HOURS_PER_DAY 24

extern RTCZero rtc;

/** 
 * @brief Call back for the sleep alarm
 */
static void alarmMatch() {
    rtc.detachInterrupt();
}

uint8_t next_sample_time[MINUTES_PER_HOUR] = {0};

/**
 * @brief Compute once the sample times for a given interval
 * 
 * Build an array that can be used to look up the the minute
 * for the next sample for a given sample interval. For example,
 * if the sample interval is 5 minutes, the next time to sample
 * for the :00 sample is :05, the next time to sample for the
 * sample taken at :25 is :30 and for :55 it is :00. Because
 * samples are taken at the time + an offset that compensates
 * for the number of nodes, the sample taken at :25 might actually
 * happen at :26 or :27. The next time in each of those cases
 * is still :30 since the code that sets the time will add the
 * offset based on other information (mainly the node number).
 * 
 * @todo This is used when the samples are taken multiple times
 * per hour, not when they are taken once per hour.
 * 
 * @param minutes_interval Minutes between two samples. Should 
 * evenly divide 60
 * 
 * @return true if called with a value interval, false otherwise
 */
bool compute_samples(uint8_t minutes_interval) {
    if (minutes_interval <= 0 || minutes_interval > 60 || (minutes_interval % 60 != 0))
        return false;

    for (uint8_t i = 0; i < MINUTES_PER_HOUR; ++i) {
        next_sample_time[i] = (minutes_interval + ((i / minutes_interval) * minutes_interval)) % MINUTES_PER_HOUR;
    }

    return true;
}

/**
 * @brief Set the minute-mark standby time based on clock time. Calls standbyMode().
 * 
 * This function is used to enter the sleep mode for N minutes at a time. See 
 * standby_hour_interval() for a function that will sleep until the next hour.
 * 
 * @param rtc The real time clock object.
 * @param alarm_minute_interval The minute marker for the alarm.
 */
void standby_minutes_interval(RTCZero rtc, uint8_t alarm_minute_interval) {
    // Get the current time to make sure the alarm iis in the future.
    uint8_t mm = rtc.getMinutes();

    // The alarm time is fixed at 'alarm_minute_interval' points around the
    // clock, with the added offset based on node number to minimize transmission
    // collisions.
    const uint8_t ss_offset = (NODE_ADDRESS * SECONDS_PER_NODE) % 60;
    const uint8_t mm_offset = ss_offset / 60;

    // 23 + 5 --> 28; 28 / 5 = 5; 5 * 5 = 25
    // 56 + 5 --> 61; 60  / 5 = 12; 12 * 5 = 60; mm_offset = 1, mm_alarm = 61, mm_alarm = 1
    // 0 + 5 --> 5; 5 / 5 = 1; 1 * 5 = 5
    uint8_t next_minute_mark = next_sample_time[mm];
    uint8_t mm_alarm = next_minute_mark + mm_offset;

    // FIXME rtc.setAlarmHours((rtc.getHours() + 1) % HOURS_PER_DAY);

    rtc.setAlarmMinutes(mm_alarm);
    rtc.setAlarmSeconds(ss_offset);

    rtc.enableAlarm(rtc.MATCH_MMSS);

    rtc.attachInterrupt(alarmMatch);

    delay(10); // 10ms wait here. jhrg 12/5/20
    // At this point the node will enter sleep and wake up when the alarm is triggered.
    // Execution resumes in alarmMatch() and then the line following the standbyMode()
    // call.
    rtc.standbyMode();
}

/**
 * @brief Set the hourly standby time based on clock time. Calls standbyMode().
 * 
 * This function will set the RocketScream RTC so that the RS will sleep until
 * the next hourly wakeup time. The leaf nodes actual wakeup time is determined
 * by adding the minutes offset for all the nodes (e.g., a set of nodes all begin
 * to wake up at 3 minutes after the top of the hour) and then the offset in 
 * minutes and seconds from that time.
 * 
 * Example: if there are two leaf nodes (#1 and #2) and they wake up at 3 minutes
 * offset, then the actual wakeup time in minutes and seconds is 03m, 1 * 4 = 04s
 * and 03m, 08s (2 * 4). SECONDS_PER_NODE == 4 above. If there are 16 nodes, the 
 * minutes will also increment. The times will be 03m 04s, 03m 08s, ..., 04m 00s, 
 * 04m 04s.
 * 
 * @param rtc The real time clock object.
 * @param alarm_minute Start wakeup this many minutes from the top of the hour.
 * 
 * @todo Write tests for this code
 */
void standby_hour_interval(RTCZero rtc, uint8_t alarm_minute) {
    // FIXME This function should make sure that the alarm will really be set
    // . for the next hour.

    // The alarm time is fixed at 'alarm_minute_interval' points around the
    // clock, with the added offset based on node number to minimize transmission
    // collisions.
    const uint8_t ss_offset = (NODE_ADDRESS * SECONDS_PER_NODE) % 60;
    const uint8_t mm_offset = ss_offset / 60;

    rtc.setAlarmHours((rtc.getHours() + 1) % HOURS_PER_DAY);
    rtc.setAlarmMinutes(alarm_minute + mm_offset);
    rtc.setAlarmSeconds(ss_offset);

    rtc.enableAlarm(rtc.MATCH_HHMMSS);

    rtc.attachInterrupt(alarmMatch);

    delay(10); // 10ms wait here. jhrg 12/5/20
    // At this point the node will enter sleep and wake up when the alarm is triggered.
    // Execution resumes in alarmMatch() and then the line following the standbyMode()
    // call.
    rtc.standbyMode();
}
