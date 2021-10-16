
#ifndef standby_time_h
#define standby_time_h

// #include <Arduino.h>
//#include <RTCZero.h>

extern uint8_t next_sample_time[];

bool compute_samples(uint8_t minutes_interval);
//void standby_minutes_interval(RTCZero rtc, uint8_t alarm_minute_interval);
//void standby_hour_interval(RTCZero rtc, uint8_t alarm_minute_interval);

#endif
