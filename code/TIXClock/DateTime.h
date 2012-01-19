#include <Arduino.h>
#include <avr/pgmspace.h>

#ifndef SURREALITYLABS_DATETIME
#define SURREALITYLABS_DATETIME

#define SECONDS_PER_DAY 86400L
#define SECONDS_FROM_1970_TO_2000 946684800

// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
public:
    DateTime (uint32_t t =0);
    DateTime (uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
    DateTime (const char* date, const char* time);
    uint16_t year() const       { return 2000 + yOff; }
    void setYear(uint16_t val)    { yOff = val - 2000; }
    uint8_t month() const       { return m; }
    void setMonth(uint8_t val)       { m = val; }
    uint8_t day() const         { return d; }
    void setDay(uint8_t val)         { d = val; }
    uint8_t hour() const        { return hh; }
    void setHour(uint8_t val)        { hh = val; }
    uint8_t minute() const      { return mm; }
    void setMinute(uint8_t val)      { mm = val; }
    void setSecond(uint8_t val)      { ss = val; }
    uint8_t second() const      { return ss; }
    uint8_t dayOfWeek() const;

    // 32-bit times as seconds since 1/1/2000
    long secondstime() const;   
    // 32-bit times as seconds since 1/1/1970
    uint32_t unixtime(void) const;

protected:
    uint8_t yOff, m, d, hh, mm, ss;
};

#endif
