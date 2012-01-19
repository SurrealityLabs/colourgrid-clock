#include <Arduino.h>
#include "DateTime.h"
#include "RTC.h"
#include "Wire.h"

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

uint8_t RTC_MCP794XX::begin(void) {
    return 1;
}

uint8_t RTC_MCP794XX::isrunning(void) {
    Wire.beginTransmission(MCP794XX_ADDRESS);
    Wire.write(byte(3));	
    Wire.endTransmission();

    Wire.requestFrom(MCP794XX_ADDRESS, 1);
    uint8_t ss = Wire.read();
    return (ss & 0x20);
}

void RTC_MCP794XX::adjust(const DateTime& dt) {
    Wire.beginTransmission(MCP794XX_ADDRESS);
    Wire.write(byte(0));
    Wire.write(bin2bcd(dt.second()) | 0x80);
    Wire.write(bin2bcd(dt.minute()));
    Wire.write(bin2bcd(dt.hour()));
    Wire.write(bin2bcd(0x08));
    Wire.write(bin2bcd(dt.day()));
    Wire.write(bin2bcd(dt.month()));
    Wire.write(bin2bcd(dt.year() - 2000));
    Wire.write(byte(0));
    Wire.endTransmission();
}

DateTime RTC_MCP794XX::now() {
    Wire.beginTransmission(MCP794XX_ADDRESS);
    Wire.write(byte(0));	
    Wire.endTransmission();
  
    Wire.requestFrom(MCP794XX_ADDRESS, 7);
    uint8_t ss = bcd2bin(Wire.read() & 0x7F);
    uint8_t mm = bcd2bin(Wire.read());
    uint8_t hh = bcd2bin(Wire.read());
    Wire.read();
    uint8_t d = bcd2bin(Wire.read());
    uint8_t m = bcd2bin(Wire.read() & 0x1F);
    uint16_t y = bcd2bin(Wire.read()) + 2000;
  
    return DateTime (y, m, d, hh, mm, ss);
}

uint8_t RTC_DS1307::begin(void) {
    return 1;
}

uint8_t RTC_DS1307::isrunning(void) {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(byte(0));	
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, 1);
    uint8_t ss = Wire.read();
    return !(ss >> 7);
}

void RTC_DS1307::adjust(const DateTime& dt) {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(byte(0));
    Wire.write(bin2bcd(dt.second()));
    Wire.write(bin2bcd(dt.minute()));
    Wire.write(bin2bcd(dt.hour()));
    Wire.write(bin2bcd(0));
    Wire.write(bin2bcd(dt.day()));
    Wire.write(bin2bcd(dt.month()));
    Wire.write(bin2bcd(dt.year() - 2000));
    Wire.write(byte(0));
    Wire.endTransmission();
}

DateTime RTC_DS1307::now() {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(byte(0));	
    Wire.endTransmission();
  
    Wire.requestFrom(DS1307_ADDRESS, 7);
    uint8_t ss = bcd2bin(Wire.read() & 0x7F);
    uint8_t mm = bcd2bin(Wire.read());
    uint8_t hh = bcd2bin(Wire.read());
    Wire.read();
    uint8_t d = bcd2bin(Wire.read());
    uint8_t m = bcd2bin(Wire.read());
    uint16_t y = bcd2bin(Wire.read()) + 2000;
  
    return DateTime (y, m, d, hh, mm, ss);
}
