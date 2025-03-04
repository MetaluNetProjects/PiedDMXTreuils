// Pongar config

#pragma once

class Config {
public:
    void eeprom_declare();
    void receivebytes(const char* data, uint8_t len);
    // data:
    uint16_t dmx_start = 1;
};

extern Config config;

