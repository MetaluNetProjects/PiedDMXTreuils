// Pongar config

#pragma once

class Config {
public:
    void eeprom_declare();
    void receivebytes(const char* data, uint8_t len);
    // data:
    uint16_t dmx_start = 1;
    uint16_t dmx_set_start = 16;
    uint16_t dmx_set_unlock = 24;
};

extern Config config;

