// Pongar config

#include "fraise.h"
#include "fraise_eeprom.h"
#include "config.h"

Config config;

void Config::eeprom_declare() {
    eeprom_declare_data((char*)this, sizeof(Config));
}

void Config::receivebytes(const char* data, uint8_t len) {
    char command = fraise_get_uint8();
    switch(command) {
    case 1: 
        dmx_start = fraise_get_uint16();
        dmx_set_start = fraise_get_uint16();
        dmx_set_unlock = fraise_get_uint16();
        break;
    case 101:
        fraise_put_init();
        fraise_put_uint8(200);
        fraise_put_uint16(dmx_start);
        fraise_put_uint16(dmx_set_start);
        fraise_put_uint16(dmx_set_unlock);
        fraise_put_send();
        break;
    }
}

