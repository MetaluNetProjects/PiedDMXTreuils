// Lidar firmware

#define BOARD pico
#include "fraise.h"
#include "fraise_master.h"
#include "hardware/uart.h"
#include "string.h"
#include "fraise_eeprom.h"
#include "DmxInput.h"
#include "config.h"
#include "hardware/watchdog.h"
#include "hardware/vreg.h"
#include <vector>
#include <math.h>

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
int ledPeriod = 250;

//const uint BUTTON_PIN = 14;

const int DMX_TX_PIN = 0;
const int DMX_RX_PIN = 1;
const int DMX_DRV_PIN = 2;
#define DMX_CHAN_COUNT 512
unsigned char dmxBuf[DMX_CHAN_COUNT];
const int FRUIT_OFFSET = 49;
DmxInput dmxInput;

bool button, button_last;
int button_count;

void send_dest(int mot, int dest) {
    if(mot > 6) mot = 6;
    char buf[7] = {120, 0, 10};
    buf[3] = dest >> 24;
    buf[4] = dest >> 16;
    buf[5] = dest >> 8;
    buf[6] = dest & 255;
    fraise_master_sendbytes(FRUIT_OFFSET + mot, buf, sizeof(buf));
    printf("dest %d %d\n", FRUIT_OFFSET + mot, dest);
}

void set_pos(int mot, int pos) {
    if(mot > 6) mot = 6;
    char buf[10];
    int len = 0;
    buf[len++] = 120;
    buf[len++] = 7;
    buf[len++] = pos >> 24;
    buf[len++] = pos >> 16;
    buf[len++] = pos >> 8;
    buf[len++] = pos & 255;
    fraise_master_sendbytes(FRUIT_OFFSET + mot, buf, len);
    printf("pos %d %d\n", FRUIT_OFFSET + mot, pos);
}

void send_pwm(int mot, int pwm) {
    if(mot > 6) mot = 6;
    if(pwm < -1023) pwm = -1023;
    else if(pwm > 1023) pwm = 1023;

    char buf[4] = {120, 4};
    buf[2] = pwm >> 8;
    buf[3] = pwm & 255;
    fraise_master_sendbytes(FRUIT_OFFSET + mot, buf, sizeof(buf));
    /*printf("dest %d %d: %02X %02X %02X %02X %02X %02X \n", FRUIT_OFFSET + mot, dest,
        buf[0], buf[1], buf[2], buf[3], buf[4]);*/
    printf("pwm %d %d %d %d\n", FRUIT_OFFSET + mot, pwm, buf[2], buf[3]); // -1023 = FC01
}

class Perche {
    private:
    int32_t position = 0;
    int32_t last_pos = 0;
    int32_t destination = 0;
    int16_t speed = 1;
    uint16_t channel;
    uint16_t set_channel;
    std::vector<uint8_t> motors;
    bool running = false;
    bool setting_changed = true;

    public:
    static const int NB_CHANS = 4;
    static const int POSITION_MAX = 6 * 250 / 0.22; // 6 meters, 250 step/tour, 22 cm/tour
    static const int POSITION_MIN = 100;
    static const int SPEED_MAX = 650;//250;
    Perche(std::vector<uint8_t> motors, uint16_t channel, uint16_t set_channel): 
      channel(channel), set_channel(set_channel), motors(motors) {}

    void set_destination(int32_t dest) {
        destination = dest * 256;
        if(destination < 0) destination = 0;
    }

    void set_speed(uint16_t s) {
        speed = s;
        //if(speed == 0) speed = 1;
    }

    void stop() {
        for(auto m: motors) {
            send_pwm(m, 0);
        }
        position = destination;
        running = false;
        printf("channel %d stop\n", channel);
    }

    void reset_positions() {
        for(auto m: motors) {
            send_pwm(m, 0);
            set_pos(m, POSITION_MAX);
        }
        set_destination(POSITION_MAX);
        stop();
        printf("channel %d reset pos %d dest %d\n", channel, position / 256, destination / 256);
    }
    
    void parse_dmx_setting() {
        static const int deadzone = 55;
        //stop();
        for(int i = 0; i < (int)motors.size(); i++) {
            int dmx_rewind      = (int)dmxBuf[config.dmx_set_start + set_channel + i * 2 + 0];
            int dmx_fastfw      = (int)dmxBuf[config.dmx_set_start + set_channel + i * 2 + 1];
            int dz_rewind      = MAX(dmx_rewind - deadzone, 0);
            int dz_fastfw      = MAX(dmx_fastfw - deadzone, 0);
            int pwm = ((dz_fastfw - dz_rewind) * 1023) / (255 - deadzone);
            send_pwm(motors[i], pwm);
            if(dz_rewind > 0 || dz_fastfw > 0) {
                setting_changed = true;
            }
        }
        running = false;
    }

    void parse_dmx_use() {
        int dmx_dest    = dmxBuf[config.dmx_start + channel + 0];
        int dmx_speed   = dmxBuf[config.dmx_start + channel + 1];
        if(setting_changed) {
            if(dmx_dest != 0) return;
            reset_positions();
            setting_changed = false;
        }
        if(!running) {
            if(dmx_dest != 0) return;
            running = true;
        }
        int new_dest = POSITION_MAX - (dmx_dest * (POSITION_MAX - POSITION_MIN)) / 255;
        set_destination(new_dest);
        set_speed((dmx_speed * SPEED_MAX) / 255);
    }

    void update() {
        //parse_dmx();
        if(!running || !speed) return;
        if(position < destination) {
            position += speed;
            if(position > destination) position = destination;
        }
        else if(position > destination) {
            position -= speed;
            if(position < 0) position = 0;
        }
        int pos = position / 256;
        if(pos != last_pos) {
            for(auto m: motors) send_dest(m, pos);
            last_pos = pos;
        }
    }
};

/*Perche pA({0, 1}, 0);
Perche pB({2, 3}, Perche::NB_CHANS);
Perche pC({5}, Perche::NB_CHANS * 2);*/

Perche pA({1, 3}, 0, 0);
Perche pB({2, 4}, 2, 4);
//Perche pC({/*4, */5}, Perche::NB_CHANS * 2);

std::vector<Perche*> perches{&pA, &pB};

void parse_dmx_global() {
    int lock_val = dmxBuf[config.dmx_set_unlock];
    enum State {USE, STOP, SETTING} state;
    static State old_state = USE;    
    if(lock_val < 10) state = USE;
    else if(lock_val > 245) state = SETTING;
    else state = STOP;
    
    switch(state) {
        case USE: for(auto p: perches) p->parse_dmx_use(); break;
        case SETTING: for(auto p: perches) p->parse_dmx_setting(); break;
        default: ;
    }
    if((old_state != state) && (state == STOP)) for(auto p: perches) p->stop();
    old_state = state;
}

bool dmxRcvd = false;

void __isr dmxDataRecevied(DmxInput* instance) {
     // A DMX frame has been received :-)
     // Toggle some LED, depending on which pin the data arrived
     gpio_put(LED_PIN, !gpio_get(LED_PIN));
     dmxRcvd = true;
     //for(auto p: perches) p->parse_dmx();
}

void setup() {
    eeprom_load();
    gpio_init(DMX_DRV_PIN);
    gpio_set_dir(DMX_DRV_PIN, GPIO_OUT);
    gpio_put(DMX_DRV_PIN, 1);
    dmxInput.begin(DMX_RX_PIN, 1 /*start_chan*/, DMX_CHAN_COUNT);
    dmxInput.read_async(dmxBuf, dmxDataRecevied);
    watchdog_enable(1000, 0); // enable watchdog, requiring to update it every second
}

void loop() {
    static absolute_time_t nextLed;
    static absolute_time_t nextMotorUpdate;
    static bool led = false;

    if(time_reached(nextLed)) {
        gpio_put(LED_PIN, led = !led);
        nextLed = make_timeout_time_ms(ledPeriod);
    }

    if(time_reached(nextMotorUpdate)) {
        nextMotorUpdate = make_timeout_time_ms(10);
        for(auto p: perches) p->update();
    }

    if(dmxRcvd) {
        dmxRcvd = false;
        parse_dmx_global();
    }
    watchdog_update();
}

void fraise_receivebytes(const char *data, uint8_t len) {
    uint8_t command = fraise_get_uint8();
    //fraise_printf("l get command %d\n", command);
    switch(command) {
    case 1:
        ledPeriod = (int)fraise_get_uint8() * 10;
        break;
    case 20:
        config.receivebytes(data + 1, len - 1);
        break;
    case 30:
        send_dest(fraise_get_uint8(), fraise_get_uint16());
        break;
    case 31:
        send_pwm(fraise_get_uint8(), fraise_get_int16());
        break;
    case 40: {
            uint8_t perche = fraise_get_uint8();
            if(perche >= perches.size()) perche = perches.size() - 1;
            perches[perche]->set_destination(fraise_get_uint16());
        }
        break;
    case 41: {
            uint8_t perche = fraise_get_uint8();
            if(perche >= perches.size()) perche = perches.size() - 1;
            perches[perche]->set_speed(fraise_get_uint16());
        }
        break;
    case 100: panic("panic!"); break;
    case 120: printf("reset wdt %d por %d\n", watchdog_caused_reboot(), (vreg_and_chip_reset_hw->chip_reset & 256) != 0); break;
    case 121: 
        printf("scratchs %ld %ld %ld %ld\n", watchdog_hw->scratch[0], watchdog_hw->scratch[1], watchdog_hw->scratch[2], watchdog_hw->scratch[3]);
        break;
    case 122: 
        watchdog_hw->scratch[0] = fraise_get_uint16();
        watchdog_hw->scratch[1] = fraise_get_uint16();
        watchdog_hw->scratch[2] = fraise_get_uint16();
        watchdog_hw->scratch[3] = fraise_get_uint16();
        break;
    case 200: {
            printf("dmx %d %d %d %d %d %d\n", dmxBuf[1], dmxBuf[2], dmxBuf[3], dmxBuf[4], dmxBuf[5], dmxBuf[6]);
        }
        break;
    case 203:
        //dmx.reset();
        break;
    case 204:
        //dmx.status();
        break;
    default:
        printf("rcvd ");
        for(int i = 0; i < len; i++) printf("%d ", (uint8_t)data[i]);
        putchar('\n');
    }
}

void fraise_receivechars(const char *data, uint8_t len) {
    const char eesave_str[] = "SAVE_EEPROM";
    if(data[0] == 'E') { // Echo
        printf("E%s\n", data + 1);
    }
    else if(len >= strlen(eesave_str) && !strncmp(data, eesave_str, strlen(eesave_str))) {
        printf("l saving eeprom\n");
        eeprom_save();
    }
}

void eeprom_declare_main() {
    config.eeprom_declare();
}

