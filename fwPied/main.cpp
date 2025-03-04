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
//uart_inst_t *DMX_UART = uart0;
#define DMX_CHAN_COUNT 512
unsigned char dmxBuf[DMX_CHAN_COUNT];
//DmxSlave dmx;

DmxInput dmxInput;
uint16_t dmxStartChannel = 1;

bool button, button_last;
int button_count;

void send_dest(int mot, int dest) {
    if(mot > 5) mot = 5;
    char buf[5] = {120, 0, 10};
    buf[3] = dest >> 8;
    buf[4] = dest & 255;
    fraise_master_sendbytes(50 + mot, buf, sizeof(buf));
    /*printf("dest %d %d: %02X %02X %02X %02X %02X %02X \n", 50 + mot, dest,
        buf[0], buf[1], buf[2], buf[3], buf[4]);*/
    printf("dest %d %d\n", 50 + mot, dest);
}

void send_pwm(int mot, int pwm) {
    if(mot > 5) mot = 5;
    if(pwm < -1023) pwm = -1023;
    else if(pwm > 1023) pwm = 1023;

    char buf[4] = {120, 4};
    buf[2] = pwm >> 8;
    buf[3] = pwm & 255;
    fraise_master_sendbytes(50 + mot, buf, sizeof(buf));
    /*printf("dest %d %d: %02X %02X %02X %02X %02X %02X \n", 50 + mot, dest,
        buf[0], buf[1], buf[2], buf[3], buf[4]);*/
    printf("pwm %d %d %d %d\n", 50 + mot, pwm, buf[2], buf[3]); // -1023 = FC01
}

class Perche {
    private:
    int32_t position = 0;
    int32_t last_pos = 0;
    int32_t destination = 0;
    uint16_t speed = 1;
    uint16_t channel;
    std::vector<uint8_t> motors;
    bool running = false;

    public:
    static const int NB_CHANS = 4;
    static const int POSITION_MAX = 6 * 250 / 0.15; // 6 meters, 250 step/tour, 15 cm/tour
    static const int SPEED_MAX = 100;
    Perche(std::vector<uint8_t> motors, uint16_t channel): channel(channel), motors(motors) {}

    void set_destination(int32_t dest) {
        destination = dest * 256;
        if(destination < 0) destination = 0;
    }

    void set_speed(uint16_t s) {
        speed = s;
        //if(speed == 0) speed = 1;
    }

    void parse_dmx() {
        static const int deadzone = 55;
        uint8_t dmx_dest    = dmxBuf[dmxStartChannel + channel + 0];
        uint8_t dmx_speed   = dmxBuf[dmxStartChannel + channel + 1];
        int dmx_rewind      = MAX(dmxBuf[dmxStartChannel + channel + 2] - deadzone, 0);
        int dmx_fastfw      = MAX(dmxBuf[dmxStartChannel + channel + 3] - deadzone, 0);
        if(dmx_rewind > 0 || dmx_fastfw > 0) {
            int pwm = ((dmx_fastfw - dmx_rewind) * 1023) / (255 - deadzone);
            for(auto m: motors) send_pwm(m, pwm);
            running = false;
        } else {
            if(!running) {
                last_pos = -1;
                for(auto m: motors) send_pwm(m, 0);
            }
            running = true;
        }
        set_destination((dmx_dest * POSITION_MAX) / 255);
        set_speed((dmx_speed * SPEED_MAX) / 255);
    }

    void update() {
        parse_dmx();
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

Perche pA({0, 1}, 0);
Perche pB({2, 3}, Perche::NB_CHANS);
Perche pC({/*4, */5}, Perche::NB_CHANS * 2);

std::vector<Perche*> perches{&pA, &pB, &pC};

void __isr dmxDataRecevied(DmxInput* instance) {
     // A DMX frame has been received :-)
     // Toggle some LED, depending on which pin the data arrived
     gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

void setup() {
    eeprom_load();

    /*gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);*/

    //dmx.init(DMX_UART, DMX_TX_PIN, DMX_RX_PIN);
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

    /*if(gpio_get(BUTTON_PIN)) {
        if(button_count > 0) button_count--;
        else button = false;
    } else {
        if(button_count < 1000) button_count++;
        else button = true;
    }
    if(button_last != button) {
        button_last = button;
        printf("b %d\n", button);
        if(button) {
        }
    }*/

    /*if(dmx.transfer_finished()) {
        dmx.transfer_frame(dmxBuf, DMX_CHAN_COUNT);
    }*/
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

