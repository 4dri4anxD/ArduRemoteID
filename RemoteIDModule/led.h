#pragma once

#include <stdint.h>
#include "board_config.h"

#ifdef WS2812_LED_PIN
#include <Adafruit_NeoPixel.h>
#endif

enum class BLINK_TIMES : uint8_t {
    MOUNTING = 1,
    READ = 2,
    INIT = 3,
    READ_AIRPORTS = 4,
    READ_COUNTRIES = 5,
};

class Led {
public:
    enum class LedState {
        INIT=0,
        PFST_FAIL,
        ARM_FAIL,
        ARM_OK,
        STARTING,
        UPDATE_SUCCESS,
        UPDATE_FAIL,
        OFF
    };

    void set_state(LedState _state) {
        state = _state;
    }
    void update(void);

private:
    void init(void);
    bool done_init;
    uint32_t last_led_trig_ms;
    uint32_t last_extra_led_trig_ms;
    LedState state;

#ifdef WS2812_LED_PIN
    uint32_t last_led_strip_ms;
    Adafruit_NeoPixel ledStrip{2, WS2812_LED_PIN, NEO_GRB + NEO_KHZ800}; //the BlueMark db210pro boards has two LEDs, therefore we need to use 2.
#endif
};

extern Led led;
