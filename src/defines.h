#pragma once

#define NUM_LEDS (960 - 2 + 43) // 240 480 720 960

#define TIME_ON_AFTER_MOVEMENT 60 // seconds with leds on after movement detected

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define DATA_PIN 19
//#define PIN_LED 32
#define PIN_RELAY 5
#define PIN_PIR 23
#define PIN_DOPPLER 16
#define PIN_BTN_R 17
#define PIN_BTN_G 18
#define PIN_BTN_B 26

#define TURN_ON_RELY HIGH
#define TURN_OFF_RELY LOW

enum LED_EFFECT {
    CONSTANT_BKG,
    CONSTANT_RDM_BKG,
    PULSE,
    BIDIR_PULSE,
    MOVING_BANNER,
    SPARKS, 
    RAINBOW,
    MAX_EFFECT = RAINBOW
}; //, CUSTOM = MAX_EFFECT+1 };

#define DEF_PULSE_WIDTH 24
#define DEF_PULSE_POWER 246
#define DEF_PULSE_SPEED 1.0f
#define MAX_PULSE_SPEED 7.0f
#define DEF_BACK_BRIGHTNESS 25

#define DEF_TARGET_CURRENT 900 // default milliamps
#define MIN_TARGET_CURRENT 90 //
#define MAX_TARGET_CURRENT 2090 // max milliamps
#define DEF_INTENSITY 10
#define MIN_INTENSITY 1
#define MAX_INTENSITY 100

// MQTT related
#define MQTT_BROKER "192.168.1.140"
#define MQTT_PORT 1888

#define TOPIC_INTENSITY "caseta/leds/intensity"
#define TOPIC_BACK_BRIGHTNESS "caseta/leds/backbright"
#define TOPIC_STYLE "caseta/leds/style"
#define TOPIC_ALWAYS_ON "caseta/leds/alwayson"
#define TOPIC_AUTO_MODE "caseta/leds/auto"
#define TOPIC_TIME "caseta/leds/time"
#define TOPIC_EFFECT "caseta/leds/effect"
#define TOPIC_LAUNCH_PULSE "caseta/leds/pulse"

#define TOPIC_DEBUG "caseta/leds/debug"

#define UPDATE_TIME_EVERY_SECS 300
