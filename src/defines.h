#pragma once

#define NUM_LEDS (960-2) //240 480 720 960

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define DATA_PIN    19
//#define PIN_LED 32
#define PIN_RELAY   5
#define PIN_PIR     23
#define PIN_DOPPLER 16
#define PIN_BTN_R   17
#define PIN_BTN_G   18
#define PIN_BTN_B   26

enum LED_EFFECT { BLACK_BKG, RGB_BKG };

#define DEF_PULSE_WIDTH 24
#define DEF_PULSE_POWER 240
#define DEF_PULSE_SPEED 1.0f
