#include <Arduino.h>

#include <list>
#include <memory>
#include <vector>

#include <FS.h>
#include <FastLED.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <driver/rtc_io.h>

#include "EffectStyles.h"
#include "LedEffects.h"
#include "SharedUtils/OtaUpdater.h"
#include "SharedUtils/Utils.h"
#include "defines.h"
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#ifdef WITH_SCREEN
#include <U8g2lib.h>
// U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
#endif

#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define UPDATE_EVERY_MS 1
#define FRAMES_FPS 100

#define DEBOUNCE_TIME 250 // Filtre anti-rebond (debouncer)

#define RETRY_WIFI_EVERY_SECS 60
#define PROCESS_OTA_EVERY_MS 2500
#define KEEP_FIRST_RAINBOW_FOR (1 * 60 * 1000) // when turning on the micro. the first effect will allways be the rainbow

CRGBArray<NUM_LEDS> _TheLeds;
unsigned long _lastUpdate = 0;
unsigned long _timeON = 0;

OtaUpdater _OTA;
PubSubClient _ThePubSub;
WiFiClient _TheWifi;
WiFiUDP _TheWifi4UDP;
NTPClient _TheNTPClient(_TheWifi4UDP);

uint32_t _numCicles = 1;
float _fps = 0;
uint32_t _lastTimeSent = 0;

volatile bool _movementDetected = false;
volatile uint32_t _LastMovement = 0;
volatile bool _addHalfPulse = false;

volatile bool _LedsON = false;
uint32_t _LastCheck4Wifi = 0;
uint32_t _LastPowerMilliamps = 0;
uint32_t _LastSleep = 0;
bool _forceLedsON = false;
bool _forceLedsOFF = false;
uint32_t _IntensityMAmps = DEF_TARGET_CURRENT;
uint32_t _Intensity = DEF_INTENSITY;
bool _sleeping = false;

bool _CustomEffect = false; // if true, the effect will be defined via mqtt
LED_EFFECT _TheCustomEffect = LED_EFFECT::CONSTANT_BKG;
LedEffect::StatusConfig _TheGlobalLedConfig;
std::list<std::unique_ptr<LedEffect>> _TheEffects;
CRGB _LastCustomBackColor(5, 5, 5);
//
// Advanced declarations
//

// Tries to reconnect to wifi (if disconnected), or nothing if WiFi is already connected or if last try was before RETRY_WIFI_EVERY_SECS
// Returns true if WiFi is NOW connected and before was not.
bool Connect2WiFi();
// Connects to the MQTT broquer if not connected.
void Connect2MQTT();
// PubSubClient callback for received messages
void PubSubCallback(char* pTopic, uint8_t* pData, unsigned int dalaLength);

// volatile uint32_t _DebounceTimer = 0;
// void IRAM_ATTR ButtonPressed()
// {
// 	if(millis() - _DebounceTimer >= DEBOUNCE_TIME) { // && !_UpdateRequired
// 		_DebounceTimer = millis();

// 		int pinValueR = digitalRead(PIN_BTN_R);
// 		int pinValueG = digitalRead(PIN_BTN_G);
// 		int pinValueB = digitalRead(PIN_BTN_B);

// 		// if(_TheEffect == RGB_BKG) {
// 			// if(pinValueR) _TheGlobalLedConfig.bckR += 1;
// 			// if(pinValueG) _TheGlobalLedConfig.bckG += 1;
// 			// if(pinValueB) _TheGlobalLedConfig.bckB += 1;
// 		// }
// 		// else if(_TheEffect == BLACK_BKG) {
// 		// 	if(pinValueR) _TheGlobalLedConfig.bckR += 1;
// 		// 	if(pinValueG && _TheGlobalLedConfig.bckR) _TheGlobalLedConfig.bckR -= 1;
// 		// 	if(pinValueB) _TheGlobalLedConfig.bckR = 0;
// 		// }
// 	}
// }

// void IRAM_ATTR MovementDetected()
// {
// 	if(millis() - _LastMovement >= DEBOUNCE_TIME) {
// 		log_d("[%d] Motion Detected PIR!!", millis());

// 		//int pinValue = digitalRead(PIN_PIR);
// 		//log_d("[%d] Pin value=%d", (int)millis(), pinValue);

// 		_LastMovement = millis();
// 		_movementDetected = true;
// 	}
// }

void IRAM_ATTR MovementDetectedDoppler()
{
    int pinValue = digitalRead(PIN_DOPPLER);
    //	log_d("[%d] Doppler Pin value=%d", (int)millis(), pinValue);
    if (pinValue && (millis() - _LastMovement) >= DEBOUNCE_TIME) {
        log_d("[%d] Motion Detected Doppler!!", millis());

        // int pinValue = digitalRead(PIN_PIR);
        // log_d("[%d] Pin value=%d", (int)millis(), pinValue);

        _LastMovement = millis();
        _movementDetected = true;
        if (_LedsON) {
            _addHalfPulse = true;
        }
    }
}

// Adds a Pulse effect to the _Effects array, specifying the speed and the hue of the Pulse
void AddPulseEffect(float speed, uint8_t hue, uint8_t width = DEF_PULSE_WIDTH)
{
    std::unique_ptr<LedEffect_MovingPulse> effect = std::unique_ptr<LedEffect_MovingPulse>(new LedEffect_MovingPulse(hue, speed, width));
    effect->SetConfig(&_TheGlobalLedConfig);
    _TheEffects.push_back(std::move(effect));
}

// Adds a HalfPulse effect to the _Effects array, specifying the speed and the hue of the Pulse
void AddHalfPulseEffect(float speed, uint8_t hue, uint8_t width = DEF_PULSE_WIDTH)
{
    std::unique_ptr<LedEffect_HalfBidirectionalPulse> effect = std::unique_ptr<LedEffect_HalfBidirectionalPulse>(new LedEffect_HalfBidirectionalPulse(hue, speed, width));
    effect->SetConfig(&_TheGlobalLedConfig);
    _TheEffects.push_back(std::move(effect));
}

// Adds a Bidirectional Pulse effect to the _Effects array, specifying the speed and the hue of the Pulse
void AddBiPulseEffect(float speed, uint8_t hue)
{
    std::unique_ptr<LedEffect_BidirectionalPulse> effect = std::unique_ptr<LedEffect_BidirectionalPulse>(new LedEffect_BidirectionalPulse(hue, speed));
    effect->SetConfig(&_TheGlobalLedConfig);
    _TheEffects.push_back(std::move(effect));
}

void AddRainbow(float speed)
{
    std::unique_ptr<LedEffect_Rainbow> effect = std::unique_ptr<LedEffect_Rainbow>(new LedEffect_Rainbow(speed));
    effect->SetConfig(&_TheGlobalLedConfig);
    _TheEffects.push_back(std::move(effect));
}

void AddMovingBanner(float speed, std::vector<uint8_t>& bands)
{
    // std::vector<uint8_t> bands = {
    // 	HSVHue::HUE_YELLOW, HSVHue::HUE_RED,
    // 	HSVHue::HUE_YELLOW, HSVHue::HUE_RED,
    // 	HSVHue::HUE_YELLOW, HSVHue::HUE_RED,
    // 	HSVHue::HUE_YELLOW, HSVHue::HUE_RED}; //last yellow is the first one
    if (bands.size() <= 2) {
        bands.clear();
        const uint8_t max_colors = 8;
        uint8_t col1 = random8(0, max_colors);
        uint8_t col2 = random8(0, max_colors);
        if (col1 == col2) {
            col2 = (col1 + 1) % 8;
        }
        for (uint8_t i = 0; i < 4; i++) {
            bands.push_back(col1);
            bands.push_back(col2);
        }
    }

    std::unique_ptr<LedEffect_MovingBanner> effect = std::unique_ptr<LedEffect_MovingBanner>(new LedEffect_MovingBanner(bands, _TheGlobalLedConfig.maxPulsePower, false, speed));
    effect->SetConfig(&_TheGlobalLedConfig);
    _TheEffects.push_back(std::move(effect));
}

void AddSparksEffect(uint8_t theHue)
{
    std::unique_ptr<LedEffect_Sparks> effect = std::unique_ptr<LedEffect_Sparks>(new LedEffect_Sparks(theHue));
    effect->SetConfig(&_TheGlobalLedConfig);
    _TheEffects.push_back(std::move(effect));
}

// Adds a random set of effects to the Effects array
void CreateRandomEffect()
{
    LED_EFFECT eff = (LED_EFFECT)(random8(LED_EFFECT::MAX_EFFECT + 1));
    uint8_t combi = 0;

    if (millis() < KEEP_FIRST_RAINBOW_FOR) {
        eff = LED_EFFECT::SPARKS;
    }

    switch (eff) {
    case LED_EFFECT::PULSE:
        _TheGlobalLedConfig.bckColor.setRGB(5, 5, 5);
        AddPulseEffect(0.50f, HSVHue::HUE_YELLOW, random8(DEF_PULSE_WIDTH * 2 / 3, DEF_PULSE_WIDTH * 3 / 2));
        AddPulseEffect(1.00f, HSVHue::HUE_AQUA, random8(DEF_PULSE_WIDTH * 2 / 3, DEF_PULSE_WIDTH * 3 / 2));
        AddPulseEffect(1.50f, HSVHue::HUE_PINK, random8(DEF_PULSE_WIDTH * 2 / 3, DEF_PULSE_WIDTH * 3 / 2));
        AddPulseEffect(2.00f, HSVHue::HUE_RED, random8(DEF_PULSE_WIDTH * 2 / 3, DEF_PULSE_WIDTH * 3 / 2));
        AddPulseEffect(2.50f, HSVHue::HUE_BLUE, random8(DEF_PULSE_WIDTH * 2 / 3, DEF_PULSE_WIDTH * 3 / 2));
        AddPulseEffect(3.00f, HSVHue::HUE_GREEN, random8(DEF_PULSE_WIDTH * 2 / 3, DEF_PULSE_WIDTH * 3 / 2));
        // _ThePubSub.publish(TOPIC_DEBUG, "Pulse", true);
        break;
    case LED_EFFECT::BIDIR_PULSE: {
        combi = random8(g_BackAndBidirPulseCombinations.size());
        bool gray = random8(2) == 1 ? true : false;
        if (gray) {
            _TheGlobalLedConfig.bckColor.setRGB(5, 5, 5);
        } else {
            _TheGlobalLedConfig.bckColor.setRGB(
                g_BackAndBidirPulseCombinations[combi].BackR,
                g_BackAndBidirPulseCombinations[combi].BackG,
                g_BackAndBidirPulseCombinations[combi].BackB);
        }

        // AddBiPulseEffect(3.00f, g_BackAndBidirPulseCombinations[combi].PulseHue);
        AddBiPulseEffect(random8(DEF_PULSE_SPEED, MAX_PULSE_SPEED), g_BackAndBidirPulseCombinations[combi].PulseHue);
        // _ThePubSub.publish(TOPIC_DEBUG, "BiDirPulse", true);
        break;
    }
    case LED_EFFECT::MOVING_BANNER:
        combi = random8(g_BannerCombinations.size());

        // AddMovingBanner(2.00f, g_BannerCombinations[combi]);
        AddMovingBanner(random8(DEF_PULSE_SPEED, DEF_PULSE_SPEED * 2), g_BannerCombinations[combi]);
        // _ThePubSub.publish(TOPIC_DEBUG, "MovingBanner", true);
        break;
    case LED_EFFECT::SPARKS:
        AddSparksEffect(random8());
        break;
    case LED_EFFECT::RAINBOW:
        _TheGlobalLedConfig.bckColor.setRGB(5, 5, 5);

        AddRainbow(random8(DEF_PULSE_SPEED, DEF_PULSE_SPEED * 2));
        // _ThePubSub.publish(TOPIC_DEBUG, "Rainbow", true);
        break;
    case LED_EFFECT::CONSTANT_BKG:
    case LED_EFFECT::CONSTANT_RDM_BKG:
    default:
        _TheGlobalLedConfig.bckColor.setHSV(random8(255), 255, 100);
        break;
    }
}

// Creates again _theCustomEffect after turning on the leds
void RecreateCustomEffect()
{
    switch (_TheCustomEffect) {
    case LED_EFFECT::CONSTANT_BKG: // nothing to do
        _TheGlobalLedConfig.bckColor = _LastCustomBackColor;
        break;
    case LED_EFFECT::CONSTANT_RDM_BKG: // nothing to do
        _TheGlobalLedConfig.bckColor.setHSV(random8(), 255, 100); //_TheGlobalLedConfig.bckBrightness);
        _LastCustomBackColor = _TheGlobalLedConfig.bckColor;
        break;
    case LED_EFFECT::RAINBOW:
        _TheGlobalLedConfig.bckColor.setRGB(5, 5, 5); // i don't remember why i set the bck color here....

        AddRainbow(random8(DEF_PULSE_SPEED, DEF_PULSE_SPEED * 2));
        break;
    }
}

bool Connect2WiFi()
{
    if (WiFi.isConnected()) {
        return false; // false because was already connected
    }
    auto temps = millis() / 1000;

    if (temps < 3 || (temps - _LastCheck4Wifi) >= RETRY_WIFI_EVERY_SECS) {
        _LastCheck4Wifi = temps;
        log_d("[%d] Trying WiFi connection to [%s]", millis(), WIFI_SSID);
        auto err = WiFi.begin(WIFI_SSID, WIFI_PASS); // FROM mykeys.h
        err = (wl_status_t)WiFi.waitForConnectResult();
        if (err != wl_status_t::WL_CONNECTED) {
            log_d("WiFi connection FAILED! Error=[%d]. Will retry later", err);
            return false;
        } else {
            log_d("WiFi CONNECTED!");
            _TheNTPClient.begin();
            _TheNTPClient.setTimeOffset(3600);
            return true;
        }
    }
    return false; // Too soon to retry, wait.
}

void Connect2MQTT()
{
    if (!_ThePubSub.connected()) {
        _ThePubSub.setClient(_TheWifi);
        _ThePubSub.setServer(MQTT_BROKER, MQTT_PORT);
        _ThePubSub.setCallback(PubSubCallback);
        //		String s = WiFi.macAddress());
        if (!_ThePubSub.connect((String("ESP32_CorridorLeds") + WiFi.macAddress()[0]).c_str())) {
            log_d("ERROR!! PubSubClient was not able to connect to PiRuter!!");
        } else { // Subscribe to the feeds
            log_d("PubSubClient connected to PiRuter MQTT broker!!");
            _ThePubSub.publish(TOPIC_DEBUG, "PubSubClient connected to PiRuter MQTT broker!!", true);

            if (!_ThePubSub.subscribe(TOPIC_INTENSITY)) {
                log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_INTENSITY);
            }
            if (!_ThePubSub.subscribe(TOPIC_ALWAYS_ON)) {
                log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_ALWAYS_ON);
            }
            if (!_ThePubSub.subscribe(TOPIC_EFFECT)) {
                log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_EFFECT);
            }
            if (!_ThePubSub.subscribe(TOPIC_AUTO_MODE)) {
                log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_AUTO_MODE);
            }
            if (!_ThePubSub.subscribe(TOPIC_LAUNCH_PULSE)) {
                log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_LAUNCH_PULSE);
            }
        }
    }
}

// Checks if any effect has expired and can be removed
void CleanEffects()
{
    auto it = _TheEffects.begin();
    while (it != _TheEffects.end()) {
        if ((*it)->DeleteOnTurnOff()) {
            it = _TheEffects.erase(it);
        } else {
            ++it;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    // wait for serial monitor to open
    while (!Serial)
        ;

    // pinMode(PIN_LED, OUTPUT);
    //  pinMode(PIN_BTN_R, INPUT);
    //  pinMode(PIN_BTN_G, INPUT);
    //  pinMode(PIN_BTN_B, INPUT);
    //  pinMode(PIN_PIR, INPUT);
    rtc_gpio_deinit((gpio_num_t)PIN_DOPPLER);
    pinMode(PIN_DOPPLER, INPUT);
    pinMode(PIN_RELAY, OUTPUT);

    digitalWrite(PIN_RELAY, HIGH);

    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(_TheLeds, NUM_LEDS);
    //	FastLED.setBrightness(4);
    // FastLED.setTemperature(ColorTemperature::DirectSunlight);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, _IntensityMAmps); // FastLED power management set at 5V, 1500mA
    random16_set_seed(millis());

    // now we add the always-present-static-backcolor effect
    _TheGlobalLedConfig.bckColor.setRGB(0, 0, 0);
    std::unique_ptr<LedEffect_ConstantBackground> effect1 = std::unique_ptr<LedEffect_ConstantBackground>(new LedEffect_ConstantBackground());
    effect1->SetConfig(&_TheGlobalLedConfig);
    _TheEffects.push_back(std::move(effect1));

    // std::unique_ptr<LedEffect_Fire2012> effectFire = std::unique_ptr<LedEffect_Fire2012>(new LedEffect_Fire2012());
    // effectFire->SetConfig(&_TheGlobalLedConfig);
    // _TheEffects.push_back(std::move(effectFire));

    // attachInterrupt(PIN_BTN_R, ButtonPressed, RISING);
    // attachInterrupt(PIN_BTN_G, ButtonPressed, RISING);
    // attachInterrupt(PIN_BTN_B, ButtonPressed, RISING);
    // attachInterrupt(PIN_PIR, MovementDetected, RISING);
    attachInterrupt(PIN_DOPPLER, MovementDetectedDoppler, CHANGE);

    _OTA.Setup();
    WiFi.mode(WIFI_STA);
    if (Connect2WiFi()) { // Recheck wifi connection. Returns true when wifi was down and now is up
        _OTA.Begin();
    }

    // auto err = gpio_wakeup_enable((gpio_num_t)PIN_DOPPLER, GPIO_INTR_HIGH_LEVEL);
    // log_d("gpio_wakeup_enable returned [%d]", err);
    // err = esp_sleep_enable_gpio_wakeup();
    // log_d("esp_sleep_enable_gpio_wakeup returned [%d]", err);
}

void loop()
{
    bool goSleep = false;
    auto now = millis();

    if (_sleeping) {
        rtc_gpio_deinit((gpio_num_t)PIN_DOPPLER);
        pinMode(PIN_DOPPLER, INPUT);
        _movementDetected = true;
        _LastSleep = now;
        _sleeping = false;
    }

    // now<CHECK_FOR_OTA_TIME &&
    if ((now - _LastCheck4Wifi) > PROCESS_OTA_EVERY_MS) {
        if (Connect2WiFi()) { // Recheck wifi connection. Returns true when wifi was down and now is up
            _OTA.Begin();
        }
        if (WiFi.isConnected()) {
            _OTA.Process();
        }
        _LastCheck4Wifi = now;
    }
    if (WiFi.isConnected()) {
        _TheNTPClient.update();
        if (!_ThePubSub.connected()) {
            Connect2MQTT();
        }
    }

    if (_forceLedsOFF || ((now - _LastMovement) > (TIME_ON_AFTER_MOVEMENT * 1000) && _LedsON && !_forceLedsON)) {
        _LedsON = false;
        _forceLedsOFF = false;
        log_d("[%d] Turning off the leds", now);
        digitalWrite(PIN_RELAY, TURN_OFF_RELY);

        uint32_t totalTime = (now - _timeON);
        _timeON = 0;
        float timePerUpdate = (float)totalTime / (float)_numCicles;
        _fps = 1000.0 / timePerUpdate;
        // _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Turning OFF the leds. TimePerUpdate=[%dms] FPS=[%2.2f]", (uint32_t)timePerUpdate, _fps).c_str(), true);

        // check if any effect must be deleted on turn off
        CleanEffects();
        // if (now > CHECK_FOR_OTA_TIME) {
        //     goSleep = true;
        // }
    } else if (!_LedsON && (_forceLedsON || _movementDetected)) {
        _movementDetected = false;
        log_d("[%d] Turning on the leds", now);
        _lastUpdate = 0;
        _timeON = now;
        if (!_CustomEffect) {
            CreateRandomEffect();
        } else {
            RecreateCustomEffect();
        }
        _LedsON = true;
        _numCicles = 1;
        FastLED.setMaxPowerInVoltsAndMilliamps(5, 10);
        FastLED.show(0);
        digitalWrite(PIN_RELAY, TURN_ON_RELY);
    }

    if (_LedsON) { //} && (now - _lastUpdate) >= UPDATE_EVERY_MS) {
        if (_addHalfPulse) { // afegim els "laser pulses" a sobre de tots els altres efectes
            _addHalfPulse = false;
            AddHalfPulseEffect(7.0, random8(255));
        }
        _lastUpdate = now;
        auto it = _TheEffects.begin();
        while (it != _TheEffects.end()) {
            (*it)->Draw(_TheLeds);
            (*it)->Advance();
            if ((*it)->IsFinished()) {
                it = _TheEffects.erase(it);
            } else {
                ++it;
            }
        }

        if (_numCicles == 5) {
            FastLED.setMaxPowerInVoltsAndMilliamps(5, _IntensityMAmps);
        }
        // FastLED.show(_Intensity);
        FastLED.show();

        _numCicles++;
        // if(_numCicles == FRAMES_FPS) {
        // 	uint32_t totalTime = (now - _lastUpdate);
        // 	_fps = totalTime / (float)FRAMES_FPS;
        // 	log_d("Update time=%3.1fms", _fps);
        // 	//_ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Update time=%3.1fms", _fps).c_str(), true);
        // 	_numCicles=0;
        // 	//_totalTime=0;
        // 	_lastUpdate=now;
        // }
    }
    if ((!_LedsON || goSleep) && (now - _LastSleep) > (TIME_ON_AFTER_MOVEMENT * 2 * 1000)) {
        //		log_d("going 2 sleep [%d]", millis());
        //		_ThePubSub.publish(TOPIC_DEBUG, "Going to sleep!", true);
        //		delay(2000);
        //		goSleep=true;
    }

    if (_ThePubSub.connected()) {
        if ((now - _lastTimeSent) > UPDATE_TIME_EVERY_SECS * 1000) {
            _ThePubSub.publish(TOPIC_TIME, _TheNTPClient.getFormattedTime().c_str(), true);
            _lastTimeSent = now;
        }
        _ThePubSub.loop(); // allow the pubsubclient to process incoming messages
    }
    if (goSleep) {
        //		_sleeping=true;
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_DOPPLER, 1);
        // esp_light_sleep_start();
        // esp_deep_sleep_start();
        //		_movementDetected=true;
        _LastSleep = millis();
        // esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    }
}

void PubSubCallback(char* pTopic, uint8_t* pData, unsigned int dataLenght)
{
    std::string theTopic(pTopic);
    std::string theMsg;

    for (uint16_t i = 0; i < dataLenght; i++) {
        theMsg.push_back((char)pData[i]);
    }
    log_d("Received message from [%s]: [%s]", theTopic.c_str(), theMsg.c_str());

    if (theTopic.find(TOPIC_ALWAYS_ON) != std::string::npos) {
        if (theMsg == "SI") {
            _forceLedsON = true;
        } else {
            log_d("Turning off the LEDS...");
            _forceLedsON = false;
        }
    } else if (theTopic.find(TOPIC_INTENSITY) != std::string::npos) {
        auto origIntensity = _Intensity;
        auto newIntensity = max(MIN_INTENSITY, min(std::atoi(theMsg.c_str()), MAX_INTENSITY));

        if (newIntensity != origIntensity) {
            // log_d("Changing led intensity=%d", newIntensity);
            _Intensity = newIntensity;
            float percentPower = (float)_Intensity / (float)MAX_INTENSITY;
            _IntensityMAmps = (uint32_t)(((MAX_TARGET_CURRENT - MIN_TARGET_CURRENT) * percentPower) + MIN_TARGET_CURRENT);
            FastLED.setMaxPowerInVoltsAndMilliamps(5, _IntensityMAmps); // FastLED power management set at 5V, 1500mA
            _ThePubSub.publish(TOPIC_INTENSITY, Utils::string_format("%d", _Intensity).c_str(), true);
            _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Updated intensity=%dmAhs", _IntensityMAmps).c_str(), true);
        }
    }
    // else if(theTopic.find(TOPIC_BACK_BRIGHTNESS) != std::string::npos) {
    // 	auto newBright = max(1, min(std::atoi(theMsg.c_str()), 255));

    // 	if(newBright != _TheGlobalLedConfig.bckBrightness) {
    // 		_TheGlobalLedConfig.  = newBright;
    // 		_ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Updated backBrightness=%d", newBright).c_str(), true);
    // 	}
    // }
    else if (theTopic.find(TOPIC_EFFECT) != std::string::npos) {
        char origStr[theMsg.size() + 1];
        char *pEffect = nullptr, *pStyle = nullptr;
        strcpy(origStr, theMsg.c_str());

        if (Utils::SplitString2Values(origStr, &pEffect, &pStyle, "=")) {
            // log_d("Received Effect=[%s] with style=[%s]", pEffect, pStyle);
            _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Received Effect=[%s] with style=[%s]", pEffect, pStyle).c_str(), true);

            bool setColor = true;
            if (strcmp(pEffect, "cback") == 0) {
                uint8_t backHue;
                bool isRandom = false;
                if (strcmp(pStyle, "red") == 0) {
                    backHue = HSVHue::HUE_RED;
                } else if (strcmp(pStyle, "orange") == 0) {
                    backHue = HSVHue::HUE_ORANGE;
                } else if (strcmp(pStyle, "yellow") == 0) {
                    backHue = HSVHue::HUE_YELLOW;
                } else if (strcmp(pStyle, "green") == 0) {
                    backHue = HSVHue::HUE_GREEN;
                } else if (strcmp(pStyle, "aqua") == 0) {
                    backHue = HSVHue::HUE_AQUA;
                } else if (strcmp(pStyle, "blue") == 0) {
                    backHue = HSVHue::HUE_BLUE;
                } else if (strcmp(pStyle, "purple") == 0) {
                    backHue = HSVHue::HUE_PURPLE;
                } else if (strcmp(pStyle, "pink") == 0) {
                    backHue = HSVHue::HUE_PINK;
                } else if (strcmp(pStyle, "random") == 0) {
                    backHue = random8();
                    isRandom = true;
                } else {
                    setColor = false;
                }
                if (setColor) {
                    _TheCustomEffect = isRandom ? LED_EFFECT::CONSTANT_RDM_BKG : LED_EFFECT::CONSTANT_BKG;

                    _TheGlobalLedConfig.bckColor.setHSV(backHue, 255, 100); //_TheGlobalLedConfig.bckBrightness);
                    _LastCustomBackColor = _TheGlobalLedConfig.bckColor;
                    _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Custom Effect. %s Background. BckHue=%d", pStyle, backHue).c_str(), true);
                }
            } else if (strcmp(pEffect, "rainbow") == 0) {
                CleanEffects();
                _TheCustomEffect = LED_EFFECT::RAINBOW;
                if (_LedsON) {
                    AddRainbow(DEF_PULSE_SPEED);
                }
            }
        }
    } else if (theTopic.find(TOPIC_AUTO_MODE) != std::string::npos) {
        if (theMsg == "no") {
            _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Manual Mode Activated!").c_str(), true);
            _CustomEffect = true;
            // forcem una "eliminació dels efectes actius"
            _forceLedsOFF = true;
            _LastMovement = millis();
            _movementDetected = true;
            _TheGlobalLedConfig.bckColor = _LastCustomBackColor;
        } else {
            _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("AutoMode Activated!").c_str(), true);
            _CustomEffect = false;
        }
    } else if (theTopic.find(TOPIC_LAUNCH_PULSE) != std::string::npos) {
        _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Launch Laser Pulse!").c_str(), true);
        _addHalfPulse = true;
    }
}
