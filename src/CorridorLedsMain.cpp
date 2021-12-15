#include <Arduino.h>
#include <memory>
#include <Wire.h>
#include <FS.h>
#include <WiFi.h>
#include <FastLED.h>
#include "SharedUtils/OtaUpdater.h"
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
#include <U8g2lib.h>
#include <list>

#include "defines.h"
#include "EffectStyles.h"
#include "LedEffects.h"

//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define UPDATE_EVERY_MS 10
#define FRAMES_FPS      100

#define DEBOUNCE_TIME 250 // Filtre anti-rebond (debouncer)

#define TIME_ON_AFTER_MOVEMENT 30 //seconds with leds on after movement detected

#define RETRY_WIFI_EVERY_SECS 60
#define PROCESS_OTA_EVERY_MS  2500

//CRGB _TheLeds[NUM_LEDS];
CRGBArray<NUM_LEDS>  _TheLeds;
unsigned long _lastUpdate=0;
bool _updateNeeded=false;

OtaUpdater _OTA;

uint32_t _numCicles=0;
float    _fps=0;

bool     _LedsON=false;
uint32_t _LastMovement=0;
uint32_t _LastCheck4Wifi=0;

//LED_EFFECT _TheEffect = RGB_BKG;
LedEffect::StatusConfig _TheGlobalLedConfig;
std::list< std::unique_ptr < LedEffect>> _TheEffects;

//
//Advanced declarations
//

//Tries to reconnect to wifi (if disconnected), or nothing if WiFi is already connected or if last try was before RETRY_WIFI_EVERY_SECS
//Returns true if WiFi is NOW connected and before was not.
bool Connect2WiFi();
//Repaints the screen
void PrintScreen();

volatile uint32_t _DebounceTimer = 0;
void IRAM_ATTR ButtonPressed()
{
	if(millis() - _DebounceTimer >= DEBOUNCE_TIME) { // && !_UpdateRequired
		_DebounceTimer = millis();

		int pinValueR = digitalRead(PIN_BTN_R);
		int pinValueG = digitalRead(PIN_BTN_G);
		int pinValueB = digitalRead(PIN_BTN_B);

		// if(_TheEffect == RGB_BKG) {
			if(pinValueR) _TheGlobalLedConfig.bckR += 1;
			if(pinValueG) _TheGlobalLedConfig.bckG += 1;
			if(pinValueB) _TheGlobalLedConfig.bckB += 1;
		// }
		// else if(_TheEffect == BLACK_BKG) {
		// 	if(pinValueR) _TheGlobalLedConfig.bckR += 1;
		// 	if(pinValueG && _TheGlobalLedConfig.bckR) _TheGlobalLedConfig.bckR -= 1;
		// 	if(pinValueB) _TheGlobalLedConfig.bckR = 0;
		// }
		_updateNeeded=true;
	}
}

void IRAM_ATTR MovementDetected()
{
	if(millis() - _LastMovement >= DEBOUNCE_TIME) {
		log_d("[%d] Motion Detected PIR!!", millis());

		//int pinValue = digitalRead(PIN_PIR);
		//log_d("[%d] Pin value=%d", (int)millis(), pinValue);

		_LastMovement = millis();
	}
}

void IRAM_ATTR MovementDetectedDoppler()
{
	int pinValue = digitalRead(PIN_DOPPLER);
//	log_d("[%d] Doppler Pin value=%d", (int)millis(), pinValue);
	if(pinValue && (millis() - _LastMovement) >= DEBOUNCE_TIME) {
		log_d("[%d] Motion Detected Doppler!!", millis());

		//int pinValue = digitalRead(PIN_PIR);
		//log_d("[%d] Pin value=%d", (int)millis(), pinValue);

		_LastMovement = millis();
	}
}

//Adds a Pulse effect to the _Effects array, specifying the speed and the hue of the Pulse
void AddPulseEffect(float speed,  uint8_t hue)
{
	std::unique_ptr<LedEffect_MovingPulse> effect = std::unique_ptr < LedEffect_MovingPulse>(new LedEffect_MovingPulse(hue, speed));
	effect->SetConfig(&_TheGlobalLedConfig);
	_TheEffects.push_back(std::move(effect));
}

//Adds a Bidirectional Pulse effect to the _Effects array, specifying the speed and the hue of the Pulse
void AddBiPulseEffect(float speed, uint8_t hue)
{
	std::unique_ptr<LedEffect_BidirectionalPulse> effect = std::unique_ptr < LedEffect_BidirectionalPulse>(new LedEffect_BidirectionalPulse(hue, speed));
	effect->SetConfig(&_TheGlobalLedConfig);
	_TheEffects.push_back(std::move(effect));
}

//Adds a random set of effects to the Effects array
void CreateRandomEffect()
{
	LED_EFFECT eff = (LED_EFFECT)(millis()%(LED_EFFECT::MAX_EFFECT+1));
	//Initial test, create a bidir effect

	switch(eff) {
		case LED_EFFECT::PULSE:
			_TheGlobalLedConfig.bckR = _TheGlobalLedConfig.bckG = _TheGlobalLedConfig.bckB = 3;
			AddPulseEffect(0.50f, HSVHue::HUE_YELLOW);  //yellow
			AddPulseEffect(0.75f, HSVHue::HUE_AQUA); //aqua
			AddPulseEffect(1.00f, HSVHue::HUE_PINK); //pink
			AddPulseEffect(1.75f, HSVHue::HUE_RED);   //red
			AddPulseEffect(3.00f, HSVHue::HUE_GREEN);  //green
			break;
		case LED_EFFECT::BIDIR_PULSE:
			uint8_t combi = random8(g_BackAndBidirPulseCombinations.size());
			bool gray = random8(2)==1?true:false;
			if(gray) {
				_TheGlobalLedConfig.bckR = _TheGlobalLedConfig.bckG = _TheGlobalLedConfig.bckB = 3;
			}
			else {
				_TheGlobalLedConfig.bckR = g_BackAndBidirPulseCombinations[combi].BackR;
				_TheGlobalLedConfig.bckG = g_BackAndBidirPulseCombinations[combi].BackG;
				_TheGlobalLedConfig.bckB = g_BackAndBidirPulseCombinations[combi].BackB;
			}

			AddBiPulseEffect(3.00f, g_BackAndBidirPulseCombinations[combi].PulseHue);
			break;
	}
}

bool Connect2WiFi()
{
	if(WiFi.isConnected()) {
		return false; //false because was already connected
	}
	auto temps=millis()/1000;

	if(temps<3 || (temps-_LastCheck4Wifi)>=RETRY_WIFI_EVERY_SECS) {
		_LastCheck4Wifi=temps;
		log_d("[%d] Trying WiFi connection to [%s]", millis(), WIFI_SSID);
		auto err = WiFi.begin(WIFI_SSID, WIFI_PASS); //FROM mykeys.h
		err = (wl_status_t)WiFi.waitForConnectResult();
		if(err != wl_status_t::WL_CONNECTED) {
			log_d("WiFi connection FAILED! Error=[%d]. Will retry later", err);
			return false;
		}
		else {
			log_d("WiFi CONNECTED!");
			return true;
		}
	}
	return false; //Too soon to retry, wait.
}

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while(!Serial);

	log_d("Begin Display...");
	u8g2.begin();
	u8g2.setContrast(1);
	PrintScreen();
	_updateNeeded=true;

	//pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_BTN_R, INPUT);
	pinMode(PIN_BTN_G, INPUT);
	pinMode(PIN_BTN_B, INPUT);
	pinMode(PIN_PIR, INPUT);
	pinMode(PIN_DOPPLER, INPUT);
	pinMode(PIN_RELAY, OUTPUT);

	digitalWrite(PIN_RELAY, HIGH);

	FastLED.addLeds<WS2812B, DATA_PIN, GRB>(_TheLeds, NUM_LEDS);
	//	FastLED.setBrightness(4);
	FastLED.setTemperature(ColorTemperature::DirectSunlight);
	random16_set_seed(millis());

	// for(int i = 0; i < NUM_LEDS/2; i++) {
	// 	if((i % 3) == 0) {
	// 		_TheLeds[i] = CHSV(160, 255, 64);
	// 	}
	// }
	// for(int i = NUM_LEDS / 2; i < NUM_LEDS; i++) {
	// 	if((i % 6) == 0) {
	// 		_TheLeds[i] = CHSV(220, 255, 64);
	// 	}
	// }
	// FastLED.show();

	//now we add the standard led effects
	//_TheGlobalLedConfig.bckR = 1; _TheGlobalLedConfig.bckG = 1; _TheGlobalLedConfig.bckB = 1;
	_TheGlobalLedConfig.bckR = 0; _TheGlobalLedConfig.bckG = 0; _TheGlobalLedConfig.bckB = 0;
	 std::unique_ptr<LedEffect_ConstantBackground> effect1 = std::unique_ptr<LedEffect_ConstantBackground>(new LedEffect_ConstantBackground());
	 effect1->SetConfig(&_TheGlobalLedConfig);
	 _TheEffects.push_back(std::move(effect1));
	// AddPulseEffect(0.50f, 64);  //yellow
	// AddPulseEffect(0.75f, 128); //aqua
	// AddPulseEffect(1.00f, 224); //pink
	// AddPulseEffect(1.75f, 0);   //red
	// AddPulseEffect(3.00f, 96);  //green
	//AddBiPulseEffect(3.00f, 224);

	// std::unique_ptr<LedEffect_Fire2012> effectFire = std::unique_ptr<LedEffect_Fire2012>(new LedEffect_Fire2012());
	// effectFire->SetConfig(&_TheGlobalLedConfig);
	// _TheEffects.push_back(std::move(effectFire));


	attachInterrupt(PIN_BTN_R, ButtonPressed, RISING);
	attachInterrupt(PIN_BTN_G, ButtonPressed, RISING);
	attachInterrupt(PIN_BTN_B, ButtonPressed, RISING);
	attachInterrupt(PIN_PIR, MovementDetected, RISING);
	attachInterrupt(PIN_DOPPLER, MovementDetectedDoppler, CHANGE);

	_OTA.Setup();
	WiFi.mode(WIFI_STA);
	if(Connect2WiFi()) { //Recheck wifi connection. Returns true when wifi was down and now is up
		_OTA.Begin();
	}
}

void PrintScreen()
{
 	char buff[30];
	uint8_t percent=0;
	char msgOta[30];

	switch(_OTA.Status(percent)) {
		case OtaUpdater::OtaStatus::NOT_STARTED:
			snprintf(msgOta, sizeof(msgOta), "Not Started");
			break;
		case OtaUpdater::OtaStatus::READY:
			snprintf(msgOta, sizeof(msgOta), "Ready");
			break;
		case OtaUpdater::OtaStatus::UPDATING:
			snprintf(msgOta, sizeof(msgOta), "Updating %d%%", percent);
			break;
		case OtaUpdater::OtaStatus::ERROR:
			snprintf(msgOta, sizeof(msgOta), "Error:%s", _OTA.GetLastError().c_str());
			break;
		case OtaUpdater::OtaStatus::UPDATE_FINISHED:
			snprintf(msgOta, sizeof(msgOta), "Update Finished!");
			break;
	}

	u8g2.setFont(u8g2_font_6x10_tr); //u8g2_font_6x10_tr=6x10 u8g2_font_lubR08_tr=11x11   u8g2_font_crox1h_tr width=11 height=13

	u8g2.firstPage();
	do {
		//u8g2.setFont(u8g2_font_6x10_tr); //u8g2_font_6x10_tr=6x10 u8g2_font_lubR08_tr=11x11   u8g2_font_crox1h_tr width=11 height=13
		snprintf(buff, sizeof(buff), "RGB=(%3d,%3d,%3d)",
			_TheGlobalLedConfig.bckR, _TheGlobalLedConfig.bckG, _TheGlobalLedConfig.bckB);	u8g2.drawStr(0, 10, buff);
		snprintf(buff, sizeof(buff), "LEDS=%s", _LedsON ? "ON" : "OFF");                  u8g2.drawStr(0, 21, buff);
		snprintf(buff, sizeof(buff), "Time Update=%3.1fms", _fps);                        u8g2.drawStr(0, 32, buff);
		snprintf(buff, sizeof(buff), "OTA=%s", msgOta);                                   u8g2.drawStr(0, 43, buff);
		snprintf(buff, sizeof(buff), "IP=[%s]", WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "Not Connected");  u8g2.drawStr(0, 54, buff);
		snprintf(buff, sizeof(buff), "LastUpt=%u", (int)millis());                        u8g2.drawStr(0, 63, buff);
	} while(u8g2.nextPage());

	// _lastRepaint=millis();
}

void loop()
{
	auto now=millis();
	if((now - _LastCheck4Wifi) > PROCESS_OTA_EVERY_MS) {
		if(Connect2WiFi()) { //Recheck wifi connection. Returns true when wifi was down and now is up
			_OTA.Begin();
		}
		if(WiFi.isConnected()) {
			_OTA.Process();
		}
		_LastCheck4Wifi = now;
	}
	if(_updateNeeded || _OTA.Status() == OtaUpdater::OtaStatus::UPDATING) {
		//u8g2.setPowerSave(0);
		PrintScreen();
		_updateNeeded = false;
	}

	if((now - _LastMovement) > (TIME_ON_AFTER_MOVEMENT * 1000) && _LedsON) {
		log_d("[%d] Turning off the leds", now);
		digitalWrite(PIN_RELAY, HIGH);
		_LedsON=false;
		_updateNeeded = true;
		u8g2.setPowerSave(1);

		//check if any effect must be deleted on turn off
		auto it = _TheEffects.begin();
		while(it != _TheEffects.end()) {
			if((*it)->DeleteOnTurnOff()) {
				it = _TheEffects.erase(it);
			}
			else {
				++it;
			}
		}
	}
	else if((now - _LastMovement) < (TIME_ON_AFTER_MOVEMENT * 1000)) {
		if(!_LedsON) {
			log_d("[%d] Turning on the leds", now);
			_lastUpdate = now;
			_updateNeeded = true; //només 1 cop
			u8g2.begin();

			CreateRandomEffect();
		}
		_LedsON=true;
		digitalWrite(PIN_RELAY, LOW);
	}

	if(_LedsON && (now - _lastUpdate) >= UPDATE_EVERY_MS) {
		//_lastUpdate = millis();
		auto it=_TheEffects.begin();
		while(it!=_TheEffects.end()) {
			(*it)->Draw(_TheLeds);
			(*it)->Advance();
			if((*it)->IsFinished()) {
				it = _TheEffects.erase(it);
			}
			else {
				++it;
			}
		}

		FastLED.show();

		_numCicles++;
		if(_numCicles == FRAMES_FPS) {
			uint32_t totalTime = (millis() - _lastUpdate);
			_fps = totalTime / (float)FRAMES_FPS;
			log_d("Update time=%3.1fms", _fps);
			_numCicles=0;
			//_totalTime=0;
			_lastUpdate=millis();

			PrintScreen();
			_updateNeeded = false;
		}
	}
 	//FastLED.delay(1);
	// Fire2012(); // run simulation frame

	// FastLED.show(); // display this frame
	// FastLED.delay(1000 / FRAMES_PER_SECOND);
}
