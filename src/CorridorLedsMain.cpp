#include <Arduino.h>
#include <memory>
#include <Wire.h>
#include <FS.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <FastLED.h>
#include "SharedUtils/OtaUpdater.h"
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
#include <list>
#include "SharedUtils/Utils.h"
#include "defines.h"
#include "EffectStyles.h"
#include "LedEffects.h"

#ifdef WITH_SCREEN
#include <U8g2lib.h>
//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
#endif

#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define UPDATE_EVERY_MS 10
#define FRAMES_FPS      100

#define DEBOUNCE_TIME 250 // Filtre anti-rebond (debouncer)

#define TIME_ON_AFTER_MOVEMENT 30 //seconds with leds on after movement detected

#define RETRY_WIFI_EVERY_SECS 60
#define PROCESS_OTA_EVERY_MS  2500
#define CHECK_FOR_OTA_TIME    (5*60*1000) //only process OTA the first 5 minutes after boot

//CRGB _TheLeds[NUM_LEDS];
CRGBArray<NUM_LEDS>  _TheLeds;
unsigned long _lastUpdate=0;
bool _updateNeeded=false;

OtaUpdater    _OTA;
PubSubClient  _ThePubSub;
WiFiClient    _TheWifi;
WiFiUDP       _TheWifi4UDP;
NTPClient     _TheNTPClient(_TheWifi4UDP);

uint32_t _numCicles=0;
float    _fps=0;
uint32_t _lastTimeSent = 0;

bool     _LedsON=false;
volatile bool     _movementDetected=false;
volatile uint32_t _LastMovement=0;
uint32_t _LastCheck4Wifi=0;
uint32_t _LastPowerMilliamps=0;
bool     _forceLedsON = false;
uint32_t _IntensityMAmps = DEF_TARGET_CURRENT;
uint32_t _Intensity = DEF_TARGET_INTENSITY;

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
//Connects to the MQTT broquer if not connected.
void Connect2MQTT();
//PubSubClient callback for received messages
void PubSubCallback(char* pTopic, uint8_t* pData, unsigned int dalaLength);

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
		_movementDetected = true;
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
		_movementDetected = true;
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

void AddRainbow(float speed)
{
	std::unique_ptr<LedEffect_Rainbow> effect = std::unique_ptr < LedEffect_Rainbow>(new LedEffect_Rainbow(speed));
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
	if(bands.size()<=2) {
		bands.clear();
		const uint8_t max_colors=8;
		uint8_t col1 = random8(0, max_colors);
		uint8_t col2 = random8(0, max_colors);
		if(col1==col2) {
			col2=(col1+1)%8;
		}
		for(uint8_t i=0; i<4; i++) {
			bands.push_back(col1);
			bands.push_back(col2);
		}
	}

	std::unique_ptr<LedEffect_MovingBanner> effect = std::unique_ptr < LedEffect_MovingBanner>(new LedEffect_MovingBanner(bands, _TheGlobalLedConfig.maxPulsePower, false, speed));
	effect->SetConfig(&_TheGlobalLedConfig);
	_TheEffects.push_back(std::move(effect));
}

//Adds a random set of effects to the Effects array
void CreateRandomEffect()
{
	LED_EFFECT eff = (LED_EFFECT)(random8(LED_EFFECT::MAX_EFFECT+1));
	uint8_t combi=0;
	//Initial test, create a bidir effect

	if(millis() < CHECK_FOR_OTA_TIME) {
		eff = LED_EFFECT::RAINBOW;
	}

	switch(eff) {
		case LED_EFFECT::PULSE:
			_TheGlobalLedConfig.bckR = _TheGlobalLedConfig.bckG = _TheGlobalLedConfig.bckB = 5;
			AddPulseEffect(0.50f, HSVHue::HUE_YELLOW);  //yellow 1
			AddPulseEffect(1.00f, HSVHue::HUE_AQUA); //aqua2
			AddPulseEffect(1.50f, HSVHue::HUE_PINK); //pink3
			AddPulseEffect(2.00f, HSVHue::HUE_RED);   //red4.5
			AddPulseEffect(3.00f, HSVHue::HUE_GREEN);  //green7
			break;
		case LED_EFFECT::BIDIR_PULSE:
		{
			combi = random8(g_BackAndBidirPulseCombinations.size());
			bool gray = random8(2)==1?true:false;
			if(gray) {
				_TheGlobalLedConfig.bckR = _TheGlobalLedConfig.bckG = _TheGlobalLedConfig.bckB = 5;
			}
			else {
				_TheGlobalLedConfig.bckR = g_BackAndBidirPulseCombinations[combi].BackR;
				_TheGlobalLedConfig.bckG = g_BackAndBidirPulseCombinations[combi].BackG;
				_TheGlobalLedConfig.bckB = g_BackAndBidirPulseCombinations[combi].BackB;
			}

			//AddBiPulseEffect(3.00f, g_BackAndBidirPulseCombinations[combi].PulseHue);
			AddBiPulseEffect(random8(DEF_PULSE_SPEED, MAX_PULSE_SPEED), g_BackAndBidirPulseCombinations[combi].PulseHue);
			break;
		}
		case LED_EFFECT::MOVING_BANNER:
			combi = random8(g_BannerCombinations.size());

			//AddMovingBanner(2.00f, g_BannerCombinations[combi]);
			AddMovingBanner(random8(DEF_PULSE_SPEED, DEF_PULSE_SPEED * 2), g_BannerCombinations[combi]);
			break;
		case LED_EFFECT::RAINBOW:
			_TheGlobalLedConfig.bckR = _TheGlobalLedConfig.bckG = _TheGlobalLedConfig.bckB = 0;

			//AddMovingBanner(2.00f, g_BannerCombinations[combi]);
			AddRainbow(random8(DEF_PULSE_SPEED, DEF_PULSE_SPEED * 2));
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
			_TheNTPClient.begin();
			_TheNTPClient.setTimeOffset(3600);
			return true;
		}
	}
	return false; //Too soon to retry, wait.
}

void Connect2MQTT()
{
	if(!_ThePubSub.connected()) {
		_ThePubSub.setClient(_TheWifi);
		_ThePubSub.setServer(MQTT_BROKER, MQTT_PORT);
		_ThePubSub.setCallback(PubSubCallback);
		if(!_ThePubSub.connect("ESP32_CorridorLeds")) {
			log_d("ERROR!! PubSubClient was not able to connect to PiRuter!!");
		}
		else { //Subscribe to the feeds
			log_d("PubSubClient connected to PiRuter MQTT broker!!");
			if(!_ThePubSub.subscribe(TOPIC_INTENSITY)) {
				log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_INTENSITY);
			}
			if(!_ThePubSub.subscribe(TOPIC_ALWAYS_ON)) {
				log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_ALWAYS_ON);
			}
		}
	}
}

void PubSubCallback(char* pTopic, uint8_t* pData, unsigned int dataLenght)
{
	std::string theTopic(pTopic);
	std::string theMsg;

	for(uint16_t i = 0; i < dataLenght; i++) {
		theMsg.push_back((char)pData[i]);
	}
	log_d("Received message from [%s]: [%s]", theTopic.c_str(), theMsg.c_str());

	if(theTopic.find(TOPIC_ALWAYS_ON) != std::string::npos) {
		if(theMsg == "SI") {
			_forceLedsON = true;
		}
		else {
			log_d("Turning off the LEDS...");
			_forceLedsON = false;
		}
	}
	else if(theTopic.find(TOPIC_INTENSITY) != std::string::npos) {
		auto origIntensity=_Intensity;
		auto newIntensity = min(std::atoi(theMsg.c_str()), MAX_TARGET_INTENSITY);
		if(newIntensity < MIN_TARGET_INTENSITY) {
			newIntensity = MIN_TARGET_INTENSITY;
		}
		if(newIntensity != origIntensity) {
			log_d("Changing led intensity=%d", newIntensity);
			_Intensity = newIntensity;
//			FastLED.setMaxPowerInVoltsAndMilliamps(5, _IntensityMAmps);              // FastLED power management set at 5V, 1500mA
			_ThePubSub.publish(TOPIC_INTENSITY, Utils::string_format("%d", _Intensity).c_str(), true);
			_ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Updated intensity=%d", _Intensity).c_str(), true);
		}
	}
}


void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while(!Serial);

#ifdef WITH_SCREEN
	log_d("Begin Display...");
	u8g2.begin();
	u8g2.setContrast(1);
	PrintScreen();
	_updateNeeded=true;
#endif

	//pinMode(PIN_LED, OUTPUT);
	// pinMode(PIN_BTN_R, INPUT);
	// pinMode(PIN_BTN_G, INPUT);
	// pinMode(PIN_BTN_B, INPUT);
	// pinMode(PIN_PIR, INPUT);
	pinMode(PIN_DOPPLER, INPUT);
	pinMode(PIN_RELAY, OUTPUT);

	digitalWrite(PIN_RELAY, HIGH);

	FastLED.addLeds<WS2812B, DATA_PIN, GRB>(_TheLeds, NUM_LEDS);
	//	FastLED.setBrightness(4);
	//FastLED.setTemperature(ColorTemperature::DirectSunlight);
	//FastLED.setMaxPowerInVoltsAndMilliamps(5, _IntensityMAmps);              // FastLED power management set at 5V, 1500mA
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
	if(Connect2WiFi()) { //Recheck wifi connection. Returns true when wifi was down and now is up
		_OTA.Begin();
	}
}

void PrintScreen()
{
#ifdef WITH_SCREEN
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
#endif
}

void loop()
{
	auto now=millis();
	//now<CHECK_FOR_OTA_TIME &&
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
	if(WiFi.isConnected()) {
		_TheNTPClient.update();
		if(!_ThePubSub.connected()) {
			Connect2MQTT();
		}
	}

	if((now - _LastMovement) > (TIME_ON_AFTER_MOVEMENT * 1000) && _LedsON && !_forceLedsON) {
		log_d("[%d] Turning off the leds", now);
		digitalWrite(PIN_RELAY, TURN_OFF_RELY);
		_LedsON=false;
		_updateNeeded = true;
#ifdef WITH_SCREEN
		u8g2.setPowerSave(1);
#endif

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
	else if(!_LedsON && (_forceLedsON || _movementDetected)) {
		_movementDetected=false;
		log_d("[%d] Turning on the leds", now);
		_lastUpdate = now;
		_updateNeeded = true; //només 1 cop
#ifdef WITH_SCREEN
		u8g2.begin();
#endif
		CreateRandomEffect();
		_LedsON=true;
		digitalWrite(PIN_RELAY, TURN_ON_RELY);
	}

	if(_LedsON && (now - _lastUpdate) >= UPDATE_EVERY_MS) {
		_lastUpdate = now;
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

		FastLED.show(_Intensity);

		_numCicles++;
		if(_numCicles == FRAMES_FPS) {
			uint32_t totalTime = (now - _lastUpdate);
			_fps = totalTime / (float)FRAMES_FPS;
			log_d("Update time=%3.1fms", _fps);
			//_ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("Update time=%3.1fms", _fps).c_str(), true);
			_numCicles=0;
			//_totalTime=0;
			_lastUpdate=now;

			PrintScreen();
			_updateNeeded = false;
		}
	}
	if(_ThePubSub.connected()) {
		if((now - _lastTimeSent) > UPDATE_TIME_EVERY_SECS * 1000) {
			_ThePubSub.publish(TOPIC_TIME, _TheNTPClient.getFormattedTime().c_str(), true);
			_lastTimeSent = now;
		}
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
	}
}
