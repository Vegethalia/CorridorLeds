
//THIS FILE CAN BE USED AS A TEMPLATE TO CREATE A NEW Ex_file.cpp!!!
#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <WiFi.h>
#include <FastLED.h>
#include "SharedUtils/OtaUpdater.h"
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
#include <U8g2lib.h>

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

#define NUM_LEDS (960-2) //240 480 720 960

#define DATA_PIN    19
//#define PIN_LED 32
#define PIN_RELAY   5
#define PIN_PIR     23
#define PIN_DOPPLER 16
#define PIN_BTN_R   17
#define PIN_BTN_G   18
#define PIN_BTN_B   26

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
// unsigned long _lastCheckWifiOta=0;
//unsigned long _lastRepaint=0;
bool _updateNeeded=false;
OtaUpdater _OTA;

uint32_t _numCicles=0;
float    _fps=0;
uint8_t  _bckR = 1; //8,4,16=lila
uint8_t  _bckG = 2;
uint8_t  _bckB = 2;

//Led state with motion detection
bool     _LedsON=false;
uint32_t _LastMovement=0;
uint32_t _LastCheck4Wifi=0;

struct LedWave {
	float speed;
	float ledPos;
	uint8_t hue;

	void Inc() {ledPos+=speed;}
	void Reset() {ledPos=0.0f;}
};

LedWave _Wave1{ 0.25f, 0.0f, 64 };
LedWave _Wave2{ 0.50f, 0.0f, 128 };
LedWave _Wave3{ 1.00f, 0.0f, 224 };
LedWave _Wave4{ 1.75f, 0.0f, 0 };
LedWave _Wave5{ 3.00f, 0.0f, 96 };

//
//Advanced declarations
//

//Tries to reconnect to wifi (if disconnected), or nothing if WiFi is already connected or if last try was before RETRY_WIFI_EVERY_SECS
//Returns true if WiFi is NOW connected and before was not.
bool Connect2WiFi();
//Repaints the screen
 void PrintScreen();

// bool _UpdateRequired = false;
volatile uint32_t _DebounceTimer = 0;
void IRAM_ATTR ButtonPressed()
{
	if(millis() - _DebounceTimer >= DEBOUNCE_TIME) { // && !_UpdateRequired
		_DebounceTimer = millis();

		int pinValueR = digitalRead(PIN_BTN_R);
		int pinValueG = digitalRead(PIN_BTN_G);
		int pinValueB = digitalRead(PIN_BTN_B);

		if(pinValueR) _bckR+=1;
		if(pinValueG) _bckG+=1;
		if(pinValueB) _bckB+=1;
//		_UpdateRequired = true;
		//log_d("Updated Via OTA 2: Button State [%d, %d, %d] - RGB=(%d,%d,%d)", pinValueR, pinValueG, pinValueB, _bckR, _bckG, _bckB);
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
	u8g2.setContrast(2);
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

void DrawWave(uint16_t pos, uint8_t waveWidth, uint8_t hue, uint8_t maxBrightness=255, bool additive=false)
{
	int halfWave = waveWidth/2;
	for(int i = pos; i < pos + halfWave; i++) {
		byte vWave = maxBrightness / (halfWave - (i - pos));
		int pixelPos = i % NUM_LEDS;
		int pixelPosSim = (pos + (pos + halfWave * 2 - i - 1)) % NUM_LEDS;
		if(!additive) {
		_TheLeds[pixelPos] = CHSV(hue, 255, vWave);
		_TheLeds[pixelPosSim] = CHSV(hue, 255, vWave);
		}
		else {
			_TheLeds[pixelPos] += CHSV(hue, 255, vWave);
			_TheLeds[pixelPosSim] += CHSV(hue, 255, vWave);
		}
	}
}

#define COOLING  55
// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 120
#define BRIGHTNESS  200
#define FRAMES_PER_SECOND 60
bool gReverseDirection = false;

void Fire2012()
{
// Array of temperature readings at each simulation cell
	static byte heat[NUM_LEDS];

	// Step 1.  Cool down every cell a little
	for(int i = 0; i < NUM_LEDS; i++) {
		heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
	}

	// Step 2.  Heat from each cell drifts 'up' and diffuses a little
	for(int k = NUM_LEDS - 1; k >= 2; k--) {
		heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
	}

	// Step 3.  Randomly ignite new 'sparks' of heat near the bottom
	if(random8() < SPARKING) {
		int y = random8(7);
		heat[y] = qadd8(heat[y], random8(160, 255));
	}

	// Step 4.  Map from heat cells to LED colors
	for(int j = 0; j < NUM_LEDS; j++) {
		CRGB color = HeatColor(heat[j]);
		int pixelnumber;
		if(gReverseDirection) {
			pixelnumber = (NUM_LEDS - 1) - j;
		}
		else {
			pixelnumber = j;
		}
		_TheLeds[pixelnumber] = color;
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
		snprintf(buff, sizeof(buff), "RGB=(%3d,%3d,%3d)", _bckR, _bckG, _bckB);    u8g2.drawStr(0, 10, buff);
		snprintf(buff, sizeof(buff), "LEDS=%s", _LedsON ? "ON" : "OFF");           u8g2.drawStr(0, 21, buff);
		snprintf(buff, sizeof(buff), "Time Update=%3.1fms", _fps);                 u8g2.drawStr(0, 32, buff);
		snprintf(buff, sizeof(buff), "OTA=%s", msgOta);                            u8g2.drawStr(0, 43, buff);
		snprintf(buff, sizeof(buff), "IP=[%s]", WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "Not Connected");  u8g2.drawStr(0, 54, buff);
		snprintf(buff, sizeof(buff), "LastUpt=%u", (int)millis());                 u8g2.drawStr(0, 63, buff);


		// u8g2.setFont(u8g2_font_profont10_mf);
		// u8g2.drawStr(0, 6, "Soc Peque 6px");
		// u8g2.setFont(u8g2_font_ncenB14_tr);
		// u8g2.drawStr(0, 24, "Hola Bola!");
		// //u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
		// u8g2.setFont(u8g2_font_crox1h_tf);
		// u8g2.drawStr(0, 38, "Soc Gran 13px");
		// // u8g2.setFont(u8g2_font_p01type_tf);
		// // u8g2.drawStr(64, 30, "Soc Peque 4px");
		// u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
		// u8g2.drawStr(32, 60, "ABCDE");
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
	// else if((now - _LastMovement) > (TIME_ON_AFTER_MOVEMENT * 2 * 1000) && !_LedsON) {
	// 	//u8g2.setPowerSave(1);
	// }
	// if(_UpdateRequired) { //Button pressed
	// 	_bckR++;
	// 	_bckG++;
	// 	_bckB++;
	// 	log_d("Button pressed. RGB=(%d,%d,%d)", _bckR, _bckG, _bckB);
	// 	_UpdateRequired=false;
	// }

  // static uint8_t hue=0;
	// _TheLeds(0, NUM_LEDS - 1).fill_rainbow(hue);  // fill the first 20 items with a rainbow
	// hue+=5;
	// // _TheLeds(NUM_LEDS / 2, NUM_LEDS - 1) = _TheLeds(NUM_LEDS / 2 - 1, 0);
  // 	FastLED.delay(20);

	// _TheLeds[0] = CRGB::Blue;
	// _TheLeds[1] = CRGB::Green;
	// _TheLeds[2] = CRGB::Red;
	// FastLED.delay(500);
	// _TheLeds[0] = CRGB::Black;
	// _TheLeds[1] = CRGB::Black;
	// _TheLeds[2] = CRGB::Black;
	// FastLED.delay(500);

	// int max_leds = 30;
	// static uint8_t v=10;
	// static int dir=1;
	// _TheLeds(0, max_leds)=CHSV(160, 255, v);
	// v+=dir;
	// if(v>=128) {
	// 	dir=-1;
	// }
	// else if(v<=1) {
	// 	dir=+1;
	// }
	// FastLED.delay(20);
	//for

// 	static int hue=1;
// 	for(int i=0; i<NUM_LEDS; i++) {
// 		_TheLeds[i] = CHSV(hue, 255, 128);
// 	}
// 	FastLED.delay(20);
// 	hue++;

//  static int hue=1;
// 	for(int i=0; i<NUM_LEDS; i++) {
// 		_TheLeds[i] = CHSV(hue, i*4, 128);
// 	}
// 	FastLED.delay(20);
// 	hue++;

	if((now - _LastMovement) > (TIME_ON_AFTER_MOVEMENT * 1000) && _LedsON) {
		log_d("[%d] Turning off the leds", now);
		digitalWrite(PIN_RELAY, HIGH);
		_LedsON=false;
		_updateNeeded = true;
		_Wave1.Reset();	_Wave2.Reset();	_Wave3.Reset();	_Wave4.Reset(); _Wave5.Reset();
	}
	else if((now - _LastMovement) < (TIME_ON_AFTER_MOVEMENT * 1000)) {
		if(!_LedsON) {
			log_d("[%d] Turning on the leds", now);
			_lastUpdate = now;
			_updateNeeded = true; //només 1 cop
		}
		_LedsON=true;
		digitalWrite(PIN_RELAY, LOW);
	}

	if(_LedsON && (now - _lastUpdate) >= UPDATE_EVERY_MS) {
		//_lastUpdate = millis();
	//all leds deep blue beating
			// static float vPulse=8.0;
			// static float dirPulse=0.125f;
			// static float rg=1.0f;
			// static float rgDir=0.05f;
			// static float fHue=0.0f;
			// _TheLeds(0, NUM_LEDS - 1) = CHSV((uint8_t)fHue, 255, 32);
			// fHue+=dirPulse;
			// if(fHue>255.0) {
			// 	fHue=0.0;
			// }
//			_TheLeds(0, NUM_LEDS - 1) = CRGB((uint8_t)rg, (uint8_t)rg, (uint8_t)vPulse);
			_TheLeds(0, NUM_LEDS - 1) = CRGB(_bckR, _bckG, _bckB);//CRGB(0, 0, 0);
		// 	vPulse = vPulse+dirPulse;
		// if(vPulse >= 8.0 || vPulse <= 2.0) {
		// 	dirPulse = dirPulse*(-1.0);
		// }
		// rg += dirPulse;
		// if(rg >= 5.0 || rg <= 1.0) {
		// 	dirPulse = dirPulse * (-1.0);
		// }
		// //Serial.println(vPulse);

		// static int pos=0;
		const int waveWidth=24; //12
		const int wavePower=220;//180
		// DrawWave(pos, waveWidth, 160, 160, true);
		// DrawWave(pos + waveWidth * 2, waveWidth, 220, 160, true);
		// DrawWave(pos + waveWidth * 4, waveWidth, 64, 160, true);

		// pos++;

		_Wave1.Inc();
		_Wave2.Inc();
		_Wave3.Inc();
		_Wave4.Inc();
		_Wave5.Inc();
		DrawWave(_Wave1.ledPos, waveWidth, _Wave1.hue, wavePower, true);
		DrawWave(_Wave2.ledPos, waveWidth, _Wave2.hue, wavePower, true);
		DrawWave(_Wave3.ledPos, waveWidth, _Wave3.hue, wavePower, true);
		DrawWave(_Wave4.ledPos, waveWidth, _Wave4.hue, wavePower, true);
		DrawWave(_Wave5.ledPos, waveWidth, _Wave5.hue, wavePower, true);
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
