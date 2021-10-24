
//THIS FILE CAN BE USED AS A TEMPLATE TO CREATE A NEW Ex_file.cpp!!!
#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <WiFi.h>
#include <FastLED.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

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

#define UPDATE_EVERY_MS 15
#define FRAMES_FPS      1000

#define DEBOUNCE_TIME 250 // Filtre anti-rebond (debouncer)

#define TIME_ON_AFTER_MOVEMENT 30 //seconds with leds on after movement detected

//CRGB _TheLeds[NUM_LEDS];
CRGBArray<NUM_LEDS>  _TheLeds;
unsigned long _lastUpdate=0;

uint32_t _numCicles=0;
uint32_t _totalTime=0;
uint8_t  _bckR = 1; //8,4,16=lila
uint8_t  _bckG = 1;
uint8_t  _bckB = 1;

//Led state with motion detection
bool     _LedsON=false;
uint32_t _LastMovement=0;

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

bool _UpdateRequired = false;
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
		log_d("Button State [%d, %d, %d] - RGB=(%d,%d,%d)", pinValueR, pinValueG, pinValueB, _bckR, _bckG, _bckB);
	}
}

void IRAM_ATTR MovementDetected()
{
	if(millis() - _LastMovement >= DEBOUNCE_TIME) {
		log_d("[%d] Motion Detected!!", millis());

		//int pinValue = digitalRead(PIN_PIR);
		//log_d("[%d] Pin value=%d", (int)millis(), pinValue);

		_LastMovement = millis();
	}
}

void IRAM_ATTR MovementDetectedDoppler()
{
	int pinValue = digitalRead(PIN_DOPPLER);
	log_d("[%d] Doppler Pin value=%d", (int)millis(), pinValue);
	if(pinValue && (millis() - _LastMovement) >= DEBOUNCE_TIME) {
		log_d("[%d] Motion Detected Doppler!!", millis());

		//int pinValue = digitalRead(PIN_PIR);
		//log_d("[%d] Pin value=%d", (int)millis(), pinValue);

		_LastMovement = millis();
	}
}

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while(!Serial);

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

void loop()
{
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

	if((millis() - _LastMovement) > (TIME_ON_AFTER_MOVEMENT*1000) && _LedsON) {
		log_d("[%d] Turning off the leds", millis());
		digitalWrite(PIN_RELAY, HIGH);
		_LedsON=false;
		_Wave1.Reset();	_Wave2.Reset();	_Wave3.Reset();	_Wave4.Reset(); _Wave5.Reset();
	}
	else if((millis() - _LastMovement) < (TIME_ON_AFTER_MOVEMENT * 1000)) {
		if(!_LedsON) {
			log_d("[%d] Turning on the leds", millis());
		}
		_LedsON=true;
		digitalWrite(PIN_RELAY, LOW);
	}

	if(_LedsON && (millis() - _lastUpdate) >= UPDATE_EVERY_MS) {
		_lastUpdate = millis();
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
		_totalTime += (millis() - _lastUpdate);
		if(_numCicles == FRAMES_FPS) {
			log_d("Update time=%dms", _totalTime/FRAMES_FPS);
			_numCicles=0;
			_totalTime=0;
		}
	}
 	//FastLED.delay(1);
	// Fire2012(); // run simulation frame

	// FastLED.show(); // display this frame
	// FastLED.delay(1000 / FRAMES_PER_SECOND);
}
