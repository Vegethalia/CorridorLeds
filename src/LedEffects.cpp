#include <FastLED.h>
#include "LedEffects.h"


/////////////////////////////////////////
/// LedEffect_ConstantBackground
/////////////////////////////////////////
LedEffect_ConstantBackground::LedEffect_ConstantBackground()
{

}

LedEffect_ConstantBackground::~LedEffect_ConstantBackground()
{

}

void LedEffect_ConstantBackground::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	if(!_pTheConfig) {
		return;
	}
	theLeds(0, NUM_LEDS - 1) = CRGB(_pTheConfig->bckR, _pTheConfig->bckG, _pTheConfig->bckB);
}

void LedEffect_ConstantBackground::Advance()
{
	//nothing to do
}


/////////////////////////////////////////
/// LedEffect_MovingPulse
/////////////////////////////////////////
LedEffect_MovingPulse::LedEffect_MovingPulse(uint8_t hue, float speed, uint8_t width, bool additive)
{
	_theHue=hue;
	_width=width;
	_speed=speed;
	_currentPos=0.0f;
	_additiveDrawing=additive;
}

LedEffect_MovingPulse::~LedEffect_MovingPulse()
{

}

void LedEffect_MovingPulse::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	uint8_t centralInt=DEF_PULSE_POWER;
	if(_pTheConfig) {
		centralInt = _pTheConfig->maxPulsePower;
	}

	int halfWave = _width / 2;
	uint16_t pos=_currentPos;
	for(int i = pos; i < pos + halfWave; i++) {
		byte vWave = centralInt / (halfWave - (i - pos));
		int pixelPos = i % NUM_LEDS;
		int pixelPosSim = (pos + (pos + halfWave * 2 - i - 1)) % NUM_LEDS;
		if(!_additiveDrawing) {
			theLeds[pixelPos] = CHSV(_theHue, 255, vWave);
			theLeds[pixelPosSim] = CHSV(_theHue, 255, vWave);
		}
		else {
			theLeds[pixelPos] += CHSV(_theHue, 255, vWave);
			theLeds[pixelPosSim] += CHSV(_theHue, 255, vWave);
		}
	}
}

void LedEffect_MovingPulse::Advance()
{
	_currentPos+=_speed;
}

/////////////////////////////////////////
/// LedEffect_Fire2012
/////////////////////////////////////////
LedEffect_Fire2012::LedEffect_Fire2012()
	: _initPos(NUM_LEDS / 2)
	, _reverseDirection(false)
{

	memset(_heat, FIRE_LEDS, sizeof(uint8_t));
}

LedEffect_Fire2012::~LedEffect_Fire2012()
{

}

void LedEffect_Fire2012::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	// Step 1.  Cool down every cell a little
	for(int i = 0; i < FIRE_LEDS; i++) {
		_heat[i] = qsub8(_heat[i], random8(0, ((COOLING * 10) / FIRE_LEDS) + 2));
	}

	// Step 2.  Heat from each cell drifts 'up' and diffuses a little
	for(int k = FIRE_LEDS - 1; k >= 2; k--) {
		_heat[k] = (_heat[k - 1] + _heat[k - 2] + _heat[k - 2]) / 3;
	}

	// Step 3.  Randomly ignite new 'sparks' of heat near the bottom
	if(random8() < SPARKING) {
		int y = random8(7);
		_heat[y] = qadd8(_heat[y], random8(160, 255));
	}

	// Step 4.  Map from heat cells to LED colors
	for(int j = 0; j < FIRE_LEDS; j++) {
		CRGB color = HeatColor(_heat[j]);
		int pixelnumber;
		if(_reverseDirection) {
			pixelnumber = (FIRE_LEDS - 1) - j;
		}
		else {
			pixelnumber = j;
		}
		theLeds[_initPos + pixelnumber*2] = color;
		theLeds[_initPos + pixelnumber*2 + 1] = color;
		theLeds[_initPos + pixelnumber*2 + 2] = color;
	}
}

void LedEffect_Fire2012::Advance()
{
	//nothing to do
}
