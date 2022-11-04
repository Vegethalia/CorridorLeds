#include <FastLED.h>
#include <vector>
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
	theLeds(0, NUM_LEDS - 1) = _pTheConfig->bckColor;
}

void LedEffect_ConstantBackground::Advance()
{
	//nothing to do
}

void LedEffect_ConstantBackground::Reset()
{
	//nothing to do
}

/////////////////////////////////////////
/// LedEffect_Rainbow
/////////////////////////////////////////
LedEffect_Rainbow::LedEffect_Rainbow(float speed)
	:_currentPos(0.0f)
	, _speed(speed)
{
	_DeleteOnTurnOff = true;
}

LedEffect_Rainbow::~LedEffect_Rainbow()
{

}

void LedEffect_Rainbow::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	if(!_pTheConfig) {
		return;
	}
	fill_rainbow(theLeds, NUM_LEDS, (uint8_t)_currentPos, (uint8_t)_speed);
}

void LedEffect_Rainbow::Advance()
{
	_currentPos+=_speed;
	if(_currentPos>255) {
		_currentPos = 0.0f;
	}
}

void LedEffect_Rainbow::Reset()
{
	_currentPos=0.0;
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

	_DeleteOnTurnOff = true;
}

LedEffect_MovingPulse::~LedEffect_MovingPulse()
{

}

void LedEffect_MovingPulse::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	uint8_t centralInt = DEF_PULSE_POWER;
	if(_pTheConfig) {
		centralInt = _pTheConfig->maxPulsePower;
	}
	DrawAtPos(theLeds, _currentPos, centralInt);
}

void LedEffect_MovingPulse::DrawAtPos(CRGBArray<NUM_LEDS>& theLeds, float thePos, uint8_t theMaxInt)
{
	const int halfWave = _width / 2;
	const uint16_t pos = thePos;
	const uint16_t centralLed = pos + halfWave;
	const uint16_t simetricConstant = (2 * pos) + _width;
	const uint8_t theHue=_theHue;
	const float powerStep = ((float)theMaxInt / (float)halfWave);
	for(int i = pos; i < centralLed; i++) {
		byte vWave = (i - pos + 1) * powerStep;
		int pixelPos = i % NUM_LEDS;
		int pixelPosSim = (simetricConstant - i - 1) % NUM_LEDS;
		if(!_additiveDrawing) {
			theLeds[pixelPos] = CHSV(theHue, 255, vWave);
			theLeds[pixelPosSim] = CHSV(theHue, 255, vWave);
		}
		else {
			theLeds[pixelPos] += CHSV(theHue, 255, vWave);
			theLeds[pixelPosSim] += CHSV(theHue, 255, vWave);
		}
	}
}

void LedEffect_MovingPulse::Advance()
{
	_currentPos+=_speed;
	if(_currentPos >= NUM_LEDS) {
		_currentPos -= NUM_LEDS;
	}
}

void LedEffect_MovingPulse::Reset()
{
	_IsFinished=false; //this effect never finishes...
	_currentPos = 0.0f;
}

/////////////////////////////////////////
/// LedEffect_MovingBanner
/////////////////////////////////////////
LedEffect_MovingBanner::LedEffect_MovingBanner(const std::vector<uint8_t>& bands, uint8_t maxBrightness, bool sameBrightness, float speed)
{
	_speed = speed;
	_currentPos = 0.0f;

	//create the bands
	const uint16_t numBands=min((int)bands.size(), NUM_LEDS);
	const float bandSize = (float)NUM_LEDS / (float)numBands;

	float fled = 0.0f;
	int iband = 0;

	if(sameBrightness) {
		for(; fled<(float)(NUM_LEDS); fled+=bandSize, iband++) {
			for(int iled = (int)fled; iled < (int)fled+bandSize; iled++) {
				_TheHues[iled] = CHSV(bands[iband], 255, maxBrightness);
			}
		}
	}
	else {
		int halfBand=bandSize/2;
		if(halfBand%2==1) {
			halfBand++;
		}
		float stepPower=maxBrightness/halfBand;
		for(; fled < (float)(NUM_LEDS); fled += bandSize, iband++) {
			float bright = stepPower;
			for(int iled = (int)fled, iPow=0; iled < (int)fled + bandSize; iled++, iPow++) {
				if(iPow<halfBand) {
					bright+=stepPower;
				}
				else {
					bright-=stepPower;
				}
				//bright = max(bright/25.0f, bright);
				_TheHues[iled] = CHSV(bands[iband], 255, (uint8_t)bright);
			}
		}

	}

	_DeleteOnTurnOff = true;
}

LedEffect_MovingBanner::~LedEffect_MovingBanner()
{

}

void LedEffect_MovingBanner::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	DrawAtPos(theLeds, _currentPos);
}

void LedEffect_MovingBanner::DrawAtPos(CRGBArray<NUM_LEDS>& theLeds, float thePos)
{
	int iPos=(int)thePos;
	for(int i=0; i<NUM_LEDS; i++) {
		theLeds[i] = _TheHues[iPos];
		iPos++;
		if(iPos>=NUM_LEDS) {
			iPos=0;
		}
	}
}

void LedEffect_MovingBanner::Advance()
{
	_currentPos += _speed;
	if(_currentPos >= NUM_LEDS) {
		_currentPos = (float)(((int)_currentPos)%NUM_LEDS);
	}
}

void LedEffect_MovingBanner::Reset()
{
	_IsFinished = false; //this effect never finishes...
	_currentPos = 0.0f;
}

/////////////////////////////////////////
/// LedEffect_BidirectionalPulse
/////////////////////////////////////////
LedEffect_BidirectionalPulse::LedEffect_BidirectionalPulse(uint8_t hue, float speed, uint8_t width, bool additive)
	:LedEffect_MovingPulse(hue, speed, width, additive)
{
	_currentPos=width/2;
	_DeleteOnTurnOff=true;
}

LedEffect_BidirectionalPulse::~LedEffect_BidirectionalPulse()
{

}

void LedEffect_BidirectionalPulse::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	uint8_t centralInt = DEF_PULSE_POWER;
	if(_pTheConfig) {
		centralInt = _pTheConfig->maxPulsePower;
	}
	DrawAtPos(theLeds, _currentPos, centralInt);
	DrawAtPos(theLeds, NUM_LEDS - _currentPos, centralInt*0.75);
}

/////////////////////////////////////////
/// LedEffect_HalfBidirectionalPulse
/////////////////////////////////////////
LedEffect_HalfBidirectionalPulse::LedEffect_HalfBidirectionalPulse(uint8_t hue, float speed, uint8_t width, bool additive)
	:LedEffect_MovingPulse(hue, speed, width, additive)
{
	_currentPos = width / 2;
	_DeleteOnTurnOff = true;
}

LedEffect_HalfBidirectionalPulse::~LedEffect_HalfBidirectionalPulse()
{

}

void LedEffect_HalfBidirectionalPulse::Draw(CRGBArray<NUM_LEDS>& theLeds)
{
	if(_IsFinished) {
		return;
	}
	uint8_t centralInt = DEF_PULSE_POWER;
	if(_pTheConfig) {
		centralInt = _pTheConfig->maxPulsePower;
	}
	DrawAtPos(theLeds, _currentPos, centralInt);
	DrawAtPos(theLeds, NUM_LEDS - _currentPos, centralInt);
	if(_currentPos > NUM_LEDS / 2) { //This effect runs to the half and finishes
		_IsFinished = true;
	}
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
void LedEffect_Fire2012::Reset()
{
	//nothing to do
}
