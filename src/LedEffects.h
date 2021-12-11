#pragma once

#include "defines.h"

///Base class of all Led Effects.
///To create a new one, inherit from this class and implement the pure virtual methods.
class LedEffect
{
public:
	///Struct amb la config global de l'applicació
	struct StatusConfig {
		uint8_t  bckR = 1; //8,4,16=lila
		uint8_t  bckG = 1;
		uint8_t  bckB = 1;

		uint8_t maxPulsePower=DEF_PULSE_POWER;
	};

public:
	LedEffect() :_pTheConfig(nullptr), _IsFinished(false), _DeleteOnTurnOff(false){};
	virtual ~LedEffect(){};

public:
	///Seteja la config de l'applicació. Cridar abans de draw/advance
	virtual void SetConfig(StatusConfig *pTheConfig) {_pTheConfig=pTheConfig;}
	///Call to draw the current effect into the passed Led Strip
	virtual void Draw(CRGBArray<NUM_LEDS>  &_TheLeds) = 0;
	///Call to advance this effect to the next state
	virtual void Advance()=0;
	///If true, this effect has finished and can be either deleted or reseted (by calling Reset)
	virtual bool IsFinished() {return _IsFinished;}
	virtual bool DeleteOnTurnOff() { return _DeleteOnTurnOff; }
	///Call to reinit the effect to its starting state.
	///If called after IsFinished is true, the effect will run again.
	///When implementing it in child classes, remember to set _IsFinished to false.
	virtual void Reset()=0;

protected:
	const StatusConfig *_pTheConfig;
	bool _IsFinished; //if true, the effect has run its way and can be deleted. Further calls to Draw won't do anything.
	bool _DeleteOnTurnOff; //If true, delete this effect when turning off the leds
};

class LedEffect_ConstantBackground : public LedEffect
{
public:
	LedEffect_ConstantBackground();
	virtual ~LedEffect_ConstantBackground();

public:
	///Call to draw the current effect into the passed Led Strip
	virtual void Draw(CRGBArray<NUM_LEDS>& theLeds) override;
	///Call to advance this effect to the next state
	virtual void Advance() override;
	///Call to reinit the effect to its starting state.
	///If called after IsFinished is true, the effect will run again.
	///When implementing it in child classes, remember to set _IsFinished to false.
	virtual void Reset() override;
};

class LedEffect_MovingPulse : public LedEffect
{
public:
	//Initializes the pulse effect with a given color (hue value), the width of the pulse,
	//and the speed in "leds_per_update". The speed can be a decimal number.
	//The maximum intensity (the intensity in the central led) is globally defined in the StatusConfig.
	//By default the pulse is drawed adding the colors to those already present in the ledString.
	//If this "additive" behaviour is not needed, pass false to the additive param.
	LedEffect_MovingPulse(uint8_t hue, float speed = DEF_PULSE_SPEED, uint8_t width = DEF_PULSE_WIDTH, bool additive=true);
	virtual ~LedEffect_MovingPulse();

public:
	///Call to draw the current effect into the passed Led Strip
	virtual void Draw(CRGBArray<NUM_LEDS>& theLeds) override;
	///Call to advance this effect to the next state
	virtual void Advance() override;
	///Call to reinit the effect to its starting state.
	///If called after IsFinished is true, the effect will run again.
	///When implementing it in child classes, remember to set _IsFinished to false.
	virtual void Reset() override;

protected:
	///Performs the actual drawing, the central led and max intensity are passed as parameters. Called by Draw.
	void DrawAtPos(CRGBArray<NUM_LEDS>& theLeds, float thePos, uint8_t theMaxInt);

protected:
	uint8_t _theHue;
	uint8_t _width;
	float _currentPos;
	float _speed;
	bool _additiveDrawing;
};

class LedEffect_BidirectionalPulse : public LedEffect_MovingPulse
{
public:
	LedEffect_BidirectionalPulse(uint8_t hue, float speed = DEF_PULSE_SPEED, uint8_t width = DEF_PULSE_WIDTH, bool additive = true);
	virtual ~LedEffect_BidirectionalPulse();

public:
	///Call to draw the current effect into the passed Led Strip
	virtual void Draw(CRGBArray<NUM_LEDS>& theLeds) override;
	///Call to reinit the effect to its starting state.
	///If called after IsFinished is true, the effect will run again.
	///When implementing it in child classes, remember to set _IsFinished to false.
	// virtual void Reset() override;
};

#define COOLING  90 //55
// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 80 //120
//#define BRIGHTNESS  200
//#define FRAMES_PER_SECOND 60
#define FIRE_LEDS 90
class LedEffect_Fire2012 : public LedEffect
{
public:
	LedEffect_Fire2012();
	virtual ~LedEffect_Fire2012();

public:
	///Call to draw the current effect into the passed Led Strip
	virtual void Draw(CRGBArray<NUM_LEDS>& theLeds) override;
	///Call to advance this effect to the next state
	virtual void Advance() override;
	///Call to reinit the effect to its starting state.
	///If called after IsFinished is true, the effect will run again.
	///When implementing it in child classes, remember to set _IsFinished to false.
	virtual void Reset() override;

protected:
	uint8_t _heat[FIRE_LEDS*2];
	uint16_t _initPos;
	bool _reverseDirection;
};
