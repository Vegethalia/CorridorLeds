#pragma once

#include <pixeltypes.h> //define enums HSVHue and HTMLColorCode
#include "defines.h"


///This file includes de color and style combinations that can used by the different Effects

struct BackAndPulseColors {
	uint8_t BackR;
	uint8_t BackG;
	uint8_t BackB;
	uint8_t PulseHue;
};

std::vector < HSVHue> g_BasicHues = {
	HSVHue::HUE_RED, HSVHue::HUE_ORANGE, HSVHue::HUE_YELLOW, HSVHue::HUE_GREEN,
	HSVHue::HUE_AQUA, HSVHue::HUE_BLUE, HSVHue::HUE_PURPLE, HSVHue::HUE_PINK
	};

std::vector<BackAndPulseColors> g_BackAndBidirPulseCombinations = {
	{1, 0, 0, HSVHue::HUE_RED}, {2, 1, 0, HSVHue::HUE_ORANGE}, {1, 1, 0, HSVHue::HUE_YELLOW}, {0, 1, 0, HSVHue::HUE_GREEN},
	{0, 1, 1, HSVHue::HUE_AQUA}, {0, 0, 1, HSVHue::HUE_BLUE}, {1, 0, 1, HSVHue::HUE_PURPLE}, {1, 0, 1, HSVHue::HUE_PINK}
	};
