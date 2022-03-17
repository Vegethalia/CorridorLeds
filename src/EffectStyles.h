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
	{3, 2, 1, HSVHue::HUE_ORANGE}, {2, 2, 1, HSVHue::HUE_YELLOW}, {1, 2, 1, HSVHue::HUE_GREEN},
	{1, 2, 2, HSVHue::HUE_AQUA}, {1, 1, 2, HSVHue::HUE_BLUE}, {2, 1, 2, HSVHue::HUE_PURPLE}, {2, 1, 2, HSVHue::HUE_PINK}
}; //{1, 0, 0, HSVHue::HUE_RED},

std::vector<std::vector<uint8_t>> g_BannerCombinations = {
	{HSVHue::HUE_YELLOW, HSVHue::HUE_RED, HSVHue::HUE_YELLOW, HSVHue::HUE_RED, HSVHue::HUE_YELLOW, HSVHue::HUE_RED, HSVHue::HUE_YELLOW, HSVHue::HUE_RED},
	{HSVHue::HUE_BLUE, HSVHue::HUE_RED, HSVHue::HUE_BLUE, HSVHue::HUE_RED, HSVHue::HUE_BLUE, HSVHue::HUE_RED, HSVHue::HUE_BLUE, HSVHue::HUE_RED},
	{HSVHue::HUE_YELLOW, HSVHue::HUE_PURPLE, HSVHue::HUE_YELLOW, HSVHue::HUE_PURPLE, HSVHue::HUE_YELLOW, HSVHue::HUE_PURPLE, HSVHue::HUE_YELLOW, HSVHue::HUE_PURPLE},
	{HSVHue::HUE_YELLOW, HSVHue::HUE_ORANGE, HSVHue::HUE_RED, HSVHue::HUE_YELLOW, HSVHue::HUE_ORANGE, HSVHue::HUE_RED, HSVHue::HUE_YELLOW, HSVHue::HUE_ORANGE, HSVHue::HUE_RED},
	{HSVHue::HUE_BLUE, HSVHue::HUE_GREEN, HSVHue::HUE_BLUE, HSVHue::HUE_GREEN, HSVHue::HUE_BLUE, HSVHue::HUE_GREEN, HSVHue::HUE_BLUE, HSVHue::HUE_GREEN},
	{HSVHue::HUE_BLUE, HSVHue::HUE_AQUA, HSVHue::HUE_BLUE, HSVHue::HUE_AQUA, HSVHue::HUE_BLUE, HSVHue::HUE_AQUA, HSVHue::HUE_BLUE, HSVHue::HUE_AQUA},
	{ HSVHue::HUE_AQUA, HSVHue::HUE_PURPLE, HSVHue::HUE_AQUA, HSVHue::HUE_PURPLE, HSVHue::HUE_AQUA, HSVHue::HUE_PURPLE, HSVHue::HUE_AQUA, HSVHue::HUE_PURPLE },
	{ HSVHue::HUE_PINK, HSVHue::HUE_PURPLE, HSVHue::HUE_PINK, HSVHue::HUE_PURPLE, HSVHue::HUE_PINK, HSVHue::HUE_PURPLE, HSVHue::HUE_PINK, HSVHue::HUE_PURPLE }
	,{HSVHue::HUE_BLUE, HSVHue::HUE_YELLOW}
	};
