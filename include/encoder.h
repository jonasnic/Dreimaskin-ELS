#pragma once

#include "driver/pcnt.h"

constexpr int SPINDEL_ENCODER_CHANNEL_A_PIN = 32;
constexpr int SPINDEL_ENCODER_CHANNEL_B_PIN = 33;
constexpr int SPINDEL_ENCODER_CHANNEL_Z_PIN = 34;
constexpr int SPINDEL_ENCODER_PPR = 24;
constexpr int SPINDEL_ENCODER_COUNT_PER_REV = SPINDEL_ENCODER_PPR * 4;

extern pcnt_unit_t SPINDEL_ENCODER_PCNT_UNIT;

void setupEncoder();
