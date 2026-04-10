#pragma once

#include "driver/pcnt.h"

#define SPINDEL_TO_ENCODER_RATIO 1 // Gear ratio between the spindle and the encoder, if the encoder is not directly on the spindle 
constexpr int SPINDEL_ENCODER_CHANNEL_A_PIN = 32;
constexpr int SPINDEL_ENCODER_CHANNEL_B_PIN = 33;
constexpr int SPINDEL_ENCODER_CHANNEL_Z_PIN = 34;
constexpr int SPINDEL_ENCODER_PPR = 100;
// constexpr int SPINDEL_ENCODER_PPR = 24;

constexpr int SPINDEL_ENCODER_COUNT_PER_REV = SPINDEL_ENCODER_PPR * 4 * SPINDEL_TO_ENCODER_RATIO; // Total counts per revolution of the spindle, considering the encoder resolution and gear ratio

extern pcnt_unit_t SPINDEL_ENCODER_PCNT_UNIT;

void setupEncoder();
int16_t readEncoder_steps_sinse_last();
int64_t getEncoderPosition();