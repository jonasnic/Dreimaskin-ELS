#pragma once

#include "driver/pcnt.h"

// constexpr uint16_t SPINDEL_TO_ENCODER_RATIO = 200/40; // Gear ratio between the spindle and the encoder, if the encoder is not directly on the spindle 
constexpr uint16_t SPINDEL_TO_ENCODER_RATIO = 1; // Gear ratio between the spindle and the encoder, if the encoder is not directly on the spindle 

constexpr int SPINDEL_ENCODER_CHANNEL_A_PIN = 32;
constexpr int SPINDEL_ENCODER_CHANNEL_B_PIN = 33;
constexpr int SPINDEL_ENCODER_CHANNEL_Z_PIN = 35;
// constexpr int SPINDEL_ENCODER_PPR = 24; // testBench encoder has 24 pulses per revolution, but with quadrature decoding we get 4 counts per pulse, so 96 counts per revolution of the encoder. If we have a gear ratio of 5:1 between the spindle and the encoder, then we get 480 counts per revolution of the spindle.
constexpr int SPINDEL_ENCODER_PPR = 1600;


constexpr int quadratureFactor = 4; // when counting all flanks of the encoder signal, we get 4 counts per pulse, so we need to multiply the pulses per revolution by 4 to get the counts per revolution
constexpr int SPINDEL_ENCODER_COUNT_PER_REV = SPINDEL_ENCODER_PPR * quadratureFactor * SPINDEL_TO_ENCODER_RATIO; // Total counts per revolution of the spindle, considering the encoder resolution and gear ratio

extern pcnt_unit_t SPINDEL_ENCODER_PCNT_UNIT;

void setupEncoder();
int16_t readEncoder_steps_sinse_last();
int64_t getEncoderPosition();
void resetEncoder();
float get_encoder_degrees();