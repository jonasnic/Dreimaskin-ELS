#pragma once

#include "driver/pcnt.h"
#include <stdint.h>

// constexpr uint16_t SPINDLE_TO_ENCODER_RATIO = 200/40; // Gear ratio between the spindle and the encoder, if the encoder is not directly on the spindle 
constexpr uint16_t spindle_pully_teeth = 110; // Number of teeth on the spindle pulley
constexpr uint16_t encoder_pully_teeth = 40; // Number of teeth on the encoder pulley
constexpr float SPINDLE_TO_ENCODER_RATIO = (float)spindle_pully_teeth / (float)encoder_pully_teeth; // Gear ratio between the spindle and the encoder, if the encoder is not directly on the spindle 

constexpr int SPINDLE_ENCODER_CHANNEL_A_PIN = 32;
constexpr int SPINDLE_ENCODER_CHANNEL_B_PIN = 33;
constexpr int SPINDLE_ENCODER_CHANNEL_Z_PIN = 35;
// constexpr int SPINDLE_ENCODER_PPR = 24; // testBench encoder has 24 pulses per revolution, but with quadrature decoding we get 4 counts per pulse, so 96 counts per revolution of the encoder. If we have a gear ratio of 5:1 between the spindle and the encoder, then we get 480 counts per revolution of the spindle.
constexpr int SPINDLE_ENCODER_PPR = 1600;


constexpr int quadratureFactor = 4; // when counting all flanks of the encoder signal, we get 4 counts per pulse, so we need to multiply the pulses per revolution by 4 to get the counts per revolution
constexpr int32_t SPINDLE_ENCODER_COUNT_PER_REV = SPINDLE_ENCODER_PPR * quadratureFactor * SPINDLE_TO_ENCODER_RATIO; // Total counts per revolution of the spindle, considering the encoder resolution and gear ratio

extern pcnt_unit_t SPINDLE_ENCODER_PCNT_UNIT;

void setupEncoder();
void encoder_update();
int16_t read_encoder_steps_since_last();
int64_t getEncoderPosition();
void resetEncoder();
float get_encoder_degrees();
int32_t getEncoderRPM();
float getEncoderRPS();