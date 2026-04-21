#include "encoder.h"

#include <Arduino.h>

pcnt_unit_t SPINDEL_ENCODER_PCNT_UNIT = PCNT_UNIT_0;
int64_t encoder_count = 0; // this will hold the total count of the encoder, we will update it with the deltas we read from the pcnt in readEncoder_steps_sinse_last() to keep track of the actual position in steps, since the pcnt counter is only 16 bit and will overflow/underflow quickly
constexpr int16_t PCNT_LIMIT = 32000;
constexpr int16_t DELTA_THRESHOLD = 20000; // threshold to detect overflow/underflow, should be less than half of the PCNT_LIMIT

void setupEncoder() {
    pinMode(SPINDEL_ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
    pinMode(SPINDEL_ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);
    pinMode(SPINDEL_ENCODER_CHANNEL_Z_PIN, INPUT_PULLUP);

    pcnt_config_t pcntA{
        .pulse_gpio_num = SPINDEL_ENCODER_CHANNEL_A_PIN,
        .ctrl_gpio_num = SPINDEL_ENCODER_CHANNEL_B_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = PCNT_LIMIT,
        .counter_l_lim = -PCNT_LIMIT,
        .unit = SPINDEL_ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_config_t pcntB{
        .pulse_gpio_num = SPINDEL_ENCODER_CHANNEL_B_PIN,
        .ctrl_gpio_num = SPINDEL_ENCODER_CHANNEL_A_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = PCNT_LIMIT,
        .counter_l_lim = -PCNT_LIMIT,
        .unit = SPINDEL_ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_1};

    pcnt_unit_config(&pcntA);
    pcnt_unit_config(&pcntB);

    pcnt_counter_pause(SPINDEL_ENCODER_PCNT_UNIT);
    pcnt_counter_clear(SPINDEL_ENCODER_PCNT_UNIT);
    pcnt_counter_resume(SPINDEL_ENCODER_PCNT_UNIT);
}

int16_t readEncoder_steps_sinse_last() {
    int16_t current_count;
    pcnt_get_counter_value(SPINDEL_ENCODER_PCNT_UNIT, &current_count);
    static int16_t last_count = 0;
    int16_t delta = current_count - last_count;
    if (delta > DELTA_THRESHOLD) {
        // Handle overflow
        delta -= PCNT_LIMIT;
    } else if (delta < -DELTA_THRESHOLD) {
        // Handle underflow
        delta += PCNT_LIMIT;
    }
    encoder_count += delta;
    last_count = current_count;

    return delta;
    // return current_count;
}
int64_t getEncoderPosition() {
    return encoder_count;
}

float get_encoder_degrees() {
    return fmodf(((float)encoder_count / SPINDEL_ENCODER_COUNT_PER_REV) * 360.0f, 360.0f); // Calculate the angle in degrees based on the encoder count and counts per revolution, and wrap it to 0-360 degrees
}

void resetEncoder() {
    encoder_count = 0;
    pcnt_counter_pause(SPINDEL_ENCODER_PCNT_UNIT);
    pcnt_counter_clear(SPINDEL_ENCODER_PCNT_UNIT);
    pcnt_counter_resume(SPINDEL_ENCODER_PCNT_UNIT);
}
