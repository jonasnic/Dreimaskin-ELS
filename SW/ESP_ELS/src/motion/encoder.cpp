#include "encoder.h"

#include <Arduino.h>

pcnt_unit_t SPINDLE_ENCODER_PCNT_UNIT = PCNT_UNIT_0;
int64_t encoder_count = 0; // this will hold the total count of the encoder, we will update it with the deltas we read from the pcnt in read_encoder_steps_since_last() to keep track of the actual position in steps, since the pcnt counter is only 16 bit and will overflow/underflow quickly
int16_t delta; // last delta value;
uint64_t delta_time; // time of the last delta update, used to calculate speed if needed
constexpr int16_t PCNT_LIMIT = 32000;
constexpr int16_t DELTA_THRESHOLD = 20000; // threshold to detect overflow/underflow, should be less than half of the PCNT_LIMIT

void setupEncoder() {
    pinMode(SPINDLE_ENCODER_CHANNEL_A_PIN, INPUT_PULLUP);
    pinMode(SPINDLE_ENCODER_CHANNEL_B_PIN, INPUT_PULLUP);
    pinMode(SPINDLE_ENCODER_CHANNEL_Z_PIN, INPUT_PULLUP);

    pcnt_config_t pcntA{
        .pulse_gpio_num = SPINDLE_ENCODER_CHANNEL_A_PIN,
        .ctrl_gpio_num = SPINDLE_ENCODER_CHANNEL_B_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = PCNT_LIMIT,
        .counter_l_lim = -PCNT_LIMIT,
        .unit = SPINDLE_ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_config_t pcntB{
        .pulse_gpio_num = SPINDLE_ENCODER_CHANNEL_B_PIN,
        .ctrl_gpio_num = SPINDLE_ENCODER_CHANNEL_A_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = PCNT_LIMIT,
        .counter_l_lim = -PCNT_LIMIT,
        .unit = SPINDLE_ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_1};

    pcnt_unit_config(&pcntA);
    pcnt_unit_config(&pcntB);

    pcnt_counter_pause(SPINDLE_ENCODER_PCNT_UNIT);
    pcnt_counter_clear(SPINDLE_ENCODER_PCNT_UNIT);
    pcnt_counter_resume(SPINDLE_ENCODER_PCNT_UNIT);
}

void encoder_update(){
    int16_t current_count;
    pcnt_get_counter_value(SPINDLE_ENCODER_PCNT_UNIT, &current_count);
    static int16_t last_count = 0;
    delta = current_count - last_count;
    last_count = current_count;
    if (delta > DELTA_THRESHOLD) {
        // Handle overflow
        delta -= PCNT_LIMIT;
    } else if (delta < -DELTA_THRESHOLD) {
        // Handle underflow
        delta += PCNT_LIMIT;
    }
    static uint64_t last_update_time = 0;
    uint64_t now = micros();
    delta_time = now - last_update_time;
    last_update_time = now;
    encoder_count += delta;
    
}

int16_t read_encoder_steps_since_last() {
    return delta;
}
int64_t getEncoderPosition() {
    return encoder_count;
}
float getEncoderRPS() {
    if (delta_time == 0) return 0;
    float rps = ((float)(delta*1000000) / SPINDLE_ENCODER_COUNT_PER_REV ) / delta_time;
    return rps;
}
int32_t getEncoderRPM() {
    return (int32_t)(getEncoderRPS() * 60.0f);
}

float get_encoder_degrees() {
    return fmodf(((float)encoder_count / SPINDLE_ENCODER_COUNT_PER_REV) * 360.0f, 360.0f); // Calculate the angle in degrees based on the encoder count and counts per revolution, and wrap it to 0-360 degrees
}

void resetEncoder() {
    encoder_count = 0;
    pcnt_counter_pause(SPINDLE_ENCODER_PCNT_UNIT);
    pcnt_counter_clear(SPINDLE_ENCODER_PCNT_UNIT);
    pcnt_counter_resume(SPINDLE_ENCODER_PCNT_UNIT);
}
