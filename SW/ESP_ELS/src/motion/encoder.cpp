#include "encoder.h"

#include <Arduino.h>

pcnt_unit_t SPINDEL_ENCODER_PCNT_UNIT = PCNT_UNIT_0;
int64_t encoder_count = 0; // this will hold the total count of the encoder, we will update it with the deltas we read from the pcnt in readEncoder_steps_sinse_last() to keep track of the actual position in steps, since the pcnt counter is only 16 bit and will overflow/underflow quickly

#define h_lim INT16_MAX //INT16_MAX
#define l_lim INT16_MIN     //INT16_MIN
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
        .counter_h_lim = h_lim,
        .counter_l_lim = l_lim,
        .unit = SPINDEL_ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0};

    pcnt_config_t pcntB{
        .pulse_gpio_num = SPINDEL_ENCODER_CHANNEL_B_PIN,
        .ctrl_gpio_num = SPINDEL_ENCODER_CHANNEL_A_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        // .counter_h_lim = INT16_MAX,
        // .counter_l_lim = INT16_MIN,
        .counter_h_lim = h_lim,
        .counter_l_lim = l_lim,
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
    if  (delta > 20000) {
        // Handle overflow
        delta += INT16_MIN;
    } else if (delta < -20000) {
        // Handle underflow
        delta += INT16_MAX; 
    }
    encoder_count += delta;
    last_count = current_count;
    return delta;
    // return current_count;

}
int64_t getEncoderPosition() {
    return encoder_count;
}