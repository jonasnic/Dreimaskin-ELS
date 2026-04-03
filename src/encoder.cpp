#include "encoder.h"

#include <Arduino.h>

pcnt_unit_t SPINDEL_ENCODER_PCNT_UNIT = PCNT_UNIT_0;

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
        .counter_h_lim = INT16_MAX,
        .counter_l_lim = INT16_MIN,
        .unit = SPINDEL_ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0};

    pcnt_config_t pcntB{
        .pulse_gpio_num = SPINDEL_ENCODER_CHANNEL_B_PIN,
        .ctrl_gpio_num = SPINDEL_ENCODER_CHANNEL_A_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = INT16_MAX,
        .counter_l_lim = INT16_MIN,
        .unit = SPINDEL_ENCODER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_1};

    pcnt_unit_config(&pcntA);
    pcnt_unit_config(&pcntB);

    pcnt_counter_pause(SPINDEL_ENCODER_PCNT_UNIT);
    pcnt_counter_clear(SPINDEL_ENCODER_PCNT_UNIT);
    pcnt_counter_resume(SPINDEL_ENCODER_PCNT_UNIT);
}
