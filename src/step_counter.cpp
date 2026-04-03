#include "step_counter.h"

void setupStepCounter(gpio_num_t pulsePin, gpio_num_t dirPin) {
    pcnt_config_t pcntConfig{
        .pulse_gpio_num = pulsePin,
        .ctrl_gpio_num = dirPin,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = INT16_MAX,
        .counter_l_lim = INT16_MIN,
        .unit = STEP_COUNTER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0};

    pcnt_unit_config(&pcntConfig);
    pcnt_counter_pause(STEP_COUNTER_PCNT_UNIT);
    pcnt_counter_clear(STEP_COUNTER_PCNT_UNIT);
    pcnt_counter_resume(STEP_COUNTER_PCNT_UNIT);
}
