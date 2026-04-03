#pragma once

#include "driver/pcnt.h"

constexpr pcnt_unit_t STEP_COUNTER_PCNT_UNIT = PCNT_UNIT_1;

void setupStepCounter(gpio_num_t pulsePin, gpio_num_t dirPin);
