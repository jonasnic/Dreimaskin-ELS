#include "utils.h"
#include <Arduino.h>
#include "motion_task.h"


uint32_t Hz2Us(int32_t freq) {
    const uint32_t magnitude = abs(freq);
    // Avoid division by zero and set a default period if frequency is zero.
    return magnitude ? 1000000U / magnitude : 100;
}

