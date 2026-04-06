#include "utils.h"
#include <Arduino.h>
#include "motion_task.h"


uint32_t Hz2Us(uint32_t freq) {
    // Avoid division by zero and set a default period if frequency is zero
    return freq ? 1000000 / freq : 100;
}