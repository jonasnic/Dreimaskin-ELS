#include "utils.h"
#include <Arduino.h>
#include "motion_task.h"


uint32_t Hz2Us(uint32_t freq) {
    // Avoid division by zero and set a default period if frequency is zero
    return freq ? 1000000 / freq : 100;
}

void set_direction(bool direction) {
    static bool current_direction = digitalRead(DIR_PIN); // Keep track of the current direction to avoid unnecessary pin writes
    if (current_direction != direction) {
        digitalWrite(DIR_PIN, direction ? HIGH : LOW);
        current_direction = direction;
        delay(1); // Short delay to ensure the direction change is registered by the driver before sending pulses
    }
}