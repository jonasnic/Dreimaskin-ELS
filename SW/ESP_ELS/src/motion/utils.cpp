#include "utils.h"
#include <Arduino.h>
#include "motion_task.h"


uint32_t Hz2Us(uint32_t freq) {
    // Avoid division by zero and set a default period if frequency is zero
    return freq ? 1000000 / freq : 100;
}

void setDirection(bool dir,uint32_t currentSpeed) {
    static bool currentDir = digitalRead(DIR_PIN); // Read the initial direction from the DIR pin

    if (currentDir != dir) {
        if (currentSpeed == 0) {
            // If we're currently moving, we need to stop before changing direction

            digitalWrite(DIR_PIN, dir);
            delayMicroseconds(50); // REQUIRED
            currentDir = dir;
        }
    }
}