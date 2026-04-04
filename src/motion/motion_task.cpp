#include "motion_task.h"
#include "../common/queues.h"
#include "encoder.h"
#include "rmt_setup.h"
#include "step_counter.h"
#include "utils.h"

int fill_rmt_buffer(rmt_item32_t *buffer, uint32_t steps, uint32_t speed) {
    const uint32_t highTime = 10; // Fixed HIGH time for the pulse

    if (steps > MAX_RMT_STEPS) {
        steps = MAX_RMT_STEPS; // Limit steps to the maximum buffer size
    }

    uint32_t sleepTime = Hz2Us(speed); // Calculate the total period based on the desired speed
    if (sleepTime < highTime * 2) {
        sleepTime = highTime * 2; // Ensure the sleep time is at least twice the high time to maintain a proper pulse
    }
    uint32_t lowTime = sleepTime - highTime; // Calculate the LOW time to maintain the total period

    for (uint32_t i = 0; i < steps; i++) {
        buffer[i].level0 = 1;
        buffer[i].duration0 = highTime; // HIGH pulse duration
        buffer[i].level1 = 0;
        buffer[i].duration1 = lowTime; // LOW pulse duration
    }

    return steps; // Return the number of steps filled in the buffer
}

void moveSteps(uint32_t steps) {

    const uint32_t highTime = 10;
    uint32_t speed = 3e4; // For testing, set a fixed speed of 10,000 steps per second

    uint32_t sleepTime = Hz2Us(speed); // Calculate the sleep time based on the desired speed
    // Serial.printf("Remaining steps: %u | Speed: %u steps/s | Sleep time: %u us\n", remaining, speed, sleepTime);
    if (sleepTime < highTime * 2) {
        sleepTime = highTime * 2; // Ensure the sleep time is at least twice the high time to maintain a proper pulse
    }
    uint32_t lowTime = sleepTime - highTime; // Calculate the low time to maintain the total period

    if (steps > MAX_RMT_STEPS)
        steps = MAX_RMT_STEPS;
    rmt_item32_t items[MAX_RMT_STEPS];

    for (uint32_t i = 0; i < steps; i++) {
        items[i].level0 = 1;
        items[i].duration0 = highTime; // HIGH pulse duration
        items[i].level1 = 0;
        items[i].duration1 = lowTime; // LOW pulse duration
    }

    rmt_write_items(RMT_CH, items, steps, true); // Write the pulse sequence to the RMT channel

    // while (remaining > 0) {
    //     // uint32_t speed = Update_Speed(remaining);
    //     uint32_t speed = 1e4; // For testing, set a fixed speed of 10,000 steps per second

    //     uint32_t sleepTime = Hz2Us(speed); // Calculate the sleep time based on the desired speed
    //     // Serial.printf("Remaining steps: %u | Speed: %u steps/s | Sleep time: %u us\n", remaining, speed, sleepTime);
    //     if (sleepTime < highTime * 2) {
    //         sleepTime = highTime * 2; // Ensure the sleep time is at least twice the high time to maintain a proper pulse
    //     }

    //     uint32_t lowTime = sleepTime - highTime; // Calculate the low time to maintain the total period
    //     uint32_t chunk = remaining;
    //     if (chunk > MAX_RMT_STEPS)
    //         chunk = MAX_RMT_STEPS;

    //     rmt_item32_t items[MAX_RMT_STEPS];

    //     for (uint32_t i = 0; i < chunk; i++) {
    //         items[i].level0 = 1;
    //         items[i].duration0 = highTime; // HIGH pulse duration
    //         items[i].level1 = 0;
    //         items[i].duration1 = lowTime; // LOW pulse duration
    //     }

    //     rmt_write_items(RMT_CH, items, chunk, true); // Write the pulse sequence to the RMT channel

    //     remaining -= chunk;
    // }
}

void setDirection(bool dir) {
    static bool currentDir = digitalRead(DIR_PIN); // Read the initial direction from the DIR pin

    if (currentDir != dir) {
        digitalWrite(DIR_PIN, dir);
        delayMicroseconds(50); // REQUIRED
        currentDir = dir;
    }
}

uint32_t Update_Speed(uint32_t steps) {
    int32_t TargetSpeed =
        MIN_SPEED +
        (int64_t)steps * (MAX_SPEED - MIN_SPEED) / STEPS_PER_REV;

    TargetSpeed = constrain(TargetSpeed, MIN_SPEED, MAX_SPEED);

    static int32_t currentSpeed = MIN_SPEED;

    if (currentSpeed < TargetSpeed) {
        currentSpeed += ACCELERATION;
        if (currentSpeed > TargetSpeed)
            currentSpeed = TargetSpeed;
    } else if (currentSpeed > TargetSpeed) {
        currentSpeed -= DECELERATION;
        if (currentSpeed < TargetSpeed)
            currentSpeed = TargetSpeed;
    }

    // HARD SAFETY CLAMP
    currentSpeed = constrain(currentSpeed, MIN_SPEED, MAX_SPEED);

    return currentSpeed;
}

void loopMotionTask() {

    int16_t StepCount;
    pcnt_get_counter_value(STEP_COUNTER_PCNT_UNIT, &StepCount);

    static int16_t lastStepCount = 0; // persist between loops
    int16_t stepDelta;

    stepDelta = StepCount - lastStepCount;
    if (stepDelta > 30000) {
        stepDelta -= INT16_MIN;
    }

    if (stepDelta < -30000) {
        stepDelta += INT16_MAX;
    }
    static int32_t totalSteps = 0;
    totalSteps += stepDelta;

    static int i = 0;
}

void runMotionPlanner(int32_t &targetCount) {
    // This function can implement a motion planning algorithm to smoothly move towards the target count
    // For simplicity, we will just call updateStepper in this example, but you could add acceleration profiles, etc. here
    // updateStepper(targetCount);
}
void update_current_position(int32_t *current_position) {
    int16_t StepCount;
    pcnt_get_counter_value(STEP_COUNTER_PCNT_UNIT, &StepCount);

    static int16_t lastStepCount = 0; // persist between calls
    int16_t stepDelta;

    stepDelta = StepCount - lastStepCount;
    if (stepDelta > 30000) {
        stepDelta -= INT16_MIN;
    }

    if (stepDelta < -30000) {
        stepDelta += INT16_MAX;
    }
    *current_position += stepDelta;
    lastStepCount = StepCount;
}

void move_toward_target(int32_t *target, int32_t *current_position) {
    int32_t stepsToMove = *target - *current_position;
    if (stepsToMove != 0) {
        setDirection(stepsToMove > 0); // Set direction based on whether we need to move forward or backward
        moveSteps(abs(stepsToMove));   // Move the required number of steps
    }
}

void motionTask(void *pv) {
    MotionCommand cmd;

    int32_t target_position = 0;
    int32_t current_position = 0;

    // Any initialization code for the motion task can go here
    // digitalWrite(PULSE_PIN, LOW);     // Ensure the PULSE pin is LOW at startup
    digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor driver
    digitalWrite(DIR_PIN, HIGH);   // Set initial direction (e.g., LOW for forward)
    // pinMode(PULSE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    setupEncoder();                                // Initialize the encoder interface
    setupRMT((gpio_num_t)PULSE_PIN, RMT_CH, NULL); // Initialize the RMT peripheral for generating step pulses
    setupStepCounter((gpio_num_t)PULSE_WATCH_PIN, (gpio_num_t)DIR_WATCH_PIN);

    for (;;) {
        update_current_position(&current_position);

        if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
            target_position = cmd.target;
        }
        move_toward_target(&target_position, &current_position);

        // runMotionPlanner(target);

        static int i = 0;
        if(i++ % 100 == 0) {
            MotionData data;
            data.type = POSITION;
            data.value.position = current_position;
            xQueueSend(UIQueue, &data, 0);
        }

        vTaskDelay(1); // yields CPU but keeps timing stable
    }
}