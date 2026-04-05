#include "motion_task.h"
#include "../common/queues.h"
#include "encoder.h"
#include "rmt_setup.h"
#include "step_counter.h"
#include "utils.h"

volatile int32_t target_position = 0;
volatile int32_t current_position = 0;
volatile bool running = false;   // Flag to indicate if the motion task is currently executing a move. will use to know if ISR are in effect or not
static int32_t currentSpeed = 0; // signed steps/sec

void moveSteps(uint32_t steps) {

    // const uint32_t highTime = 10;
    // uint32_t speed = 3e4; // For testing, set a fixed speed of 10,000 steps per second

    // uint32_t sleepTime = Hz2Us(speed); // Calculate the sleep time based on the desired speed
    // // Serial.printf("Remaining steps: %u | Speed: %u steps/s | Sleep time: %u us\n", remaining, speed, sleepTime);
    // if (sleepTime < highTime * 2) {
    //     sleepTime = highTime * 2; // Ensure the sleep time is at least twice the high time to maintain a proper pulse
    // }
    // uint32_t lowTime = sleepTime - highTime; // Calculate the low time to maintain the total period
    uint32_t speed = Update_Speed(steps);
    uint32_t sleepTime = Hz2Us(speed);             // Calculate the sleep time based on the desired speed
    const uint32_t highTime = 2;                   // Fixed HIGH time for the pulse
    const uint32_t lowTime = sleepTime - highTime; // Fixed LOW time for the pulse (for testing, set to 90us for a total period of 100us, which corresponds to 10,000 steps per second)

    if (steps > MAX_RMT_STEPS)
        steps = MAX_RMT_STEPS;
    rmt_item32_t items[MAX_RMT_STEPS];

    for (uint32_t i = 0; i < steps; i++) {
        items[i].level0 = 1;
        items[i].duration0 = highTime; // HIGH pulse duration
        items[i].level1 = 0;
        items[i].duration1 = lowTime; // LOW pulse duration
    }

    rmt_write_items(RMT_CH, items, steps, false); // Write the pulse sequence to the RMT channel
    bool direction = (target_position - current_position) > 0;
    current_position += direction ? steps : -steps; // Update the current position based on the direction of movement
}

void setDirection(bool dir) {
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

static inline uint32_t stopping_distance(int32_t speed) {
    int32_t v = abs(speed);
    return (v * v) / (2 * DECELERATION);
}

uint32_t Update_Speed(int32_t remainingSteps) {
    int32_t dir = (remainingSteps > 0) ? 1 : -1;

    // int32_t target_speed =
    //     map(remainingSteps,
    //         // map(abs(remainingSteps),
    //         0,
    //         STEPS_PER_REV,
    //         MIN_SPEED,
    //         MAX_SPEED);

    // int32_t target_speed = (MAX_SPEED - MIN_SPEED) / STEPS_PER_REV * abs(remainingSteps) + MIN_SPEED;
    int32_t target_speed = ((int64_t)abs(remainingSteps) * (MAX_SPEED - MIN_SPEED)) / MAX_SPEED + MIN_SPEED;

    // Serial.printf("Remaining steps: %d | Target speed: %d steps/s\n", remainingSteps, target_speed);

    target_speed = constrain(target_speed, MIN_SPEED, MAX_SPEED);

    // signed target velocity
    int32_t targetVelocity = target_speed * dir;

    // low-pass velocity filter (motion smoothing)
    int32_t filter = 50;
    currentSpeed =
        (currentSpeed * (filter - 1) + targetVelocity) / filter;

    // deadband near stop
    if (abs(remainingSteps) < 2)
        currentSpeed = 0;

    return abs(currentSpeed);
}

uint32_t Update_Speed2(int32_t remainingSteps) {
    int32_t dir = (remainingSteps > 0) ? 1 : -1;

    int32_t distance = abs(remainingSteps);

    uint32_t stopDist = stopping_distance(currentSpeed);

    // ---- DECELERATION PHASE ----
    if (stopDist >= distance) {
        if (currentSpeed > 0)
            currentSpeed -= DECELERATION;
        else if (currentSpeed < 0)
            currentSpeed += DECELERATION;
    }
    // ---- ACCELERATION PHASE ----
    else {
        currentSpeed += dir * ACCELERATION;
    }

    // clamp
    if (currentSpeed > MAX_SPEED) currentSpeed = MAX_SPEED;
    if (currentSpeed < -MAX_SPEED) currentSpeed = -MAX_SPEED;

    // avoid deadband stall
    if (currentSpeed != 0 &&
        abs(currentSpeed) < MIN_SPEED)
        currentSpeed = dir * MIN_SPEED;

    return abs(currentSpeed);
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

void move_toward_target() {
    int32_t stepsToMove = target_position - current_position;
    if (stepsToMove != 0) {
        setDirection(stepsToMove > 0); // Set direction based on whether we need to move forward or backward
        running = true;                // Set the running flag to indicate we're executing a move
        moveSteps(abs(stepsToMove));   // Move the required number of steps
    }
}
/*
This callback is called by the RMT driver when it finishes transmitting the pulse sequence.
We use it to check if we've reached the target position and, if not, to continue moving towards the target.
It is also called in between moves, like to initiate the first move towards the target when we receive a new command in the motion task loop.
*/
void onRMTTransmissionComplete(rmt_channel_t channel, void *arg) {
    if (current_position == target_position) {
        running = false;  // Clear the running flag when we reach the target
        currentSpeed = 0; // Ensure speed is set to 0 when we reach the target
    } else {
        move_toward_target(); // Continue moving towards the target if we haven't reached it yet
    }
}
void motionTask(void *pv) {
    MotionCommand cmd;

    //-------- For sending data to the UI task --------
    MotionData motionData;
    motionData.type = POSITION;

    MotionData speedData;
    speedData.type = SPEED;
    //-----------------------------------------------

    //---------Seting the pins and peripherals---------
    digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor driver
    digitalWrite(DIR_PIN, LOW);    // Set initial direction (e.g., LOW for forward)
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    setupEncoder();                                                     // Initialize the encoder interface
    setupRMT((gpio_num_t)PULSE_PIN, RMT_CH, onRMTTransmissionComplete); // Initialize the RMT peripheral for generating step pulses
    setupStepCounter((gpio_num_t)PULSE_WATCH_PIN, (gpio_num_t)DIR_WATCH_PIN);
    //-----------------------------------------------

    for (;;) {
        // update_current_position(&current_position);

        if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
            target_position = cmd.target;
        }
        // move_toward_target();
        if (!running) {
            onRMTTransmissionComplete(RMT_CH, NULL); // Manually call the RMT transmission complete callback to start the motion towards the target. In a real implementation, this would be triggered by the RMT hardware when it finishes sending pulses.
        }
        // runMotionPlanner(target);

        static int i = 0;
        if (i++ % 100 == 0) {

            motionData.value.position = current_position;
            xQueueSend(UIQueue, &motionData, 0);

            speedData.value.speed = currentSpeed;
            xQueueSend(UIQueue, &speedData, 0);
        }

        vTaskDelay(1); // yields CPU but keeps timing stable
    }
}