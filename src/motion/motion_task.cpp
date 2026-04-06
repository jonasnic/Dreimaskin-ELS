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
static volatile bool motionBlockDone = true;

// Static RMT buffer to avoid large stack allocation
static rmt_item32_t rmt_buffer[MAX_RMT_STEPS];

static bool speed_direction_is_positive() {
    return currentSpeed >= 0;
}

static void apply_position_delta(uint32_t steps, bool dir) {
    if (dir)
        current_position += steps;
    else
        current_position -= steps;
}

static void generate_motion_block(uint32_t steps, int32_t accel_sign, bool dir) {
    if (steps == 0) return;
    if (steps > MAX_RMT_STEPS) steps = MAX_RMT_STEPS;

    const uint32_t min_period = (uint32_t)(1000000.0f / MAX_SPEED);

    float speed = fabsf((float)currentSpeed);
    if (speed < 1.0f) speed = 1.0f; // only for starting from 0

    for (uint32_t i = 0; i < steps; i++) {
        uint32_t period = Hz2Us((uint32_t)speed);
        if (period < min_period) period = min_period;

        rmt_buffer[i].level0 = 1;
        rmt_buffer[i].duration0 = PULSE_HIGH_TIME_US;
        rmt_buffer[i].level1 = 0;
        rmt_buffer[i].duration1 = period - PULSE_HIGH_TIME_US;

        // update speed per step
        if (accel_sign != 0) {
            speed += (float)accel_sign / speed;

            if (speed < 0.0f) speed = 0.0f; // allow full stop
            if (speed > MAX_SPEED) speed = MAX_SPEED;
        }
    }

    currentSpeed = dir ? (int32_t)speed : -(int32_t)speed;

    // stop running if speed reached 0
    if (currentSpeed == 0)
        running = false;
    else
        running = true;

    if (rmt_write_items(RMT_CH, rmt_buffer, steps, false) == ESP_OK) {
        motionBlockDone = false;
    } else {
        running = false;
        currentSpeed = 0;
        motionBlockDone = true;
    }
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

// stopping distance in steps
static inline uint32_t steps_to_stop(int32_t speed) {
    uint32_t v = abs(speed);
    return (uint64_t)v * v / (2 * ACCELERATION);
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

static void move_toward_target() {
    int32_t dist = target_position - current_position;
    bool dir = (dist > 0);

    // prevent instant reversal at speed
    if ((currentSpeed > 0 && !dir) || (currentSpeed < 0 && dir)) {
        uint32_t block = steps_to_stop(currentSpeed);
        if (block == 0)
            block = 1;
        if (block > MAX_RMT_STEPS)
            block = MAX_RMT_STEPS;

        bool currentDir = speed_direction_is_positive();
        generate_motion_block(block, -ACCELERATION, currentDir);
        apply_position_delta(block, currentDir);
        return;
    }

    if (dist != 0) {
        setDirection(dir);

        uint32_t remaining = abs(dist);
        uint32_t stop_steps = steps_to_stop(currentSpeed);

        uint32_t block = remaining;
        if (block > MAX_RMT_STEPS)
            block = MAX_RMT_STEPS;

        if (stop_steps >= remaining)
            generate_motion_block(block, DECELERATION, dir); // decel
        else if (abs(currentSpeed) < MAX_SPEED)
            generate_motion_block(block, ACCELERATION, dir); // accel
        else
            generate_motion_block(block, 0, dir); // cruise

        apply_position_delta(block, dir);

        running = true;
    } else {
        // target reached → brake if still moving
        if (currentSpeed != 0) {
            uint32_t stop_steps = steps_to_stop(currentSpeed);
            bool currentDir = speed_direction_is_positive();
            generate_motion_block(stop_steps, -ACCELERATION, currentDir);
            apply_position_delta(stop_steps, currentDir);
            running = true;
        } else {
            running = false; // actually stopped
            currentSpeed = 0;
        }
    }
}

/*
This callback is called by the RMT driver when it finishes transmitting the pulse sequence.
We use it to check if we've reached the target position and, if not, to continue moving towards the target.
It is also called in between moves, like to initiate the first move towards the target when we receive a new command in the motion task loop.
*/
void IRAM_ATTR onRMTTransmissionComplete(rmt_channel_t channel, void *arg) {
    if (channel == RMT_CH) {
        motionBlockDone = true;
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
    bool lastRunning = false;
    for (;;) {
        // update_current_position(&current_position);
        bool targetChanged = false;

        if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
            target_position = cmd.target;
            targetChanged = true;
        }
        if (lastRunning != running) {
            Serial.printf("Running: %s\n", running ? "Yes" : "No");
            lastRunning = running;
        }

        if (motionBlockDone || (!running && targetChanged)) {
            move_toward_target();
        }

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