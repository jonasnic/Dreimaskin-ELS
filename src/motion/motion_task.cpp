#include "motion_task.h"
#include "../common/queues.h"
#include "encoder.h"
#include "freertos/task.h"
#include "rmt_setup.h"
#include "step_counter.h"
#include "utils.h"

volatile int32_t target_position = 0;
volatile int32_t current_position = 0;
volatile bool running = false;   // Flag to indicate if the motion task is currently executing a move. will use to know if ISR are in effect or not
static int32_t currentSpeed = 0; // signed steps/sec
static volatile bool motionBlockDone = true;
static TaskHandle_t motionTaskHandle = NULL;
static MotionMode motionMode = MOTION_MODE_POSITION;
static constexpr float kMaxSpeedDeltaPerBatch = 3000.0f;
static constexpr uint32_t kMaxPulsePeriodUs = 2000;

// Ping-pong buffers reduce gaps by avoiding write/read contention on the same buffer.
static rmt_item32_t rmt_buffers[2][MAX_RMT_STEPS];
static uint8_t activeBufferIndex = 1;

struct MotionBatch {
    uint8_t bufferIndex;
    uint16_t steps;
    bool dir;
    int32_t endSpeed;
    bool valid;
};

static MotionBatch pendingBatch = {};

static bool uses_preplanned_batches() {
    return motionMode == MOTION_MODE_POSITION;
}

static bool speed_direction_is_positive() {
    return currentSpeed >= 0;
}

static void apply_position_delta(uint32_t steps, bool dir) {
    if (dir)
        current_position += steps;
    else
        current_position -= steps;
}

static uint8_t choose_buffer_index() {
    return activeBufferIndex ^ 1;
}

static bool build_motion_batch(uint8_t bufferIndex, uint32_t steps, int32_t accel_sign, bool dir, int32_t startSpeed, MotionBatch *batch) {
    if (steps == 0 || batch == NULL) return false;
    if (steps > MAX_RMT_STEPS) steps = MAX_RMT_STEPS;

    const uint32_t min_period = (uint32_t)(1000000.0f / MAX_SPEED);

    float speed_start = fabsf((float)startSpeed);
    if (speed_start < 1.0f) speed_start = 1.0f; // only for starting from 0

    rmt_item32_t *buffer = rmt_buffers[bufferIndex];

    // Constant pulse period for this whole batch; speed is updated once after the batch.
    uint32_t period = Hz2Us((uint32_t)speed_start);
    if (period > kMaxPulsePeriodUs) period = kMaxPulsePeriodUs;
    if (period < min_period) period = min_period;

    for (uint32_t i = 0; i < steps; i++) {
        buffer[i].level0 = 1;
        buffer[i].duration0 = PULSE_HIGH_TIME_US;
        buffer[i].level1 = 0;
        buffer[i].duration1 = period - PULSE_HIGH_TIME_US;
    }

    float speed_end = speed_start;
    if (accel_sign != 0) {
        // Batch-level acceleration update (no per-step acceleration calculation).
        float dv = ((float)accel_sign * (float)steps) / speed_start;
        if (dv > kMaxSpeedDeltaPerBatch) dv = kMaxSpeedDeltaPerBatch;
        if (dv < -kMaxSpeedDeltaPerBatch) dv = -kMaxSpeedDeltaPerBatch;
        speed_end += dv;
    }

    if (speed_end < 0.0f) speed_end = 0.0f;
    if (speed_end > MAX_SPEED) speed_end = MAX_SPEED;

    batch->bufferIndex = bufferIndex;
    batch->steps = (uint16_t)steps;
    batch->dir = dir;
    batch->endSpeed = dir ? (int32_t)speed_end : -(int32_t)speed_end;
    batch->valid = true;
    return true;
}

static bool submit_motion_batch(const MotionBatch &batch) {
    setDirection(batch.dir);

    if (rmt_write_items(RMT_CH, rmt_buffers[batch.bufferIndex], batch.steps, false) == ESP_OK) {
        activeBufferIndex = batch.bufferIndex;
        currentSpeed = batch.endSpeed;
        apply_position_delta(batch.steps, batch.dir);
        motionBlockDone = false;
        running = true;
        return true;
    } else {
        running = false;
        currentSpeed = 0;
        motionBlockDone = true;
        return false;
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

static bool plan_motion_batch(int32_t planPosition, int32_t planSpeed, MotionBatch *batch) {
    if (batch == NULL) return false;

    int32_t dist = target_position - planPosition;
    uint32_t remaining = abs(dist);
    bool dir = (dist > 0);

    if (remaining == 0) {
        batch->valid = false;
        return false;
    }

    // prevent instant reversal at speed
    if ((planSpeed > 0 && !dir) || (planSpeed < 0 && dir)) {
        uint32_t block = steps_to_stop(planSpeed);
        if (block == 0)
            block = 1;
        if (block > MAX_RMT_STEPS)
            block = MAX_RMT_STEPS;

        bool currentDir = planSpeed >= 0;
        return build_motion_batch(choose_buffer_index(), block, -ACCELERATION, currentDir, planSpeed, batch);
    }

    uint32_t stop_steps = steps_to_stop(planSpeed);

    uint32_t block = remaining;
    if (block > MAX_RMT_STEPS)
        block = MAX_RMT_STEPS;

    if (stop_steps >= remaining)
        return build_motion_batch(choose_buffer_index(), block, DECELERATION, dir, planSpeed, batch);
    if (abs(planSpeed) < MAX_SPEED)
        return build_motion_batch(choose_buffer_index(), block, ACCELERATION, dir, planSpeed, batch);
    return build_motion_batch(choose_buffer_index(), block, 0, dir, planSpeed, batch);
}

static void queue_next_motion_batch() {
    MotionBatch batch = {};
    if (!plan_motion_batch(current_position, currentSpeed, &batch)) {
        running = false;
        currentSpeed = 0;
        return;
    }

    submit_motion_batch(batch);
}

static void prepare_pending_batch() {
    if (!uses_preplanned_batches() || !running || motionBlockDone || pendingBatch.valid) {
        return;
    }

    MotionBatch batch = {};
    if (!plan_motion_batch(current_position, currentSpeed, &batch)) {
        return;
    }

    pendingBatch = batch;
}

/*
This callback is called by the RMT driver when it finishes transmitting the pulse sequence.
We use it to check if we've reached the target position and, if not, to continue moving towards the target.
It is also called in between moves, like to initiate the first move towards the target when we receive a new command in the motion task loop.
*/
void IRAM_ATTR onRMTTransmissionComplete(rmt_channel_t channel, void *arg) {
    if (channel == RMT_CH) {
        motionBlockDone = true;
        if (motionTaskHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(motionTaskHandle, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

void motionTask(void *pv) {
    motionTaskHandle = xTaskGetCurrentTaskHandle();

    MotionCommand cmd;

    //-------- For sending data to the UI task --------
    MotionData motionData;

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
    uint8_t motionReportingIndex = 0;
    MotionDataType lastDataType = POSITION;
    for (;;) {
        bool targetChanged = false;

        if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
            if (cmd.cmd == MOTION_CMD_SET_MODE) {
                motionMode = (cmd.mode == MOTION_MODE_FOLLOW) ? MOTION_MODE_FOLLOW : MOTION_MODE_POSITION;
                pendingBatch.valid = false;
                Serial.printf("Motion mode: %s\n", motionMode == MOTION_MODE_FOLLOW ? "follow" : "position");
            } else {
                target_position = cmd.target;
                targetChanged = true;
                pendingBatch.valid = false;

                if (!running && motionBlockDone) {
                    currentSpeed = 0;
                }
            }
        }
        if (lastRunning != running) {
            Serial.printf("Running: %s\n", running ? "Yes" : "No");
            lastRunning = running;
        }

        if (motionBlockDone) {
            if (uses_preplanned_batches() && pendingBatch.valid) {
                MotionBatch batch = pendingBatch;
                pendingBatch.valid = false;
                submit_motion_batch(batch);
            } else {
                queue_next_motion_batch();
            }
        } else if (!running && targetChanged) {
            queue_next_motion_batch();
        }

        prepare_pending_batch();

        if (motionReportingIndex++ == 0) {
            if (motionReportingIndex > 50) motionReportingIndex = 0;
            motionData.type = lastDataType;

            switch (lastDataType) {
            case POSITION:
                motionData.value.position = current_position;
                xQueueSend(UIQueue, &motionData, 0);
                break;
            case SPEED:
                motionData.value.speed = currentSpeed;
                xQueueSend(UIQueue, &motionData, 0);
                break;
            case DIRECTION:
                motionData.value.direction = speed_direction_is_positive() ? 1 : 0;
                xQueueSend(UIQueue, &motionData, 0);
                break;

            case DISTANCE_TO_TARGET:
                motionData.value.distance_to_target = abs(target_position - current_position);
                xQueueSend(UIQueue, &motionData, 0);
                break;
            case TARGET_POSITION:
                motionData.value.position = target_position;
                xQueueSend(UIQueue, &motionData, 0);
                break;
            default:

                break;
            }
            lastDataType = (MotionDataType)((lastDataType + 1) % MOTION_DATA_TYPE_COUNT);
        }

        // Sleep until TX complete notification or timeout; notify path minimizes inter-block gap.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    }
}