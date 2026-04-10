#include "motion_task.h"
#include "../common/queues.h"
#include "encoder.h"
#include "freertos/task.h"
#include "rmt_setup.h"
#include "step_counter.h"
#include "utils.h"

int32_t target_position_stepp = 0; // meassured in steps, can be positive or negative depending on direction
int32_t target_speed_Hz = 0;       // signed steps/sec

int32_t current_position_stepp = 0; // meassured in steps, can be positive or negative depending on direction
// volatile bool running = false;               // Flag to indicate if the motion task is currently executing a move. will use to know if ISR are in effect or not
static int32_t current_stepper_speed_Hz = 0; // signed steps/sec
static volatile bool motionBlockDone = true;
static TaskHandle_t motionTaskHandle = NULL;
static MotionMode motionMode = MOTION_MODE_POSITION;
bool running = false;
static constexpr float kMaxSpeedDeltaPerBatch = 3000.0f;

float mm_per_rev_pitch = 1; // used for setting the speed of the stepper

void report_data();
bool checkForUIUdates();

struct MotionBatch {
    uint16_t steps;
    uint32_t frequencyHz;
    bool dir;
    int32_t endSpeed;
    bool valid;
};
static MotionBatch pendingBatch = {};

static bool uses_preplanned_batches() {
    return motionMode == MOTION_MODE_POSITION;
}

static bool speed_direction_is_positive() {
    return current_stepper_speed_Hz >= 0;
}

static void apply_position_delta(uint32_t steps, bool dir) {
    if (dir)
        current_position_stepp += steps;
    else
        current_position_stepp -= steps;
}

static bool build_motion_batch(uint32_t steps, int32_t accel_sign, bool dir, int32_t startSpeed, MotionBatch *batch) {
    if (steps == 0 || batch == NULL) return false;
    if (steps > MAX_RMT_STEPS) steps = MAX_RMT_STEPS;

    float speed_start = fabsf((float)startSpeed);
    if (speed_start < 1.0f) speed_start = 1.0f; // only for starting from 0

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

    batch->steps = (uint16_t)steps;
    batch->frequencyHz = (uint32_t)speed_start;
    batch->dir = dir;
    batch->endSpeed = dir ? (int32_t)speed_end : -(int32_t)speed_end;
    batch->valid = true;
    return true;
}

static bool submit_motion_batch(const MotionBatch &batch) {
    setDirection(batch.dir, current_stepper_speed_Hz);

    loadNextBufferHz(batch.steps, batch.frequencyHz);

    current_stepper_speed_Hz = batch.endSpeed;
    apply_position_delta(batch.steps, batch.dir);
    motionBlockDone = false;
    running = true;
    return true;
}

// stopping distance in steps
static inline uint32_t steps_to_stop(int32_t speed) {
    uint32_t v = abs(speed);
    return (uint64_t)v * v / (2 * ACCELERATION);
}

static bool plan_motion_batch(int32_t planPosition, int32_t planSpeed, MotionBatch *batch) {
    if (batch == NULL) return false;

    int32_t dist = target_position_stepp - planPosition;
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
        return build_motion_batch(block, -ACCELERATION, currentDir, planSpeed, batch);
    }

    uint32_t stop_steps = steps_to_stop(planSpeed);

    uint32_t block = remaining;
    if (block > MAX_RMT_STEPS)
        block = MAX_RMT_STEPS;

    if (stop_steps >= remaining)
        return build_motion_batch(block, DECELERATION, dir, planSpeed, batch);
    if (abs(planSpeed) < MAX_SPEED)
        return build_motion_batch(block, ACCELERATION, dir, planSpeed, batch);
    return build_motion_batch(block, 0, dir, planSpeed, batch);
}

static void queue_next_motion_batch() {
    MotionBatch batch = {};
    if (!plan_motion_batch(current_position_stepp, current_stepper_speed_Hz, &batch)) {
        running = false;
        current_stepper_speed_Hz = 0;
        return;
    }

    submit_motion_batch(batch);
}

static void prepare_pending_batch() {
    if (!uses_preplanned_batches() || !running || motionBlockDone || pendingBatch.valid) {
        return;
    }

    MotionBatch batch = {};
    if (!plan_motion_batch(current_position_stepp, current_stepper_speed_Hz, &batch)) {
        return;
    }

    pendingBatch = batch;
}

/*
This callback is called by the RMT driver when it finishes transmitting the pulse sequence.
We use it to check if we've reached the target position and, if not, to continue moving towards the target.
It is also called in between moves, like to initiate the first move towards the target when we receive a new command in the motion task loop.
*/
void IRAM_ATTR onRMTTransmissionComplete(rmt_callback_arg_t *arg) {

    motionBlockDone = true;

    // this will go back inside the motion task loop where we check if we need to prepare and submit the next batch to continue moving towards the target, or if we can stop because we've reached the target
    if (motionTaskHandle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(motionTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// calcualte the speed of the target we are tracking (enocder on spindel)
void update_target_speed() {
    static uint64_t lastMicros = micros();
    uint64_t currentMicros = micros();
    uint64_t deltaMicros = currentMicros - lastMicros;
    lastMicros = currentMicros;

    static int32_t lastTargetPos = 0;
    int32_t deltaPos = target_position_stepp - lastTargetPos;
    lastTargetPos = target_position_stepp;

    int32_t newTargetSpeed = (deltaPos * 1000000) / (int64_t)deltaMicros; // steps per second

    target_speed_Hz = newTargetSpeed;
}

// MARK: MAIN
void motionTask(void *pv) {
    motionTaskHandle = xTaskGetCurrentTaskHandle();
    //---------Setting the pins and peripherals---------
    digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor driver
    digitalWrite(DIR_PIN, LOW);    // Set initial direction (e.g., LOW for forward)
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    setupEncoder(); // Initialize the encoder interface

    rmt_user_callback = onRMTTransmissionComplete;
    setupRMT((gpio_num_t)PULSE_PIN, RMT_CH); // Initialize the RMT peripheral for generating step pulses
    // delay(10000); // Short delay to ensure RMT is set up before we start sending pulses
    startRMT();

    setupStepCounter((gpio_num_t)PULSE_WATCH_PIN, (gpio_num_t)DIR_WATCH_PIN);

    //-----------------------------------------------
    bool lastRunning = false;
    int64_t encoderTotal = 0;
    for (;;) {
        static bool targetChanged = false;
        targetChanged = checkForUIUdates();

        encoderTotal += readEncoder_steps_sinse_last();

        target_position_stepp = (int32_t)lroundf(((float)encoderTotal / (float)SPINDEL_ENCODER_COUNT_PER_REV) * mm_per_rev_pitch * (float)STEPS_PER_MM); // convert encoder counts to linear position in steps

        /*  if RMT has completed the previous motion batch and started on  the next one,
            we check if we need to queue the next batch to continue moving towards the target
            or if we can stop because we've reached the target
        */
        bool dir;
        if (motionBlockDone) { // ISR just finished transmitting a batch

            motionBlockDone = false;
            update_target_speed();
            // update_target_position_from_encoder(); // update current_position_stepp with the actual position from the encoder, to correct any drift from missed steps or external forces
            // update_target_speed();

            int32_t error = target_position_stepp - current_position_stepp;

            if (error > 0)
                dir = false;
            else if (error < 0)
                dir = true;
            digitalWrite(DIR_PIN, dir);
            uint32_t stepsToLoad = constrain(abs(error), 0, 64);

            static uint8_t kp_error = 5;
            uint32_t speed = (uint32_t)(target_speed_Hz + error * kp_error); // simple P controller on the error to try to correct it, we can tune the kP gain (0.05 here) to get better performance, or even add integral and derivative terms for a full PID controller if needed

            speed = constrain(speed, MIN_SPEED, MAX_SPEED);
            static int32_t lastSpeed = speed;
            if (lastSpeed < speed) {
                speed = min(speed, lastSpeed + (uint32_t)ACCELERATION);
            } else if (lastSpeed > speed) {
                speed = max(speed, lastSpeed + (uint32_t)DECELERATION);
            }
            lastSpeed = speed;

            const uint32_t target_batch_time_us = 2000;
            uint32_t st = target_batch_time_us / Hz2Us(speed);
            stepsToLoad = min(stepsToLoad, st);

            loadNextBufferHz(stepsToLoad, speed); // this will trigger the onRMTTransmissionComplete callback when done, which will set motionBlockDone to true and allow us to load the next batch if needed
            if (dir)
                current_position_stepp -= stepsToLoad;
            else
            current_position_stepp += stepsToLoad;

            Serial.printf("Target: %d, Current: %d, Error: %d, Speed: %d, Steps loaded: %d\n", target_position_stepp, current_position_stepp, error, speed, stepsToLoad);
            // SOME TEST CODE
            //  static uint32_t frequencyHz = 600;
            //  loadNextBufferHz(64, frequencyHz);//TEST
            //  frequencyHz += 400;
            //  if (frequencyHz > MAX_SPEED) frequencyHz = MAX_SPEED;
        }

        // prepare_pending_batch();
        report_data();

        // Sleep until TX complete notification or timeout; notify path minimizes inter-block gap.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    }
}

bool checkForUIUdates() {
    bool updated = false;
    MotionCommand cmd;
    // Check for new motion commands from the motionQueue / UI task
    if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == MOTION_CMD_SET_MODE) {
            motionMode = (cmd.mode == MOTION_MODE_FOLLOW) ? MOTION_MODE_FOLLOW : MOTION_MODE_POSITION;
            pendingBatch.valid = false;
            Serial.printf("Motion mode: %s\n", motionMode == MOTION_MODE_FOLLOW ? "follow" : "position");
        } else {
            target_position_stepp = cmd.target;
            updated = true;
            pendingBatch.valid = false;
        }
    }
    return updated;
}

void report_data() {
    static uint8_t motionReportingIndex = 0;

    //-------- For sending data to the UI task --------
    MotionData motionData;
    static MotionDataType lastDataType = POSITION;

    if (motionReportingIndex++ == 0) {

        if (motionReportingIndex > 50) motionReportingIndex = 0;
        motionData.type = lastDataType;

        switch (lastDataType) {
        case POSITION:
            motionData.value.position = current_position_stepp;
            xQueueSend(UIQueue, &motionData, 0);
            break;
        case SPEED:
            motionData.value.speed = current_stepper_speed_Hz;
            xQueueSend(UIQueue, &motionData, 0);
            break;
        case DIRECTION:
            motionData.value.direction = speed_direction_is_positive() ? 1 : 0;
            xQueueSend(UIQueue, &motionData, 0);
            break;

        case DISTANCE_TO_TARGET:
            motionData.value.distance_to_target = abs(target_position_stepp - current_position_stepp);
            xQueueSend(UIQueue, &motionData, 0);
            break;
        case TARGET_POSITION:
            motionData.value.position = target_position_stepp;
            xQueueSend(UIQueue, &motionData, 0);
            break;
        default:

            break;
        }
        lastDataType = (MotionDataType)((lastDataType + 1) % MOTION_DATA_TYPE_COUNT);
    }
}