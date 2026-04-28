#include "motion_task.h"
#include "../common/queues.h"
#include "encoder.h"
#include "freertos/task.h"
#include "rmt.h"

#include "utils.h"

const int DBG_PIN = 23; // for debug timing of ISR and motion task loop

int32_t target_position_stepp = 0; // meassured in steps, can be positive or negative depending on direction
int32_t target_speed_Hz = 0;       // signed steps/sec

int32_t volatile current_position_stepp = 0; // meassured in steps, can be positive or negative depending on direction
static int32_t current_stepper_speed_Hz = 0; // signed steps/sec
static volatile bool motionBlockDone = true;
static TaskHandle_t motionTaskHandle = NULL;
static MotionMode motionMode = MOTION_MODE_POSITION;

float mm_per_rev_pitch = 1; // used for setting the speed of the stepper UPDATED FROM UI
static bool pitchChanged = true;
volatile int8_t batches_in_flight = 0; // used to track how many motion batches have been submitted but not yet completed. so we know if we can change direction or not, because we need to stop before changing direction
#define RUNNING (batches_in_flight > 0 || !motionBlockDone)
volatile bool standing_still = true; // this is set to false in the RMT ISR when a batch is completed but we still have more batches in flight, so we know if we can change direction or not, because we need to stop before changing direction
stepper_direction current_direction = DIRECTION_CW;
volatile uint32_t stepsJustDone;

void report_data();
void report_loop_time(uint32_t startTime, uint32_t endTime);
void report_batch_time(uint32_t batchTimeUs);
void report_alarm(bool triggered);
bool handelAlarm();
bool checkForUIUdates();
int32_t get_steps_needed();
int32_t calc_speed_to_order(int32_t error);


static void setStepperEnableLevel(StepperEnableLevel level) {
    digitalWrite(ENABLE_PIN, level);
}

void enableStepper() {
    setStepperEnableLevel(STEPPER_ENABLED);
}

void disableStepper() {
    setStepperEnableLevel(STEPPER_DISABLED);
}

bool isStepperEnabled() {
    return digitalRead(ENABLE_PIN) == STEPPER_ENABLED;
}

static bool uses_preplanned_batches() {
    return motionMode == MOTION_MODE_POSITION;
}

static bool speed_direction_is_positive() {
    return current_direction == DIRECTION_CW;
}



static bool submit_motion_batch(uint32_t steps) {

    loadNextBuffer(steps);

    return true;
}

/*
This callback is called by the RMT driver when it finishes transmitting the pulse sequence.
We use it to check if we've reached the target position and, if not, to continue moving towards the target.
It is also called in between moves, like to initiate the first move towards the target when we receive a new command in the motion task loop.
*/
void IRAM_ATTR onRMTTransmissionComplete(rmt_callback_arg_t *arg) {
    stepsJustDone += arg->steps_done;

    // this will go back inside the motion task loop where we check if we need to prepare and submit the next batch to continue moving towards the target, or if we can stop because we've reached the target
    if (motionTaskHandle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(motionTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// MARK: ALARM
static volatile bool alarmPinChanged = false;

void IRAM_ATTR onAlarmISR() {
    alarmPinChanged = true;
}

// calcualte the speed of the target we are tracking (enocder on spindle)
void update_target_speed() {
    static uint64_t lastMicros = micros();
    static float filteredSpeed = 0.0f;
    uint64_t currentMicros = micros();
    uint64_t deltaMicros = currentMicros - lastMicros;

    // Skip very short intervals to reduce quantization/timer jitter in speed estimate.
    if (deltaMicros < 500) {
        return;
    }

    lastMicros = currentMicros;

    static int32_t lastTargetPos = 0;
    int32_t deltaPos = target_position_stepp - lastTargetPos;
    lastTargetPos = target_position_stepp;

    float instantSpeed = (float)(deltaPos * 1000000) / (float)deltaMicros; // steps per second

    // Exponential moving average to smooth jumpy speed readings.
    static constexpr float kSpeedFilterAlpha = 0.2f;
    filteredSpeed += kSpeedFilterAlpha * (instantSpeed - filteredSpeed);

    if (fabsf(filteredSpeed) < 0.5f) {
        filteredSpeed = 0.0f;
    }

    target_speed_Hz = (int32_t)lroundf(filteredSpeed);
}

void pitch_update(float newPitch) {
    mm_per_rev_pitch = newPitch;
    pitchChanged = true;
}
void set_direction(stepper_direction dir) {
    static stepper_direction prev_dir = dir == DIRECTION_CW ? DIRECTION_CCW : DIRECTION_CW; // guarantee that the first time we call set_direction.

    if (dir != prev_dir) {
        digitalWrite(DIR_PIN, dir);
        prev_dir = dir;
    }

    // current_stepper_speed_Hz = currentSpeed;
}
static inline void apply_steps_to_current_pos(uint32_t steps) {
    if (current_direction == DIRECTION_CW) {
        current_position_stepp += steps;
    } else {
        current_position_stepp -= steps;
    }
}

bool update_direction_if_needed(int32_t error) {
    stepper_direction error_dir = error >= 0 ? DIRECTION_CW : DIRECTION_CCW;
    if (error != 0 && error_dir != current_direction) {
        if (!ready_to_change_direction()) {
            return false; // we need to wait for the current motion batches to finish and the stepper to be standing still before we can change direction, otherwise we would overshoot the target a lot
        }
        set_direction(error_dir);
        current_direction = error_dir;
    }
    return true;
}
// MARK: MAIN
void motionTask(void *pv) {
    motionTaskHandle = xTaskGetCurrentTaskHandle();
    //---------Setting the pins and peripherals---------
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    digitalWrite(DIR_PIN, HIGH); // Set initial direction to positive. speed is 0 so it won't cause an actual dir change if the DIR pin state is already LOW from before, but it will set the currentDir variable in utils.cpp to be in sync with the actual DIR pin state, so we can correctly handle direction changes later when we have speed and need to stop before changing direction

    enableStepper(); // Driver enable is active LOW

    pinMode(DBG_PIN, OUTPUT); // Debug pin for timing the ISR and motion task loop
    bool dbg_value = false;

    // ALM_PIN: active LOW alarm from stepper driver
    pinMode(ALM_PIN, INPUT_PULLUP);
    alarmPinChanged = (digitalRead(ALM_PIN) == STEPPER_ALM_TRIGGERED);   // capture state at boot
    attachInterrupt(digitalPinToInterrupt(ALM_PIN), onAlarmISR, CHANGE); // CHANGE catches both alarm and recovery edges
    report_alarm(alarmPinChanged);                                       // send initial alarm status to UI

    setupEncoder(); // Initialize the encoder interface

    rmt_user_callback = onRMTTransmissionComplete;
    setupRMT((gpio_num_t)PULSE_PIN, RMT_CH); // Initialize the RMT peripheral for generating step pulses
    // delay(10000); // Short delay to ensure RMT is set up before we start sending pulses
    startRMT();

    // setupStepCounter((gpio_num_t)PULSE_WATCH_PIN, (gpio_num_t)DIR_WATCH_PIN);

    //-----------------------------------------------
    bool lastRunning = false;
    bool alarm = false;
    float target_posFactor;
    bool lastAlarmState = false;
    uint32_t lastAlarmReportTime = 0;
    bool targetChanged = true;

    for (;;) {

        static uint64_t lastMicros = micros();
        uint64_t currentMicros = micros();
        uint64_t loopTime = currentMicros - lastMicros;
        lastMicros = currentMicros;
        // Serial.printf("\nLoop time: %u. steps: %d, ", (unsigned int)loopTime, batches[batchIndex].steps); // this is for debug. will remove later
        digitalWrite(DBG_PIN, LOW); // Toggle debug pin to measure loop time with oscilloscope
        // dbg_value = !dbg_value;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for rmt to finish
       digitalWrite(DBG_PIN, HIGH); // Toggle debug pin to measure loop time with oscilloscope
        // dbg_value = !dbg_value;
        
        uint32_t steps = stepsJustDone;
        stepsJustDone = 0;
        
        encoder_update();
        int16_t delta = read_encoder_steps_since_last();
        // calculate the target position in steps from the encoder counts, to know where we are in relation to the target and decide if we need to move towards the target or if we can stop
        static float target;
        target += (float)delta * target_posFactor;
        target_position_stepp = lroundf(target); // convert encoder counts to linear position in steps

        int32_t error = get_steps_needed();

        static stepper_direction lastDirection = DIRECTION_CW;
        stepper_direction error_dir = error >= 0 ? DIRECTION_CW : DIRECTION_CCW;
        if (!update_direction_if_needed(error)) {
            continue;// we need to change dir but are moving. wait until stepper have stopped to change direction
        }

        uint32_t steps_to_take = constrain(abs(error), 0, MAX_RMT_STEPS);
        ;

        // if (abs(error) > 1000) {
        //     Serial.println("Error too big, skipping this batch to avoid overshooting the target");
        //     continue; // this is for debug. will remove later. if the error is too big, we probably had a glitch in the encoder reading or something, so we skip this batch to avoid sending a huge burst of steps that would make us lose the target even more
        // }

        submit_motion_batch(steps_to_take);
        apply_steps_to_current_pos(steps_to_take);

        uint32_t now = micros();
        targetChanged = checkForUIUdates();
        handelAlarm();

        if (pitchChanged) { // if the pitch has changed, we need to update the target_posFactor which is used to convert encoder counts to linear position in steps, so we can correctly track the target position and speed in relation to the spindle rotation
            target_posFactor = mm_per_rev_pitch * STEPS_PER_MM / SPINDLE_ENCODER_COUNT_PER_REV;
            pitchChanged = false;
            // to avoid sudden big error coused by new factor
            current_position_stepp = target_position_stepp;
            
        }

        update_target_speed();

        // -------- PRECOMPUTE --------

        // Serial.println("Calculating next batch...");
        // Serial.printf("Error in steps: %d\n", error); // this is for debug. will remove later
        // int32_t speedHz = calc_speed_to_order(error);

        // Serial.printf("Error: %d, Target speed: %d Hz", error, speedHz); // this is for debug. will remove later

        //------------update UI with current data-----------
        report_data();
    }
}

// MARK: Motion calculations
int32_t get_steps_needed() {
    int32_t error = target_position_stepp - current_position_stepp;
    return error;
}

bool checkForUIUdates() {
    bool updated = false;
    MotionCommand cmd;
    // Check for new motion commands from the motionQueue / UI task
    if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
        if (cmd.cmd == MOTION_CMD_SET_MODE) {
            motionMode = (cmd.mode == MOTION_MODE_FOLLOW) ? MOTION_MODE_FOLLOW : MOTION_MODE_POSITION;
            // pendingBatch.valid = false;
            // Serial.printf("Motion mode: %s\n", motionMode == MOTION_MODE_FOLLOW ? "follow" : "position");
        } else if (cmd.cmd == MOTION_CMD_SET_STEPPER_ENABLE) {
            if (cmd.stepper_enabled) {
                enableStepper();
            } else {
                disableStepper();
            }
            // Serial.printf("Stepper: %s\n", isStepperEnabled() ? "enabled" : "disabled");
        } else if (cmd.cmd == MOTION_CMD_SET_PITCH) {
            //(1.23456) mm/rev should be sent as 123456) in cmd.pitch_1e5_mm_per_rev
            float newPitch = (float)cmd.pitch_1e5_mm_per_rev / 100000.0f;
            if (newPitch > 0.0f) {
                pitch_update(newPitch);
            }
        } else {
            target_position_stepp = cmd.target;
            updated = true;
            // pendingBatch.valid = false;
        }
    }
    return updated;
}
bool handelAlarm() {
    // Check if the alarm state has changed
    bool alarm = false;
    if (alarmPinChanged) {
        alarm = digitalRead(ALM_PIN) == STEPPER_ALM_TRIGGERED;
        report_alarm(alarm);
        alarmPinChanged = false;
    }
    return alarm;
}

void report_data() {

    //-------- For sending data to the UI task --------
    MotionData motionData;
    static MotionDataType lastDataType = POSITION;

    motionData.type = lastDataType;

    switch (lastDataType) {
    case POSITION:
        motionData.value.position = current_position_stepp;
        xQueueOverwrite(UIQueue, &motionData);
        break;
    case SPEED:
        motionData.value.speed = current_stepper_speed_Hz;
        xQueueOverwrite(UIQueue, &motionData);
        break;
    case DIRECTION:
        motionData.value.direction = speed_direction_is_positive() ? 1 : 0;
        xQueueOverwrite(UIQueue, &motionData);
        break;

    case DISTANCE_TO_TARGET:
        motionData.value.distance_to_target = abs(target_position_stepp - current_position_stepp);
        xQueueOverwrite(UIQueue, &motionData);
        break;
    case TARGET_POSITION:
        motionData.value.position = target_position_stepp;
        xQueueOverwrite(UIQueue, &motionData);
        break;
    default:

        break;
    }

    lastDataType = (MotionDataType)((lastDataType + 1) % MOTION_DATA_TYPE_COUNT);
}

void report_loop_time(uint32_t startTime, uint32_t endTime) {
    MotionData motionData = {};
    motionData.type = LOOP_TIME_US;
    motionData.value.loop_time_us = endTime - startTime;
    xQueueOverwrite(UIQueue, &motionData);
}

void report_batch_time(uint32_t batchTimeUs) {
    MotionData motionData = {};
    motionData.type = BATCH_TIME_US;
    motionData.value.batch_time_us = batchTimeUs;
    xQueueOverwrite(UIQueue, &motionData);
}

void report_alarm(bool triggered) {
    MotionData motionData = {};
    motionData.type = ALARM;
    motionData.value.alarm = triggered ? 1 : 0;
    xQueueOverwrite(UIQueue, &motionData);
}
