#include "driver/pcnt.h"
#include "driver/rmt.h"
#include <Arduino.h>
#include "config.h"

constexpr uint32_t constexpr_min_u32(uint32_t a, uint32_t b) {
    return (a < b) ? a : b;
}

constexpr uint8_t PULSE_PIN = 25; // Pin connected to the stepper PULSE signal
// constexpr uint8_t PULSE_WATCH_PIN = 35; // Pin used to monitor the pulse signal for PCNT counting
constexpr uint8_t DIR_PIN = 26;    // Pin connected to the stepper DIR signal
constexpr uint8_t ALM_PIN = 34;    // Pin connected to the stepper ALM signal (if available, used for error detection)
constexpr uint8_t ENABLE_PIN = 27; // Pin connected to the stepper ENABLE signal

constexpr int32_t MAX_STEPPER_RPM = 1500;                                                                          // Maximum revolutions per minute of the spin
//constexpr int32_t UPDATE_RATE_HZ = 500;                                                                            // Update rate in Hz for motion control
constexpr int32_t TARGET_BATCH_TIME = UPDATE_PERIOD_US;                                                    // 1s/hz Target time in microseconds for each motion batch. 500 is in Hz
constexpr int32_t MAX_HZ_FOR_BATCH_TIME = (1000000U / ((float)TARGET_BATCH_TIME / 64.0f));                         // Maximum speed in Hz that allows for the batch time, used to limit the speed when we want to keep a consistent update rate
constexpr int32_t MAX_SPEEDHZ = constexpr_min_u32((MAX_STEPPER_RPM / 60u) * STEPS_REV, MAX_HZ_FOR_BATCH_TIME); // Maximum speed in steps per second


constexpr int32_t ACCELERATION_RPSS = 10;                             // Acceleration in revolutions per second per second
constexpr int32_t ACCELERATION = (ACCELERATION_RPSS * STEPS_REV); // Acceleration in steps per second squared
constexpr int32_t DECELERATION = -ACCELERATION;

constexpr int8_t BELTRATIO = 3;                                                     // Gear ratio of the belt drive (if applicable)
constexpr uint32_t AXEL_PITCH = 4000;                                               // µm// migth need to channge to float or use int and scale    // mm movement per revolution of the axel (e.g., for a lead screw with 5mm pitch)
constexpr float STEPS_PER_MM = (((STEPS_REV * BELTRATIO) * 1000) / AXEL_PITCH); // Steps per millimeter of linear movement
constexpr float MM_PER_STEP = (1.0f / STEPS_PER_MM);                                // Millimeters of linear movement per step, used for calculating speed in mm/s from speed in steps/s
constexpr uint16_t PULSE_HIGH_TIME_US = 2;


uint32_t Hz2Us(int32_t speedInHz);
void setDirection(bool dir, int32_t currentSpeed = 0);
void enableStepper();
void disableStepper();
bool isStepperEnabled();

void motionTask(void *pv);

// RMT channel for Stepper control
constexpr rmt_channel_t RMT_CH = RMT_CHANNEL_0;
// how many steps fit in one RMT buffer
constexpr uint8_t MAX_RMT_STEPS = 64;

enum STEPPER_ALARM_STATE {
    STEPPER_ALM_OK = HIGH,
    STEPPER_ALM_TRIGGERED = LOW
};

enum StepperEnableLevel : uint8_t {
    STEPPER_ENABLED = LOW,
    STEPPER_DISABLED = HIGH,
};
enum stepper_direction {
    DIRECTION_CW = HIGH,
    DIRECTION_CCW = LOW,
};
