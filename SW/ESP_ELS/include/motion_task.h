#include "driver/pcnt.h"
#include "driver/rmt.h"
#include <Arduino.h>

constexpr uint32_t constexpr_min_u32(uint32_t a, uint32_t b) {
	return (a < b) ? a : b;
}

constexpr uint8_t PULSE_PIN = 25;       // Pin connected to the stepper PULSE signal
// constexpr uint8_t PULSE_WATCH_PIN = 35; // Pin used to monitor the pulse signal for PCNT counting
constexpr uint8_t DIR_PIN = 26;         // Pin connected to the stepper DIR signal
constexpr uint8_t DIR_WATCH_PIN = 34;   // Pin used to monitor the direction signal for PCNT counting
constexpr uint8_t ENABLE_PIN = 27;      // Pin connected to the stepper ENABLE signal

constexpr uint32_t STEPS_REV = 10000;                                 // Number of steps per revolution for the stepper motor
constexpr uint8_t MICROSTEPS = 1;                                  // Microstepping setting (e.g., 16 for 1/16 microstepping)
constexpr uint32_t STEPS_PER_REV = (STEPS_REV * MICROSTEPS);        // Total steps per revolution considering microstepping
constexpr uint32_t MAX_STEPPER_RPM = 1500;                                                   // Maximum revolutions per minute of the spin
constexpr uint32_t MAX_SPEEDHZ = constexpr_min_u32(MAX_STEPPER_RPM * STEPS_PER_REV / 60u, 250000u);                               // Maximum speed in steps per second
constexpr uint32_t MIN_SPEED = 1000;                                // Minimum speed in steps per second
constexpr uint32_t MIN_SPEED_PERIODE = (uint32_t)(1e6 / MAX_SPEEDHZ); // Minimum pulse period in microseconds corresponding to the maximum speed
constexpr uint32_t CAN_STOP_SPEED = 500;                            // Speed below which we can reliably stop within one batch

constexpr uint32_t ACCELERATION_RPSS= 10; // Acceleration in revolutions per minute per second
constexpr uint32_t ACCELERATION = (uint32_t)(((float)ACCELERATION_RPSS * STEPS_PER_REV) / 60.0f); // Acceleration in steps per second squared
constexpr int32_t DECELERATION = -ACCELERATION;
constexpr uint8_t BELTRATIO = 3;                                                    // Gear ratio of the belt drive (if applicable)
constexpr uint32_t AXEL_PITCH = 3770;                                               // µm                                             // migth need to channge to float or use int and scale    // mm movement per revolution of the axel (e.g., for a lead screw with 5mm pitch)
constexpr float STEPS_PER_MM = (((STEPS_PER_REV * BELTRATIO) * 1000) / AXEL_PITCH); // Steps per millimeter of linear movement
constexpr float MM_PER_STEP = (1.0f / STEPS_PER_MM);                                // Millimeters of linear movement per step, used for calculating speed in mm/s from speed in steps/s
constexpr uint16_t PULSE_HIGH_TIME_US = 4;
constexpr uint32_t BACKLASH_STEPS = 0; // Steps to compensate for leadscrew backlash on direction change; tune for your machine

uint32_t Hz2Us(uint32_t speedInHz);
void setDirection(bool dir);

void motionTask(void *pv);

// RMT channel for Stepper control
constexpr rmt_channel_t RMT_CH = RMT_CHANNEL_0;
// how many steps fit in one RMT buffer
constexpr uint8_t MAX_RMT_STEPS = 64;
