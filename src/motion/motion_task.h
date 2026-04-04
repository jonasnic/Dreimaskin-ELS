#include <Arduino.h>
#include "driver/pcnt.h"
#include "driver/rmt.h"


#define PULSE_PIN 25                           // Pin connected to the stepper PULSE signal
#define PULSE_WATCH_PIN 35                     // Pin used to monitor the pulse signal for PCNT counting
#define DIR_PIN 26                             // Pin connected to the stepper DIR signal
#define DIR_WATCH_PIN 34                       // Pin used to monitor the direction signal for PCNT counting
#define ENABLE_PIN 27                          // Pin connected to the stepper ENABLE signal
#define STEPS_REV 200                          // Number of steps per revolution for the stepper motor
#define MICROSTEPS 16                          // Microstepping setting (e.g., 16 for 1/16 microstepping)
#define STEPS_PER_REV (STEPS_REV * MICROSTEPS) // Total steps per revolution considering microstepping
#define MAX_SPEED 30000                        // Maximum speed in steps per second
#define MIN_SPEED 100                          // Minimum speed in steps per second
#define ACCELERATION 500
#define DECELERATION 1500
#define BELTRATIO 3                                             // Gear ratio of the belt drive (if applicable)
#define AXEL_PITCH 5.0                                          // mm movement per revolution of the axel (e.g., for a lead screw with 5mm pitch)
#define STEPS_PER_MM ((STEPS_PER_REV * BELTRATIO) / AXEL_PITCH) // Steps per millimeter of linear movement


#define RMT_CH RMT_CHANNEL_0
void updateStepper(int32_t &targetCount);
uint32_t Hz2microseconds(uint32_t speedInHz);
void setDirection(bool dir);
uint32_t Update_Speed(uint32_t steps);
void moveSteps(int32_t steps);

void motionTask(void *pv);

#define MAX_RMT_STEPS 60

