#include "driver/pcnt.h"
#include "driver/rmt.h"
// #include "esp_log.h"
#include "encoder.h"
#include "rmt_setup.h"
#include "step_counter.h"
#include <Arduino.h>

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
#define MAX_RMT_STEPS 60

enum Direction {
    FORWARD,
    REVERSE
};
enum enable {
    ENABLE,
    DISABLE,
};

void moveSteps(uint32_t steps) {
    uint32_t remaining = steps;
    // uint32_t sleepTime = Hz2microseconds(speed); // Calculate the sleep time based on the desired speed
    // sleepTime = 100;                             // For testing, set a fixed sleep time of 100 microseconds (10 kHz pulse frequency)

    const uint32_t highTime = 10;

    // Serial.print("Moving ");
    // Serial.print(steps);
    // Serial.print(" steps at ");
    // Serial.print(speed);
    // Serial.print(" steps/s (sleep time: ");
    // Serial.print(sleepTime);
    // Serial.println(" us)");
    while (remaining > 0) {
        uint32_t speed = Update_Speed(remaining);

        uint32_t sleepTime = Hz2microseconds(speed); // Calculate the sleep time based on the desired speed
        // Serial.printf("Remaining steps: %u | Speed: %u steps/s | Sleep time: %u us\n", remaining, speed, sleepTime);
        if (sleepTime < highTime * 2) {
            sleepTime = highTime * 2; // Ensure the sleep time is at least twice the high time to maintain a proper pulse
        }
        
        uint32_t lowTime = sleepTime - highTime; // Calculate the low time to maintain the total period
        uint32_t chunk = remaining;
        if (chunk > MAX_RMT_STEPS)
            chunk = MAX_RMT_STEPS;

        rmt_item32_t items[MAX_RMT_STEPS];

        for (uint32_t i = 0; i < chunk; i++) {
            items[i].level0 = 1;
            items[i].duration0 = highTime; // HIGH pulse duration
            items[i].level1 = 0;
            items[i].duration1 = lowTime; // LOW pulse duration
        }

        rmt_write_items(RMT_CH, items, chunk, true); // Write the pulse sequence to the RMT channel

        remaining -= chunk;
    }
}

void setup() {
    // digitalWrite(PULSE_PIN, LOW);     // Ensure the PULSE pin is LOW at startup
    digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor driver
    digitalWrite(DIR_PIN, HIGH);   // Set initial direction (e.g., LOW for forward)
    // pinMode(PULSE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    Serial.begin(115200); // Start serial communication for debugging

    setupEncoder();                          // Initialize the encoder interface
    setupRMT((gpio_num_t)PULSE_PIN, RMT_CH); // Initialize the RMT peripheral for generating step pulses
    setupStepCounter((gpio_num_t)PULSE_WATCH_PIN, (gpio_num_t)DIR_WATCH_PIN);

    // int8_t dum1, dum2, delta;

    // for (int i = 0; i < 300; i++) {
    //     dum1 +=3 ;
    //     delta = dum1 - dum2;
    //     Serial.print("dum1: ");
    //     Serial.print(dum1);
    //     Serial.print(" | dum2: ");
    //     Serial.print(dum2);
    //     Serial.print(" | delta: ");
    //     Serial.println(delta);
    //     dum2 = dum1;
    //   }
}

void loop() {
    // const int16_t StepsPrEncoderPuls = (MICROSTEPS * STEPS_PER_REV) / SPINDEL_ENCODER_PPR; // Calculate the number of steps per encoder revolution based on PPR and microstepping
    // static int32_t targetStepCount = 0;                                                    // Variable to store the target step count for the stepper motor
    // static int16_t lastEncoderCount = 0;
    // int16_t encoderCount;
    // pcnt_get_counter_value(SPINDEL_ENCODER_PCNT_UNIT, &encoderCount); // Get the current count from the pulse counter
    // if (encoderCount != lastEncoderCount) {                           // Check if the encoder count has changed
    //     int16_t countChange = encoderCount - lastEncoderCount;        // Calculate the change in encoder count since the last reading
    //     lastEncoderCount = encoderCount;                              // Print the change in encoder count to the serial monitor
    //     Serial.print("Encoder Count: ");
    //     Serial.println(encoderCount);

    //     // Add to targer
    //     targetStepCount += countChange * StepsPrEncoderPuls; // Update the target step count based on the change in encoder count
    // }

    // updateStepper(targetStepCount); // Update the stepper motor position to move towards the target step count

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
    // if (lastStepCount != StepCount) { // Print the step count every 10 changes for debugging
    //     Serial.print("Last Step Count: ");
    //     Serial.print(lastStepCount);
    //     Serial.print(" | Current Step Count: ");
    //     Serial.print(StepCount);
    //     Serial.print(" | Step Delta: ");
    //     Serial.print(stepDelta);
    //     Serial.print(" | Total Steps: ");
    //     Serial.println(totalSteps);
    // }

    static char command[20];
    static byte commandIndex = 0;
    static bool commandReady = false;
    if (Serial.available() > 0) {

        // size_t len = Serial.readBytesUntil('\n', command, sizeof(command) - 1);

        // command[len] = '\0';
        command[commandIndex] = Serial.read();
        Serial.print(command[commandIndex]);

        if (command[commandIndex] == '\n') {
            command[commandIndex] = '\0'; // Null-terminate the command string
            commandIndex = 0;             // Reset index for the next command
            commandReady = true;          // Set flag to indicate a complete command is ready

        } else {
            commandIndex++; // Move to the next index for the next character
        }

        if (commandReady) {
          Serial.print("Received command: ");
          Serial.println(command);
            int value = atoi(&command[1]);
            switch (command[0]) {

            case 's':
                moveSteps(value);
                break;
            case 'd':
                setDirection(value ? HIGH : LOW);
                break;
            case 'e':
                Serial.print("Setting ENABLE to ");
                Serial.println(value ? "HIGH (DISABLE)" : "LOW (ENABLE)");
                digitalWrite(ENABLE_PIN, value ? HIGH : LOW);
                break;
            }
        }
        // command[0] = '\0'; // Clear the command buffer after processing

        commandReady = false; // Reset the flag after processing the command
    }
    lastStepCount = StepCount;
    // moveSteps(STEPS_PER_REV * 10);        // Move the stepper motor by 100 steps at a speed of 1000 steps per second
    // setDirection(!digitalRead(DIR_PIN)); // Toggle the direction for the next movement
    // delay(1);
}

void setDirection(bool dir) {
    static bool currentDir = digitalRead(DIR_PIN); // Read the initial direction from the DIR pin

    if (currentDir != dir) {
        digitalWrite(DIR_PIN, dir);
        delayMicroseconds(50); // REQUIRED
        currentDir = dir;
    }
}
uint32_t Hz2microseconds(uint32_t frequency) {
    if (frequency == 0) {
        return 0; // Avoid division by zero
    }
    return 1000000 / frequency; // Convert frequency in Hz to period in microseconds
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

void updateStepper(int32_t &targetCount) {
    int16_t lastStepCount = 0; // Variable to store the last step count for comparison
    int16_t StepCount;
    pcnt_get_counter_value(STEP_COUNTER_PCNT_UNIT, &StepCount); // Get the current count from the step counter

    static int32_t totalStepsMoved = 0;             // Variable to keep track of the total steps moved
    totalStepsMoved += (StepCount - lastStepCount); // Update total steps moved based on the change in step count
    lastStepCount = StepCount;                      // Update the last step count for the next comparison

    static int i;
    if (i++ % 10 == 0) { // Print the step count every 10 iterations for debugging
        Serial.print("Current Step Count: ");
        Serial.println(StepCount);
    }

    if (StepCount < targetCount) {
        setDirection(HIGH);                 // Set direction to forward
        moveSteps(targetCount - StepCount); // Move the stepper motor by the required number of steps
    } else if (StepCount > targetCount) {
        setDirection(LOW);                  // Set direction to reverse
        moveSteps(StepCount - targetCount); // Move the stepper motor by the required number of steps
    }
}
