// #include "motion_task.h"
// #include "../common/queues.h"
// #include "encoder.h"
// #include "rmt_setup.h"
// #include "step_counter.h"
// #include "utils.h"

// volatile int32_t target_position = 0;
// volatile int32_t current_position = 0;
// volatile bool running = false;      // Flag to indicate if the motion task is currently executing a move. will use to know if ISR are in effect or not
// volatile int32_t current_speed = 0; // signed steps/sec

// static inline uint32_t stepps_needed_to_stop(int32_t speed = current_speed) {
//     int32_t v = abs(speed);
//     int32_t steps_to_stop = v / ACCELERATION; // Using the formula v^2 = 2*a*s, we can derive s = v^2 / (2*a). However, since we're working with discrete steps and a constant acceleration, we can simplify this to s = v / a for a linear deceleration profile.
//     return steps_to_stop;
// }

// int32_t speed_and_acceleration_calculation() {
//     int32_t stepsToMove = target_position - current_position;
//     int32_t dir = (stepsToMove > 0) ? 1 : -1;
//     // should increase speed to max speed until we reach the point where we need to start decelerating to stop at the target position. The distance required to decelerate from a given speed can be calculated using the formula: distance = (speed^2) / (2 * acceleration). So, we can calculate the distance needed to stop from the current speed and compare it to the remaining distance to the target. If the remaining distance is less than or equal to the stopping distance, we should start decelerating.
//     int32_t stopping_distance = stepps_needed_to_stop(current_speed);

//     int32_t target_speed;
//     if (abs(stepsToMove) <= stopping_distance) {
//         // We are within the stopping distance, so we should decelerate
//         target_speed = current_speed - (ACCELERATION * dir); // Decrease speed by the acceleration value in the direction of movement
//     } else {
//         // We are outside the stopping distance, so we can accelerate to max speed
//         target_speed = current_speed + (ACCELERATION * dir); // Increase speed by the acceleration value in the direction of movement
//     }
//     target_speed = constrain(target_speed, -MAX_SPEED, MAX_SPEED); // Ensure the target speed does not exceed the maximum speed limits
    
//     return target_speed;
// }

// void moveSteps(uint32_t steps) {

//     // uint32_t speed = Update_Speed(steps);
//     // uint32_t sleep_time = Hz2Us(speed);             // Calculate the sleep time based on the desired speed
//     int32_t local_current_speed = current_speed;
//     uint32_t start_sleep_time = Hz2Us(abs(local_current_speed)); // Calculate the sleep
//     int32_t target_speed = speed_and_acceleration_calculation();
//     uint32_t target_sleep_time = Hz2Us(abs(target_speed)); // Calculate the sleep time for the target speed

//     uint32_t step_difference = target_sleep_time - start_sleep_time;
    
    
//     const uint32_t high_time = 2;               // Fixed HIGH time for the pulse
    
//     if (steps > MAX_RMT_STEPS)
//     steps = MAX_RMT_STEPS;
//     rmt_item32_t items[MAX_RMT_STEPS];
    
//     uint32_t acceleration_in_us = step_difference / steps; // Calculate how much to change
//     uint32_t low_time = start_sleep_time - high_time; // Initial LOW time for the pulse, ensuring the total period starts at the current speed's timing

//     for (uint32_t i = 0; i < steps; i++) {
//         items[i].level0 = 1;
//         items[i].duration0 = high_time; // HIGH pulse duration
//         items[i].level1 = 0;
//         items[i].duration1 = low_time;                                                // LOW pulse duration
//         low_time = low_time + acceleration_in_us; // Incrementally adjust the LOW time for the next pulse to create an acceleration or deceleration effect. This will linearly change the pulse period from the start sleep time to the target sleep time over the number of steps being moved.
//     }
//     current_speed = (1e6 / (high_time + low_time)) * ((local_current_speed > 0) ? 1 : -1); // Update the current speed based on the actual pulse timing being sent to the motor. This is important for accurate speed tracking, especially when acceleration is involved.
//     rmt_write_items(RMT_CH, items, steps, false);                                          // Write the pulse sequence to the RMT channel
//     bool direction = (target_position - current_position) > 0;
//     current_position += direction ? steps : -steps; // Update the current position based on the direction of movement
// }

// uint32_t Update_Speed(int32_t remainingSteps) {
// }

// // uint32_t Update_Speed(int32_t remainingSteps) {
// //     int32_t dir = (remainingSteps > 0) ? 1 : -1;
// //     int32_t target_speed = ((int64_t)abs(remainingSteps) * (MAX_SPEED - MIN_SPEED)) / MAX_SPEED + MIN_SPEED;

// //     // Serial.printf("Remaining steps: %d | Target speed: %d steps/s\n", remainingSteps, target_speed);

// //     target_speed = constrain(target_speed, MIN_SPEED, MAX_SPEED);

// //     // signed target velocity
// //     int32_t targetVelocity = target_speed * dir;

// //     // low-pass velocity filter (motion smoothing)
// //     int32_t filter = 50;
// //     current_speed =
// //         (current_speed * (filter - 1) + targetVelocity) / filter;

// //     // deadband near stop
// //     if (abs(remainingSteps) < 2)
// //         current_speed = 0;

// //     return abs(current_speed);
// // }

// void update_current_position(int32_t *current_position) {
//     int16_t StepCount;
//     int16_t stepDelta;
//     static int16_t lastStepCount = 0; // persist between calls

//     pcnt_get_counter_value(STEP_COUNTER_PCNT_UNIT, &StepCount);

//     // handel the overflow and underflow of the 16-bit step count from the encoder
//     // PCNT overflows and underflows to 0 not INT16_MAX or INT16_MIN, so we need to detect when the count wraps around and adjust the step delta accordingly to simulate real overflow and underflow behavior of a signed 16-bit integer.
//     stepDelta = StepCount - lastStepCount;
//     if (stepDelta > 30000) {
//         stepDelta -= INT16_MIN;
//     }

//     if (stepDelta < -30000) {
//         stepDelta += INT16_MAX;
//     }
//     *current_position += stepDelta;
//     lastStepCount = StepCount;
// }

// void move_toward_target() {
//     int32_t stepsToMove = target_position - current_position;
//     if (stepsToMove != 0) {
// //        setDirection(stepsToMove > 0); // Set direction based on whether we need to move forward or backward
//         running = true;                // Set the running flag to indicate we're executing a move
//         moveSteps(abs(stepsToMove));   // Move the required number of steps
//     }
// }

// /*
// This callback is called by the RMT driver when it finishes transmitting the pulse sequence.
// We use it to check if we've reached the target position and, if not, to continue moving towards the target.
// It is also called in between moves, like to initiate the first move towards the target when we receive a new command in the motion task loop.
// */
// void onRMTTransmissionComplete(rmt_channel_t channel, void *arg) {
//     if (current_position == target_position) {
//         running = false;   // Clear the running flag when we reach the target
//         current_speed = 0; // Ensure speed is set to 0 when we reach the target
//     } else {
//         move_toward_target(); // Continue moving towards the target if we haven't reached it yet
//     }
// }
// void motionTask(void *pv) {
//     int32_t actual_current_position = 0;
//     MotionCommand cmd;

//     //-------- For sending data to the UI task --------
//     MotionData motionData;
//     motionData.type = POSITION;

//     MotionData speedData;
//     speedData.type = SPEED;
//     //-----------------------------------------------

//     //---------Seting the pins and peripherals---------
//     digitalWrite(ENABLE_PIN, LOW); // Enable the stepper motor driver
//     digitalWrite(DIR_PIN, LOW);    // Set initial direction (e.g., LOW for forward)
//     pinMode(DIR_PIN, OUTPUT);
//     pinMode(ENABLE_PIN, OUTPUT);

//     setupEncoder();                                                     // Initialize the encoder interface
//     setupRMT((gpio_num_t)PULSE_PIN, RMT_CH, onRMTTransmissionComplete); // Initialize the RMT peripheral for generating step pulses
//     setupStepCounter((gpio_num_t)PULSE_WATCH_PIN, (gpio_num_t)DIR_WATCH_PIN);
//     //-----------------------------------------------

//     for (;;) {
//         update_current_position(&actual_current_position);

//         if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
//             target_position = cmd.target;
//         }
//         // move_toward_target();
//         if (!running) {
//             onRMTTransmissionComplete(RMT_CH, NULL); // Manually call the RMT transmission complete callback to start the motion towards the target. In a real implementation, this would be triggered by the RMT hardware when it finishes sending pulses.
//         }
//         // runMotionPlanner(target);

//         static int i = 0;
//         if (i++ % 100 == 0) {

//             motionData.value.position = current_position;
//             xQueueSend(UIQueue, &motionData, 0);

//             ;
//             speedData.value.speed = current_speed;
//             xQueueSend(UIQueue, &speedData, 0);
//         }

//         vTaskDelay(1); // yields CPU but keeps timing stable
//     }
// }