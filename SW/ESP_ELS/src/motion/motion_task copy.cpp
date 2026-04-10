// // motion_task.cpp

// #include "motion_task.h"
// #include "../common/queues.h"
// #include "encoder.h"
// #include "rmt_setup.h"
// #include "step_counter.h"
// #include "utils.h"

// #include <Arduino.h>
// #include <math.h>

// /*
//     ============================================================
//     GLOBAL STATE
//     ============================================================
// */

// volatile int32_t target_position  = 0;
// volatile int32_t current_position = 0;
// volatile bool    running          = false;
// volatile int32_t current_speed    = 0;   // signed steps/sec

// /*
//     ============================================================
//     CONFIG
//     ============================================================
// */

// #define BLOCK_STEPS  64
// #define HIGH_TIME_US 2

// /*
//     ============================================================
//     HELPERS
//     ============================================================
// */

// // stopping distance in steps
// static inline uint32_t steps_to_stop(int32_t speed)
// {
//     uint32_t v = abs(speed);
//     return (uint64_t)v * v / (2 * ACCELERATION);
// }

// /*
//     ------------------------------------------------------------
//     Generate one motion block
//     accel_sign:
//         +ACCELERATION  -> accelerate
//         -ACCELERATION  -> decelerate
//          0             -> cruise
//     ------------------------------------------------------------
// */
// static void generate_motion_block(uint32_t steps, int32_t accel_sign)
// {
//     if (steps == 0) return;
//     if (steps > BLOCK_STEPS) steps = BLOCK_STEPS;

//     rmt_item32_t items[BLOCK_STEPS];
//     const uint32_t min_period = (uint32_t)(1000000.0f / MAX_SPEED);

//     float speed = fabsf((float)current_speed);
//     if (speed < 1.0f) speed = 1.0f; // only for starting from 0

//     for (uint32_t i = 0; i < steps; i++)
//     {
//         uint32_t period = Hz2Us((uint32_t)speed);
//         if (period < min_period) period = min_period;

//         items[i].level0    = 1;
//         items[i].duration0 = HIGH_TIME_US;
//         items[i].level1    = 0;
//         items[i].duration1 = period - HIGH_TIME_US;

//         // update speed per step
//         if (accel_sign != 0)
//         {
//             speed += (float)accel_sign / speed;

//             if (speed < 0.0f) speed = 0.0f;        // allow full stop
//             if (speed > MAX_SPEED) speed = MAX_SPEED;
//         }
//     }

//     // restore sign
//     current_speed = (current_speed >= 0) ? (int32_t)speed : -(int32_t)speed;

//     // stop running if speed reached 0
//     if (current_speed == 0)
//         running = false;
//     else
//         running = true;

//     rmt_write_items(RMT_CH, items, steps, false);
// }

// /*
//     ============================================================
//     STREAMING MOTION PLANNER
//     ============================================================
// */


// static void move_toward_target()
// {
//     int32_t dist = target_position - current_position;
//     bool dir = (dist > 0);

//     // prevent instant reversal at speed
//     if ((current_speed > 0 && !dir) || (current_speed < 0 && dir))
//     {
//         generate_motion_block(BLOCK_STEPS, -ACCELERATION);
//         return;
//     }

//     if (dist != 0)
//     {
//         set_direction(dir);

//         uint32_t remaining  = abs(dist);
//         uint32_t stop_steps = steps_to_stop(current_speed);

//         uint32_t block = remaining;
//         if (block > BLOCK_STEPS)
//             block = BLOCK_STEPS;

//         if (stop_steps >= remaining)
//             generate_motion_block(block, -ACCELERATION); // decel
//         else if (abs(current_speed) < MAX_SPEED)
//             generate_motion_block(block, ACCELERATION);  // accel
//         else
//             generate_motion_block(block, 0);             // cruise

//         if (dir)
//             current_position += block;
//         else
//             current_position -= block;

//         running = true;
//     }
//     else
//     {
//         // target reached → brake if still moving
//         if (current_speed != 0)
//         {
//             uint32_t stop_steps = steps_to_stop(current_speed);
//             generate_motion_block(stop_steps, -ACCELERATION);
//             running = true;
//         }
//         else
//         {
//             running = false;      // actually stopped
//             current_speed = 0;
//         }
//     }
// }

// /*
//     ============================================================
//     RMT CALLBACK
//     ============================================================
// */

// void onRMTTransmissionComplete(rmt_channel_t channel, void *arg)
// {
//     move_toward_target();
// }

// /*
//     ============================================================
//     MOTION TASK
//     ============================================================
// */

// void motionTask(void *pv)
// {
//     MotionCommand cmd;

//     MotionData motionData;
//     motionData.type = POSITION;

//     MotionData speedData;
//     speedData.type = SPEED;

//     /*
//         ---- IO SETUP ----
//     */

//     pinMode(DIR_PIN, OUTPUT);
//     pinMode(ENABLE_PIN, OUTPUT);

//     digitalWrite(ENABLE_PIN, LOW);
//     digitalWrite(DIR_PIN, LOW);

//     setupEncoder();

//     setupRMT(
//         (gpio_num_t)PULSE_PIN,
//         RMT_CH,
//         onRMTTransmissionComplete);

//     setupStepCounter(
//         (gpio_num_t)PULSE_WATCH_PIN,
//         (gpio_num_t)DIR_WATCH_PIN);

//     /*
//         ========================================================
//         MAIN LOOP
//         ========================================================
//     */

//     for (;;)
//     {
//         // receive commands
//         if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE)
//         {
//             target_position = cmd.target;
//         }

//         // kick planner if idle
//         if (!running)
//             move_toward_target();

//         /*
//             periodic UI update
//         */
//         static int i = 0;
//         if (i++ % 100 == 0)
//         {
//             if (UIQueue) // prevent NULL queue crash
//             {
//                 motionData.value.position = current_position;
//                 xQueueSend(UIQueue, &motionData, 0);

//                 speedData.value.speed = current_speed;
//                 xQueueSend(UIQueue, &speedData, 0);
//             }
//         }

//         vTaskDelay(1);
//     }
// }