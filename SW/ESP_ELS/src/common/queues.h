


#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/*
// ------------FROM UI TO MOTION TASK---------
*/
typedef enum
{
    MOTION_CMD_SET_TARGET = 1,
    MOTION_CMD_SET_MODE = 2,
    MOTION_CMD_SET_STEPPER_ENABLE = 3,
    MOTION_CMD_SET_PITCH = 4,
} MotionCommandType;

typedef enum
{
    MOTION_MODE_POSITION = 0,
    MOTION_MODE_FOLLOW = 1,
} MotionMode;

typedef struct
{
    int32_t target;
    int32_t speed;
    int32_t pitch_1e5_mm_per_rev;
    uint8_t cmd;
    uint8_t mode;
    uint8_t stepper_enabled;
} MotionCommand;



/********** For sending data from motion task to UI task **********/
typedef enum
{
    SPEED,
    POSITION,
    DIRECTION,
    DISTANCE_TO_TARGET,
    TARGET_POSITION,
    LOOP_TIME_US,
    BATCH_TIME_US,
    ALARM,
} MotionDataType;
#define MOTION_DATA_TYPE_COUNT 8

typedef union
{
    int32_t speed;
    int32_t position;
    uint8_t direction;
    int32_t distance_to_target;
    uint32_t loop_time_us;
    uint32_t batch_time_us;
    uint8_t alarm;
} MotionDataValue;

typedef struct
{
    MotionDataType type;
    MotionDataValue value;
} MotionData;




extern QueueHandle_t motionQueue;
extern QueueHandle_t UIQueue;

void initQueues();