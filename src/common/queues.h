


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
    uint8_t cmd;
    uint8_t mode;
} MotionCommand;



/********** For sending data from motion task to UI task **********/
typedef enum
{
    SPEED,
    POSITION,
    DIRECTION,
    DISTANCE_TO_TARGET,
    TARGET_POSITION,
} MotionDataType;
#define MOTION_DATA_TYPE_COUNT 5

typedef union
{
    int32_t speed;
    int32_t position;
    uint8_t direction;
    int32_t distance_to_target;
} MotionDataValue;

typedef struct
{
    MotionDataType type;
    MotionDataValue value;
} MotionData;




extern QueueHandle_t motionQueue;
extern QueueHandle_t UIQueue;

void initQueues();