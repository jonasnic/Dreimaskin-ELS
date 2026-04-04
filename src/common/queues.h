


#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct
{
    int32_t target;
    int32_t speed;
    uint8_t cmd;
} MotionCommand;

typedef enum
{
    SPEED,
    POSITION,
    DIRECTION,
} MotionDataType;

typedef union
{
    MotionDataType type;
    int32_t speed;
    int32_t position;
    uint8_t direction;
} MotionData;

extern QueueHandle_t motionQueue;

void initQueues();