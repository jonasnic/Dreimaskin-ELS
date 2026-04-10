#include "queues.h"

QueueHandle_t motionQueue;
QueueHandle_t UIQueue;

void initQueues()
{
    motionQueue = xQueueCreate(10, sizeof(MotionCommand));
    UIQueue = xQueueCreate(10, sizeof(MotionData));
    
}