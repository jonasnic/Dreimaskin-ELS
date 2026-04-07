#pragma once

#include <Arduino.h>
#include "common/queues.h"

// Start MQTT management task
void startMQTTTask(UBaseType_t priority = 1, BaseType_t core = 0);

// Check if connected to broker
bool isMQTTConnected();

// Publish position and speed updates
void publishMotionStatus(int32_t position, int32_t speed);

// Publish commanded target
void publishTargetStatus(int32_t target);

// Publish active motion mode
void publishMotionMode(MotionMode mode);

void publishMotionData(MotionData data);