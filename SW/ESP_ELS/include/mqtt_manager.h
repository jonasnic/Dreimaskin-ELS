#pragma once

#include <Arduino.h>
#include "common/queues.h"

// Legacy init entry point kept for compatibility; no task is created.
void startMQTTTask(UBaseType_t priority = 1, BaseType_t core = 0);

// Non-blocking MQTT service tick; call regularly from the UI task loop.
void mqttServiceTick();

// Single-owner contract: call MQTT APIs from the same UI task that calls mqttServiceTick().

// Check if connected to broker
bool isMQTTConnected();

// Publish position and speed updates
void publishMotionStatus(int32_t position, int32_t speed);

// Publish commanded target
void publishTargetStatus(int32_t target);

// Publish active motion mode
void publishMotionMode(MotionMode mode);

void publishMotionData(MotionData data);