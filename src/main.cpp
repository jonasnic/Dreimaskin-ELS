

#include <Arduino.h>
#include "motion_task.h"
#include "UI/ui_task.h"
#include "common/queues.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

namespace {
constexpr bool kWiFiOnlyDiagnosticMode = false;
}

#ifndef ENABLE_DEBUG_WIFI
#define ENABLE_DEBUG_WIFI 1
#endif

void setup() {
    Serial.begin(500000);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("[BOOT] setup start");
    Serial.printf("[BOOT] Free heap at boot: %u\n", (unsigned)ESP.getFreeHeap());

    initQueues();
    Serial.println("[BOOT] queues initialized");
    Serial.printf("[BOOT] Free heap after queues: %u\n", (unsigned)ESP.getFreeHeap());

#if ENABLE_DEBUG_WIFI
    startWiFiTask(1, 0);
    Serial.printf("[BOOT] Free heap after WiFi task: %u\n", (unsigned)ESP.getFreeHeap());

    if (kWiFiOnlyDiagnosticMode) {
        Serial.println("[BOOT] WiFi-only diagnostic mode enabled");
        return;
    }

    startMQTTTask(1, 0);
    Serial.printf("[BOOT] Free heap after MQTT task: %u\n", (unsigned)ESP.getFreeHeap());
#else
    Serial.println("[BOOT] Debug WiFi/MQTT disabled (ENABLE_DEBUG_WIFI=0)");
#endif

    BaseType_t motionCreated = xTaskCreatePinnedToCore(
        motionTask,
        "motion",
        8192,
        NULL,
        3,
        NULL,
        1); // CORE 1 (REALTIME)
    if (motionCreated != pdPASS) {
        Serial.println("[BOOT] Failed to create motion task");
    } else {
        Serial.println("[BOOT] Motion task created");
    }
    Serial.printf("[BOOT] Free heap after motion task: %u\n", (unsigned)ESP.getFreeHeap());

    BaseType_t uiCreated = xTaskCreatePinnedToCore(
        uiTask,
        "ui",
        8192,
        NULL,
        1,
        NULL,
        0); // CORE 0 (WiFi/UI)
    if (uiCreated != pdPASS) {
        Serial.println("[BOOT] Failed to create UI task");
    } else {
        Serial.println("[BOOT] UI task created");
    }
    Serial.printf("[BOOT] Free heap after UI task: %u\n", (unsigned)ESP.getFreeHeap());

}

void loop() {
  vTaskDelay(portMAX_DELAY); // The main loop does nothing, all work is done in tasks
}
