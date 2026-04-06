

#include <Arduino.h>
#include "motion_task.h"
#include "UI/ui_task.h"
#include "common/queues.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

void setup() {
    Serial.begin(500000);

    initQueues();

    startWiFiTask(1, 0);
    startMQTTTask(1, 0);

    
    xTaskCreatePinnedToCore(
        motionTask,
        "motion",
        8192,
        NULL,
        3,
        NULL,
        1); // CORE 1 (REALTIME)

    xTaskCreatePinnedToCore(
        uiTask,
        "ui",
        8192,
        NULL,
        1,
        NULL,
        0); // CORE 0 (WiFi/UI)
}

void loop() {
  vTaskDelay(portMAX_DELAY); // The main loop does nothing, all work is done in tasks
}
