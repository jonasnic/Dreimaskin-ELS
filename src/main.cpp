

#include <Arduino.h>
#include "motion/motion_task.h"
#include "UI/ui_task.h"
#include "common/queues.h"

void setup() {
    Serial.begin(115200);

    initQueues();

    
    xTaskCreatePinnedToCore(
        motionTask,
        "motion",
        4096,
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
