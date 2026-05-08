

#include "ui_task.h"
#include "mqtt_manager.h"

constexpr auto btnUpPin = 23;
constexpr auto btnRightPin = 22;
constexpr auto btnDownPin = 19;
constexpr auto btnLeftPin = 21;
void setupBtns();

namespace {

    bool parseModeCommand(const char *text, MotionMode *mode) {
        if (text == nullptr || mode == nullptr) {
            return false;
        }

        if (strcmp(text, "mode position") == 0 || strcmp(text, "mode move") == 0) {
            *mode = MOTION_MODE_POSITION;
            return true;
        }

        if (strcmp(text, "mode follow") == 0) {
            *mode = MOTION_MODE_FOLLOW;
            return true;
        }

        return false;
    }

    bool parseStepperEnableCommand(const char *text, bool *enable) {
        if (text == nullptr || enable == nullptr) {
            return false;
        }

        if (strcmp(text, "stepper enable") == 0 || strcmp(text, "enable stepper") == 0) {
            *enable = true;
            return true;
        }

        if (strcmp(text, "stepper disable") == 0 || strcmp(text, "disable stepper") == 0) {
            *enable = false;
            return true;
        }

        return false;
    }

}

enum buttonId_t {
    BTN_UP,
    BTN_RIGHT,
    BTN_DOWN,
    BTN_LEFT
};
struct btnEvent_t {
    buttonId_t button;
    uint32_t timestamp;
};
QueueHandle_t btnQueue;

void uiTask(void *pv) {
    btnQueue = xQueueCreate(3, sizeof(btnEvent_t));
    setupBtns();

    MotionCommand cmd;

    char incomingString[100];
    byte incomingStiringIndex = 0;
    bool stringComplete = false;
    int32_t target;
    for (;;) {

        btnEvent_t btnEvent;
        if (xQueueReceive(btnQueue, &btnEvent, 0) == pdTRUE) {
            switch (btnEvent.button) {
            case BTN_UP:
                Serial.println("Up button pressed!");
                break;
            case BTN_RIGHT:
                Serial.println("Right button pressed!");
                break;
            case BTN_DOWN:
                Serial.println("Down button pressed!");
                break;
            case BTN_LEFT:
                Serial.println("Left button pressed!");
                break;
            }
        }
        mqttServiceTick();

        if (Serial.available()) {
            char incomingChar = Serial.read();
            if (incomingChar == '\n') {
                stringComplete = true;
                incomingString[incomingStiringIndex] = '\0'; // Null-terminate the string
                incomingStiringIndex = 0;                    // Reset index for next string
            } else if (incomingStiringIndex < sizeof(incomingString) - 1) {
                incomingString[incomingStiringIndex++] = incomingChar; // Append char to string
                Serial.print(incomingChar);                            // Echo the character back to the serial monitor
            }
        }

        if (stringComplete) {

            stringComplete = false;
            incomingString[sizeof(incomingString) - 1] = '\0'; // Ensure null-termination
            Serial.print("Received command: ");
            Serial.println(incomingString);

            // restart esp32 if command is "restart"
            if (strcmp(incomingString, "restart") == 0) {
                Serial.println("Restarting ESP32...");
                esp_restart();
            }
            MotionMode mode;
            if (parseModeCommand(incomingString, &mode)) {
                cmd = {};
                cmd.cmd = MOTION_CMD_SET_MODE;
                cmd.mode = (uint8_t)mode;
                xQueueSend(motionQueue, &cmd, 0);
                publishMotionMode(mode);
            } else {
                bool enableStepper = false;
                if (parseStepperEnableCommand(incomingString, &enableStepper)) {
                    cmd = {};
                    cmd.cmd = MOTION_CMD_SET_STEPPER_ENABLE;
                    cmd.stepper_enabled = enableStepper ? 1 : 0;
                    xQueueSend(motionQueue, &cmd, 0);
                } else {
                    target = atoi(incomingString);

                    cmd = {};
                    cmd.cmd = MOTION_CMD_SET_TARGET;
                    cmd.target = target;
                    cmd.speed = 2000;

                    xQueueSend(motionQueue, &cmd, 0);
                    publishTargetStatus(target);
                }
            }
        }

        MotionData motionData;
        static int32_t lastPosition = 0;
        static int32_t lastSpeed = 0;
        static uint32_t lastLoopTimeUs = 0;
        static uint32_t lastBatchTimeUs = 0;
        static uint8_t lastAlarm = 0xFF; // 0xFF forces first-ever print

        if (xQueueReceive(UIQueue, &motionData, 0) == pdTRUE) {
            publishMotionData(motionData);

            if (motionData.type == POSITION && motionData.value.position != lastPosition) {

                // Serial.print("Current_Position:");
                // Serial.println(motionData.value.position);
                lastPosition = motionData.value.position;
            } else if (motionData.type == SPEED && motionData.value.speed != lastSpeed) {
                // Serial.print("Current_Speed:");
                // Serial.println(motionData.value.speed);
                lastSpeed = motionData.value.speed;
            } else if (motionData.type == LOOP_TIME_US && motionData.value.loop_time_us != lastLoopTimeUs) {
                // Serial.print("Motion_Block_LoopTime_us:");
                // Serial.println(motionData.value.loop_time_us);
                lastLoopTimeUs = motionData.value.loop_time_us;
            } else if (motionData.type == BATCH_TIME_US && motionData.value.batch_time_us != lastBatchTimeUs) {
                // Serial.print("Motion_BatchTime_us:");
                // Serial.println(motionData.value.batch_time_us);
                lastBatchTimeUs = motionData.value.batch_time_us;
            } else if (motionData.type == ALARM && motionData.value.alarm != lastAlarm) {
                // Serial.print("Stepper_Alarm:");
                // Serial.println(motionData.value.alarm ? "ALARM" : "OK");
                lastAlarm = motionData.value.alarm;
            }
            // // else if(motionData.type == DIRECTION) {
            // //     Serial.print("Current_Direction:");
            // //     Serial.println(motionData.value.direction ? "Positive" : "Negative");
            // // }
            // else if(motionData.type == DISTANCE_TO_TARGET) {
            //     Serial.print("Distance_to_Target:");
            //     Serial.println(motionData.value.distance_to_target);
            // }

            // Publish to MQTT if connected (data cycles through position and speed)
        }
        vTaskDelay(5);
    }
}


bool debouceCheck(uint32_t *lastInterruptTime) {
    uint32_t interruptTime = millis();
    if (interruptTime - *lastInterruptTime < 200) {
        return false;
    }
    *lastInterruptTime = interruptTime;
    return true;
}
void IRAM_ATTR onBtnUpPress() {
    static uint32_t lastInterruptTime = 0;
    if (!debouceCheck(&lastInterruptTime)) {
        return;
    }
    btnEvent_t event = {.button = BTN_UP, .timestamp = millis()};
    lastInterruptTime = event.timestamp;
    xQueueSendFromISR(btnQueue, &event, NULL);
}
void IRAM_ATTR onBtnRightPress() {
    static uint32_t lastInterruptTime = 0;
    if (!debouceCheck(&lastInterruptTime)) {
        return;
    }
    btnEvent_t event = {.button = BTN_RIGHT, .timestamp = millis()};
    lastInterruptTime = event.timestamp;
    xQueueSendFromISR(btnQueue, &event, NULL);
}
void IRAM_ATTR onBtnDownPress() {
    static uint32_t lastInterruptTime = 0;
    if (!debouceCheck(&lastInterruptTime)) {
        return;
    }
    btnEvent_t event = {.button = BTN_DOWN, .timestamp = millis()};
    lastInterruptTime = event.timestamp;
    xQueueSendFromISR(btnQueue, &event, NULL);
}
void IRAM_ATTR onBtnLeftPress() {
    static uint32_t lastInterruptTime = 0;
    if (!debouceCheck(&lastInterruptTime)) {
        return;
    }
    btnEvent_t event = {.button = BTN_LEFT, .timestamp = millis()};
    lastInterruptTime = event.timestamp;
    xQueueSendFromISR(btnQueue, &event, NULL);
}
void setupBtns() {
    pinMode(btnUpPin, INPUT_PULLUP);
    pinMode(btnRightPin, INPUT_PULLUP);
    pinMode(btnDownPin, INPUT_PULLUP);
    pinMode(btnLeftPin, INPUT_PULLUP);
    attachInterrupt(btnUpPin, onBtnUpPress, FALLING);
    attachInterrupt(btnRightPin, onBtnRightPress, FALLING);
    attachInterrupt(btnDownPin, onBtnDownPress, FALLING);
    attachInterrupt(btnLeftPin, onBtnLeftPress, FALLING);
}