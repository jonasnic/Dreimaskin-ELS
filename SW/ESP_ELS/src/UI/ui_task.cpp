

#include "ui_task.h"
#include "mqtt_manager.h"

namespace {

bool parseModeCommand(const char* text, MotionMode* mode)
{
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

}

void uiTask(void *pv)
{
    MotionCommand cmd;

    char incomingString[100];
    byte incomingStiringIndex = 0;
    bool stringComplete = false;
    int32_t target;
    for (;;)
    {
        if (Serial.available())
        {
            char incomingChar = Serial.read();
            if (incomingChar == '\n')
            {
                stringComplete = true;
                incomingString[incomingStiringIndex] = '\0'; // Null-terminate the string
                incomingStiringIndex = 0; // Reset index for next string
            }
            else if (incomingStiringIndex < sizeof(incomingString) - 1)
            {
                incomingString[incomingStiringIndex++] = incomingChar; // Append char to string
                Serial.print(incomingChar); // Echo the character back to the serial monitor
            }
        }
        
        if (stringComplete)
        {   

            stringComplete = false;
            incomingString[sizeof(incomingString) - 1] = '\0'; // Ensure null-termination
            Serial.print("Received command: ");
            Serial.println(incomingString);


            //restart esp32 if command is "restart"
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
                target = atoi(incomingString);

                cmd = {};
                cmd.cmd = MOTION_CMD_SET_TARGET;
                cmd.target = target;
                cmd.speed = 2000;

                xQueueSend(motionQueue, &cmd, 0);
                publishTargetStatus(target);
            }
        }

        MotionData motionData;
        static int32_t lastPosition = 0;
        static int32_t lastSpeed = 0;

        if(xQueueReceive(UIQueue, &motionData, 0) == pdTRUE) {
            publishMotionData(motionData);

            if(motionData.type == POSITION && motionData.value.position != lastPosition) {

                Serial.print("Current_Position:");
                Serial.println(motionData.value.position);
                lastPosition = motionData.value.position;
            }
            else if(motionData.type == SPEED && motionData.value.speed != lastSpeed) {
                Serial.print("Current_Speed:");
                Serial.println(motionData.value.speed);
                lastSpeed = motionData.value.speed;
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