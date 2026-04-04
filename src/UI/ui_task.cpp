

#include "ui_task.h"

void uiTask(void *pv)
{
    MotionCommand cmd;

    char incomingString[100];
    byte incomingStiringIndex = 0;
    bool stringComplete = false;
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
            int steps = atoi(incomingString);

            cmd.cmd = 1;
            cmd.target = steps;
            cmd.speed = 2000;

            xQueueSend(motionQueue, &cmd, 0);
        }

        MotionData data;

        if(xQueueReceive(UIQueue, &data, 0) == pdTRUE) {
            if(data.type == POSITION) {
                Serial.print("Current Position: ");
                Serial.println(data.value.position);
            }
        }
        vTaskDelay(5);
    }
}