#include "mqtt_manager.h"

#include "common/queues.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <stdlib.h>

#ifndef MQTT_BROKER
#define MQTT_BROKER "192.168.1.100"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID "dreimaskin_els"
#endif

namespace {

    WiFiClient wifiClient;
    PubSubClient mqttClient(wifiClient);

    constexpr const char *kCmdTopic = "dreimaskin_els/command";
    constexpr const char *kCmdModeTopic = "dreimaskin_els/command/mode";
    constexpr const char *kStatusPosTopic = "dreimaskin_els/status/position";
    constexpr const char *kStatusSpeedTopic = "dreimaskin_els/status/speed";
    constexpr const char *kStatusTargetTopic = "dreimaskin_els/status/target";
    constexpr const char *kStatusDistanceTopic = "dreimaskin_els/status/distance_to_target";
    constexpr const char *kStatusModeTopic = "dreimaskin_els/status/mode";
    constexpr const char *kStatusAliveTopic = "dreimaskin_els/status";
    constexpr TickType_t kReconnectInterval = pdMS_TO_TICKS(1000);

    bool parseTargetCommand(const char *text, int32_t *target) {
        if (text == nullptr || target == nullptr) {
            return false;
        }

        char *endPtr = nullptr;
        long value = strtol(text, &endPtr, 10);
        if (endPtr == text || *endPtr != '\0') {
            return false;
        }

        *target = (int32_t)value;
        return true;
    }

    bool parseModeCommand(const char *text, MotionMode *mode) {
        if (text == nullptr || mode == nullptr) {
            return false;
        }

        if (strcmp(text, "position") == 0 || strcmp(text, "move") == 0) {
            *mode = MOTION_MODE_POSITION;
            return true;
        }

        if (strcmp(text, "follow") == 0) {
            *mode = MOTION_MODE_FOLLOW;
            return true;
        }

        return false;
    }

    bool queueMotionCommand(const MotionCommand &cmd) {
        return xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
    }

    void mqttCallback(char *topic, byte *payload, unsigned int length) {
        char cmdStr[32] = {0};
        if (length >= sizeof(cmdStr) - 1) {
            return;
        }

        strncpy(cmdStr, (const char *)payload, length);
        cmdStr[length] = '\0';

        if (strcmp(topic, kCmdTopic) == 0) {
            int32_t target = 0;
            if (parseTargetCommand(cmdStr, &target)) {
                MotionCommand cmd = {};
                cmd.cmd = MOTION_CMD_SET_TARGET;
                cmd.target = target;
                cmd.speed = 2000;

                if (queueMotionCommand(cmd)) {
                    publishTargetStatus(target);
                    Serial.printf("[MQTT] Command received: target %ld\n", target);
                } else {
                    Serial.println("[MQTT] Failed to queue target command");
                }
            }
        } else if (strcmp(topic, kCmdModeTopic) == 0) {
            MotionMode mode;
            if (parseModeCommand(cmdStr, &mode)) {
                MotionCommand cmd = {};
                cmd.cmd = MOTION_CMD_SET_MODE;
                cmd.mode = (uint8_t)mode;

                if (queueMotionCommand(cmd)) {
                    publishMotionMode(mode);
                    Serial.printf("[MQTT] Mode command received: %s\n", cmdStr);
                } else {
                    Serial.println("[MQTT] Failed to queue mode command");
                }
            }
        }
    }

    void connectToMQTT() {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[MQTT] WiFi not connected, skipping MQTT connection");
            return;
        }

        if (mqttClient.connected()) {
            return;
        }

        Serial.printf("[MQTT] Connecting to %s:%d\n", MQTT_BROKER, MQTT_PORT);

        mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
        mqttClient.setCallback(mqttCallback);

        // Connect with last will: publish "DEAD" if device disconnects unexpectedly
        if (mqttClient.connect(MQTT_CLIENT_ID, kStatusAliveTopic, 0, true, "DEAD")) {
            Serial.println("[MQTT] Connected to broker");

            // Publish ALIVE message to indicate successful connection
            mqttClient.publish(kStatusAliveTopic, "ALIVE", true);
            Serial.printf("[MQTT] Published ALIVE to %s\n", kStatusAliveTopic);

            mqttClient.subscribe(kCmdTopic);
            Serial.printf("[MQTT] Subscribed to %s\n", kCmdTopic);
            mqttClient.subscribe(kCmdModeTopic);
            Serial.printf("[MQTT] Subscribed to %s\n", kCmdModeTopic);
        } else {
            Serial.printf("[MQTT] Failed to connect, rc=%d\n", mqttClient.state());
        }
    }

    void mqttTask(void *pv) {
        (void)pv;

        // Initial delay to let WiFi connect first
        vTaskDelay(pdMS_TO_TICKS(2000));

        for (;;) {
            connectToMQTT();

            if (mqttClient.connected()) {
                mqttClient.loop(); // Process incoming messages and keep alive
            }

            vTaskDelay(kReconnectInterval);
        }
    }

} // namespace

void startMQTTTask(UBaseType_t priority, BaseType_t core) {
    xTaskCreatePinnedToCore(
        mqttTask,
        "mqtt",
        6144,
        nullptr,
        priority,
        nullptr,
        core);
}

bool isMQTTConnected() {
    return mqttClient.connected();
}


void publishMotionData(MotionData data) {
    if (!mqttClient.connected()) return;

    char buf[16];
    switch (data.type) {
        case POSITION:
            snprintf(buf, sizeof(buf), "%ld", data.value.position);
            mqttClient.publish(kStatusPosTopic, buf);
            break;
        case SPEED:
            snprintf(buf, sizeof(buf), "%ld", data.value.speed);
            mqttClient.publish(kStatusSpeedTopic, buf);
            break;
        case DIRECTION:
            mqttClient.publish(kStatusModeTopic, data.value.direction ? "Positive" : "Negative");
            break;
        case DISTANCE_TO_TARGET:
            snprintf(buf, sizeof(buf), "%ld", data.value.distance_to_target);
            mqttClient.publish(kStatusDistanceTopic, buf);
            break;
        case TARGET_POSITION:
            snprintf(buf, sizeof(buf), "%ld", data.value.position);
            mqttClient.publish(kStatusTargetTopic, buf);
            break;

        default:
            break;
    }
}

void publishTargetStatus(int32_t target) {
    if (!mqttClient.connected()) return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", target);
    mqttClient.publish(kStatusTargetTopic, buf, true);
}

void publishMotionMode(MotionMode mode) {
    if (!mqttClient.connected()) return;

    const char *modeText = (mode == MOTION_MODE_FOLLOW) ? "follow" : "position";
    mqttClient.publish(kStatusModeTopic, modeText, true);
}
