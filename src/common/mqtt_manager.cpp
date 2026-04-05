#include "mqtt_manager.h"

#include <PubSubClient.h>
#include <WiFi.h>
#include "common/queues.h"

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

constexpr const char* kCmdTopic = "dreimaskin_els/command";
constexpr const char* kStatusPosTopic = "dreimaskin_els/status/position";
constexpr const char* kStatusSpeedTopic = "dreimaskin_els/status/speed";
constexpr TickType_t kReconnectInterval = pdMS_TO_TICKS(5000);

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Parse command from MQTT topic
    if (strcmp(topic, kCmdTopic) == 0) {
        // Convert payload to string
        char cmdStr[32] = {0};
        if (length < sizeof(cmdStr) - 1) {
            strncpy(cmdStr, (const char*)payload, length);
            cmdStr[length] = '\0';
            
            int steps = atoi(cmdStr);
            if (steps != 0) {
                MotionCommand cmd;
                cmd.cmd = 1;
                cmd.target = steps;
                cmd.speed = 2000;
                
                if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("[MQTT] Command received: move %d steps\n", steps);
                } else {
                    Serial.println("[MQTT] Failed to queue command");
                }
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

    if (mqttClient.connect(MQTT_CLIENT_ID)) {
        Serial.println("[MQTT] Connected to broker");
        mqttClient.subscribe(kCmdTopic);
        Serial.printf("[MQTT] Subscribed to %s\n", kCmdTopic);
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
            mqttClient.loop();  // Process incoming messages and keep alive
        }

        vTaskDelay(kReconnectInterval);
    }
}

}  // namespace

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

void publishMotionStatus(int32_t position, int32_t speed) {
    if (!mqttClient.connected()) {
        return;
    }

    char buf[16];
    
    // Publish position
    snprintf(buf, sizeof(buf), "%ld", position);
    mqttClient.publish(kStatusPosTopic, buf);
    
    // Publish speed
    snprintf(buf, sizeof(buf), "%ld", speed);
    mqttClient.publish(kStatusSpeedTopic, buf);
}
