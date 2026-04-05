#include "wifi_manager.h"

#include <WiFi.h>
#include <cstring>

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

namespace {
constexpr TickType_t kReconnectInterval = pdMS_TO_TICKS(10000);
constexpr uint32_t kConnectTimeoutMs = 8000;

bool isWiFiConfigured() {
    return std::strlen(WIFI_SSID) > 0 && std::strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0;
}

void connectToWiFi() {
    if (!isWiFiConfigured()) {
        static bool warningPrinted = false;
        if (!warningPrinted) {
            Serial.println("[WiFi] No credentials set. Update WIFI_SSID/WIFI_PASSWORD in platformio.ini.");
            warningPrinted = true;
        }
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < kConnectTimeoutMs) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WiFi] Connected, IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("[WiFi] Connection timeout. Will retry.");
    }
}

void wifiTask(void *pv) {
    (void)pv;

    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);

    for (;;) {
        connectToWiFi();
        vTaskDelay(kReconnectInterval);
    }
}
} // namespace

void startWiFiTask(UBaseType_t priority, BaseType_t core) {
    xTaskCreatePinnedToCore(
        wifiTask,
        "wifi",
        4096,
        nullptr,
        priority,
        nullptr,
        core);
}
