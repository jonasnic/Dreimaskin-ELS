#include "wifi_manager.h"

#include <WiFi.h>
#include <cstring>

namespace {
const char *wifiStatusToText(wl_status_t status) {
    switch (status) {
    case WL_NO_SHIELD:
        return "WL_NO_SHIELD";
    case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:
        return "WL_CONNECTED";
    case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    default:
        return "WL_UNKNOWN";
    }
}
} // namespace

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

namespace {
constexpr TickType_t kReconnectInterval = pdMS_TO_TICKS(1000);
constexpr uint32_t kConnectTimeoutMs = 8000;
bool connectAttemptActive = false;
uint32_t connectAttemptStartMs = 0;

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("[WiFi] STA started");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.println("[WiFi] Connected to AP");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.printf("[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.printf("[WiFi] Disconnected, reason=%u\n", (unsigned)info.wifi_sta_disconnected.reason);
        break;
    default:
        break;
    }
}

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
        connectAttemptActive = false;
        return;
    }

    if (!connectAttemptActive) {
        Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        connectAttemptActive = true;
        connectAttemptStartMs = millis();
        return;
    }

    if ((millis() - connectAttemptStartMs) >= kConnectTimeoutMs) {
        Serial.printf("[WiFi] Connect timeout. Status=%s. Retrying.\n", wifiStatusToText(WiFi.status()));
        connectAttemptActive = false;
        WiFi.reconnect();
    }
}

void wifiTask(void *pv) {
    (void)pv;

    Serial.println("[WiFi] Task started");
    Serial.printf("[WiFi] Config SSID length: %u\n", (unsigned)std::strlen(WIFI_SSID));

    Serial.println("[WiFi] Registering event handler");
    WiFi.onEvent(onWiFiEvent);
    // Avoid WiFi.mode() here: it can block under load on some builds.
    Serial.println("[WiFi] Disabling sleep");
    WiFi.setSleep(false);
    Serial.println("[WiFi] Setting non-persistent config");
    WiFi.persistent(false);
    Serial.println("[WiFi] Enabling auto-reconnect");
    WiFi.setAutoReconnect(true);
    Serial.println("[WiFi] Init complete");

    for (;;) {
        connectToWiFi();
        vTaskDelay(kReconnectInterval);
    }
}
} // namespace

void startWiFiTask(UBaseType_t priority, BaseType_t core) {
    BaseType_t created = xTaskCreatePinnedToCore(
        wifiTask,
        "wifi",
        4096,
        nullptr,
        priority,
        nullptr,
        core);

    if (created != pdPASS) {
        Serial.println("[WiFi] Failed to create WiFi task");
    } else {
        Serial.println("[WiFi] WiFi task created");
    }
}
