#include "../camera_type.h"
#ifdef CAMERA_CONFIG_SERVER

#include "config_server_main.h"
#include "../constants.h"

void init_config_server() {

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

#if DEBUG >= 1
    Serial.print("Connecting to WiFi");
#endif

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
#if DEBUG >= 1
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status());
#endif
    }

#if DEBUG >= 1
    Serial.println("");
    Serial.println("WiFi connected");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
#endif

    startCameraServer();
};

#endif