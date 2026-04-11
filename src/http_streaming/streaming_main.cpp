#include "../camera_type.h"
#ifdef CAMERA_STREAMING

#include "streaming_main.h"
#include "streaming_constants.h"
#include "../constants.h"
#include "../camera_init.h"
#include <Arduino.h>
#include "../apriltag_detector/dectector_utils.h"


camera_fb_t* frame_buffer;

char* JPEG_Getter(size_t& length, void* arg) {
    // Get a frame from camera
    if (frame_buffer) {
        esp_camera_fb_return(frame_buffer);
    }
    frame_buffer = esp_camera_fb_get();
    if (!frame_buffer) {
#if DEBUG >= 3
        Serial.println("Failed to get frame! Retrying...");
#endif
        return nullptr;
    }
    length = frame_buffer->len;
    // if (!frame_buffer->buf) {
    //     Serial.println("Capture Failed");
    // }else {
    //     Serial.println("Capture Successful");
    // }
    return (char*)frame_buffer->buf;
};


// atmt::CameraStreamingServer m_server{ (std::string("STORM_Streaming_Camera-")+std::string(CAMERA_ID)), WIFI_SSID, WIFI_PASSWORD, JPEG_Getter, 20, nullptr };
atmt::CameraStreamingServer* m_server{ nullptr };
// atmt::RobotDashboardServer m_server{ "STORM_Esp32_MainBot", WIFI_SSID, WIFI_PASSWORD };
// atmt::CameraStreamingServer m_server{ WIFI_SSID, WIFI_PASSWORD, JPEG_Getter, 1, nullptr };

void init_http_streaming() {
#if DEBUG >= 1
    Serial.println("Init HTTP Streaming... ");
#endif

    
    atmt::SerialReader serial_reader = atmt::SerialReader(atmt::Interface_Serial1, Address_StreamingCameras, RX_PIN, TX_PIN);

    bool network_received = false;
    while (!network_received) {
        Serial.println("Waiting for Network Message...");
        while (serial_reader.availableMessages()) {
            Serial.println("Reading Serial Message");
            uint8_t prefix = 0;
            uint8_t message[atmt::kMaxPacketSize];
            uint8_t length = 0;
            serial_reader.popNextMessagePrefixed(prefix, message, length);

            if (prefix == Serial_ConnectToNetwork) {
                m_server = new atmt::CameraStreamingServer((std::string("STORM_Streaming_Camera-")+std::string(CAMERA_ID)), getNetworkSSID(static_cast<NetworkIndicator>(message[0])), getNetworkPassword(static_cast<NetworkIndicator>(message[0])), JPEG_Getter, 20, nullptr);
                network_received = true;
                break;
            }
        }
        delay(250);
    }

    Serial.println("Network Message Received");

    m_server->init();
};

void http_streaming_loop() {
    // Serial.print("IP Address: ");
    // Serial.println(m_server.getIPAddress().c_str());
    m_server->periodic();
};


#endif