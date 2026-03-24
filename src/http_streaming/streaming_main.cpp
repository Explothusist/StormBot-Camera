#include "../camera_type.h"
#ifdef CAMERA_STREAMING

#include "streaming_main.h"
#include "../constants.h"
#include "../camera_init.h"
#include <Arduino.h>


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
    if (!frame_buffer->buf) {
        Serial.println("Capture Failed");
    }else {
        Serial.println("Capture Successful");
    }
    return (char*)frame_buffer->buf;
};


atmt::CameraStreamingServer m_server{ WIFI_SSID, WIFI_PASSWORD, JPEG_Getter, 20, nullptr };

void init_http_streaming() {
#if DEBUG >= 1
    Serial.print("Init HTTP Streaming... ");
#endif

    m_server.init();
};

void http_streaming_loop() {
    // Serial.print("IP Address: ");
    // Serial.println(m_server.getIPAddress().c_str());
    m_server.periodic();
};


#endif