
#include "camera_type.h"
#include "constants.h"
#include <Arduino.h>

#include "camera_init.h"

#ifdef CAMERA_CONFIG_SERVER
#include "config_server/config_server_main.h"
#endif

#ifdef CAMERA_APRILTAG_DETECTOR
#include "apriltag_detector/detector_main.h"
#endif

#ifdef CAMERA_STREAMING
#include "http_streaming/streaming_main.h"
#endif

void setup() {

    init_camera();

#ifdef CAMERA_CONFIG_SERVER
    init_config_server();
#endif

#ifdef CAMERA_APRILTAG_DETECTOR
    init_apriltag_detector();
    start_apriltag_detector_loop();
#endif

#ifdef CAMERA_STREAMING
    init_http_streaming();
#endif

}

void loop() {
    // Nothing here, the real loop is already at the end of setup()
    
#ifdef CAMERA_STREAMING
    http_streaming_loop();
#endif
}