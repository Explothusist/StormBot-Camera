#include "../camera_type.h"
#ifdef CAMERA_STREAMING

#ifndef HTTP_STREAMING_MAIN_
#define HTTP_STREAMING_MAIN_

#include "../Automat/automat.h" // Currently just serial/http server
#include "esp_camera.h"

#include "../constants.h"

// IP Address range 10.10.92.0-24

void init_http_streaming();

void http_streaming_loop();


#endif

#endif