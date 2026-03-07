#include "../camera_type.h"
#ifdef CAMERA_CONFIG_SERVER

#ifndef APRILTAG_DETECTOR_MAIN_
#define APRILTAG_DETECTOR_MAIN_

#include <WiFi.h>

#include "config_server/app_httpd.h"

#include "../constants.h"

void init_config_server();

#endif

#endif