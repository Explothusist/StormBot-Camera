#ifndef STORMBOT_CAMERA_CONSTANTS_
#define STORMBOT_CAMERA_CONSTANTS_

#include "camera_type.h"

/*
 * Define this macro to enable debug mode
 * Level 0: Absolutely no debug at all. Suitable for
 *    production use.
 * Level 1: Simple debug messages , like simple events
 *    notifications.
 * Level 2: Low level debug messages
 * Level 3: Debug messages that in a loop
 */
#define DEBUG 1

// Config for pose estimation
// Tag size (in meter). See original AprilTag readme for how to measure
#define TAG_SIZE 0.05 // 2 in (~0.05 m)

#if defined(CAMERA_CONFIG_SERVER) || defined(HTTP_RUN_AUTOMAT_SERVER)
#define WIFI_SSID "HP Laserjet-61"
#define WIFI_PASSWORD "LeidMein"
#endif

#if defined(CAMERA_APRILTAG_DETECTOR) && defined(APRILTAG_ESTIMATE_TAG_POS)
// Camera calibration data
// This information is obtained by calibrating your camera using software like 3DF Zephyr
// You have to calibrate and put your own values here, this value is just for my camera
// and likely not work on your camera.
#define FX 924.713610878 // fx (in pixel)
#define FY 924.713610878 // fy (in pixel)
#define CX 403.801748132 // cx (in pixel)
#define CY 305.082642826 // cy (in pixel)
#endif


// namespace constants {

//     constexpr int kMS_PER_FRAME = 50; // (1000/20) (FPS 20)

//     constexpr int kFRAME_WIDTH = 320; // If changing these, change mode in config
//     constexpr int kFRAME_HEIGHT = 240;

// };

// #define FRAME_RATE      2   // 10  // 20
// #define MS_PER_FRAME    490 // 95  // 50 // 1000 / 20
// #define MS_EXTRA_DELAY  10  // 5
// #define FRAME_WIDTH     320
// #define FRAME_HEIGHT    240

// #define TAG_BIT_ERROR_ALLOWED 1


#endif