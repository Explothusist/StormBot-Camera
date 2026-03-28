#ifndef STORMBOT_CAMERA_TYPE_
#define STORMBOT_CAMERA_TYPE_


// ======================
// Select camera codebase
// ======================
// #define CAMERA_APRILTAG_DETECTOR
#define CAMERA_STREAMING
// #define CAMERA_CONFIG_SERVER

#if defined(CAMERA_APRILTAG_DETECTOR) + defined(CAMERA_STREAMING) + defined(CAMERA_CONFIG_SERVER) != 1
    #error "Camera Type: Exactly one Camera type must be defined"
#endif


// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM

#ifdef CAMERA_APRILTAG_DETECTOR
    // This functionality will require heavy modification to use again
    // #define APRILTAG_ESTIMATE_TAG_POS
#endif

#ifndef CAMERA_CONFIG_SERVER
    #define HTTP_RUN_AUTOMAT_SERVER
#endif

#if defined(HTTP_USE_CONFIG_SERVER) + defined(HTTP_USE_AUTOMAT_SERVER) > 1
    #error "HTTP Type: No more than one HTTP type can be defined"
#endif

// Define CAP_TO_SD to log image to SD card (for debugging)
// This will output raw 1bpp files and then you can use
// raw2img.py to convert it to readable format.
//#define CAP_TO_SD

#endif