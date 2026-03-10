#include "../camera_type.h"
#ifdef CAMERA_APRILTAG_DETECTOR

#ifndef APRILTAG_DETECTOR_UTILS_
#define APRILTAG_DETECTOR_UTILS_

// WARNING: ANY changes made to this file MUST be duplicated
//      in the corresponding files in VexBot and EspBot!

#include <cstdint>

struct __attribute__((packed)) TagDetection {
    int32_t id;
    float x;
    float y;
    float bounding_width;
    float bounding_height;
    float corners[4][2];
};
static_assert(sizeof(TagDetection) == 52, "Struct Packing Error");
typedef enum {
    CORNER_TL = 0,
    CORNER_TR = 1,
    CORNER_BR = 2,
    CORNER_BL = 3
} TagCorner;

#define EXPECTED_MAX_TAG_COUNT 5
#define MAX_TAGS_TO_TRACK 1
struct TagSetTracking {
    atmt::ThreadsafeBuffer<TagDetection>* tags[MAX_TAGS_TO_TRACK];
};

#define TAG_ROI_AREA 1.75 // (150%) Area to search when finding already identified tag
#define LOST_TAG_ROI_AREA 2.5 // (225%) Area to search when finding already identified tag that has been lost
#define SMALL_TAG_ROI_MULT 1.5
#define CYCLES_BETWEEN_FULL_SEARCH 8
#define CYCLES_BETWEEN_FULL_SEARCH_NEAR_TAG 20
#define CYCLES_BETWEEN_FULL_SEARCH_HUGE_TAG 50

#define ABS_MIN_DETECT_TAG_SIZE_PIXELS 15 // Technically not absolute minimum
#define MIN_DETECT_PIXEL_BUFFER         5
#define MIN_DETECT_PIXEL_SIZE           20
#define MIN_DETECT_TAG_SIZE_PIXELS      MIN_DETECT_PIXEL_SIZE + MIN_DETECT_PIXEL_BUFFER // Not absolute minimum, but enough for steady consistent detection
// #define MIN_DETECT_TAG_SIZE_BUFFER 1.2
#define DROP_QUAD_DEC_TO_2_THRESHOLD    MIN_DETECT_PIXEL_SIZE * 2 + MIN_DETECT_PIXEL_BUFFER
#define DROP_QUAD_DEC_TO_4_THRESHOLD    MIN_DETECT_PIXEL_SIZE * 4 + MIN_DETECT_PIXEL_BUFFER

#define VELOCITY_SCALING 1.5 // If it moved x fast, assume it will move 1.5x in that direction next frame

enum SerialIndicators {
    Serial_GetLargestDetection = 0x05,
    Serial_GetAllDetections = 0x06
};

enum SerialAddresses {
    Address_VexBot = 0x00,
    Address_EspBot = 0x01,
    Address_Camera_1 = 0x05,
    Address_Camera_2 = 0x06,
    Address_Camera_3 = 0x07,
    Address_Camera_4 = 0x08
};

#define SERIAL_ADDRESS Address_Camera_1
#define RX_PIN 16
#define TX_PIN 17


#endif

#endif