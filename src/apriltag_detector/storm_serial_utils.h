
#ifndef STORM_SERIAL_UTILS_
#define STORM_SERIAL_UTILS_
/*
        WARNING!
        Keep all versions of this file in sync!
        Otherwise, serial will randomly fail or
        never work at all.
*/

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

enum SerialIndicators {
    Serial_GetLargestDetection = 0x05, // VexBot requesting tag data
    Serial_GetAllDetections = 0x06,
    Serial_IsSingleDetection = 0x07, // Cameras sending tag data
    Serial_IsManyDetections = 0x08
};

enum SerialAddresses {
    Address_VexBot = 0x00,
    Address_EspBot = 0x01,
    Address_Camera_1_Front = 0x05,
    Address_Camera_2_Right = 0x06,
    Address_Camera_3_Back = 0x07,
    Address_Camera_4_Left = 0x08,
    Address_Camera_5_Scoring = 0x09
};

#endif