
#ifndef STORM_SERIAL_UTILS_
#define STORM_SERIAL_UTILS_
/*
        WARNING!
        Keep all versions of this file in sync!
        Otherwise, serial will randomly fail or
        never work at all.
*/

#include <cstdint>
#include <string>

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
    Serial_IsManyDetections = 0x08,

    Serial_Heartbeat = 0x10,
    Serial_JoystickPacket = 0x11,
    Serial_AutonomousResume = 0x12,

    Serial_ConnectToNetwork = 0xE0
};

enum SerialAddresses {
    Address_VexBot = 0x00,
    Address_EspBot = 0x01,
    Address_Camera_1_Front = 0x05,
    Address_Camera_2_Right = 0x06,
    Address_Camera_3_Back = 0x07,
    Address_Camera_4_Left = 0x08,
    Address_Camera_5_Scoring = 0x09,

    Address_StreamingCameras = 0xE0,
};

enum NetworkIndicator {
    Network_Competition = 0x00,
    Network_Pit = 0x01//,
    // Network_Hotspot = 0x02
};

std::string getNetworkSSID(NetworkIndicator network) {
    switch (network) {
        case Network_Competition:
            return "STORM_T92";
        case Network_Pit:
            return "STORM_PIT";
        // case Network_Hotspot:
            // return "HP Laserjet-61";
    }
};
std::string getNetworkPassword(NetworkIndicator network) {
    switch (network) {
        case Network_Competition:
            return "43ZFSK7R";
        case Network_Pit:
            return "STORM_2026";
        // case Network_Hotspot:
            // return "HP Laserjet-61";
    }
};

#endif