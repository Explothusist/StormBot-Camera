#ifndef STORMBOT_CAMERA_TYPE_
#define STORMBOT_CAMERA_TYPE_

// #define CAMERA_APRILTAG_DETECTOR
#define CAMERA_STREAMING

#if defined(CAMERA_APRILTAG_DETECTOR) + defined(CAMERA_STREAMING) != 1
    #error "Camera Type: Exactly one Camera type must be defined"
#endif

#endif