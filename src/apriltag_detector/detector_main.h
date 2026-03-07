#include "../camera_type.h"
#ifdef CAMERA_APRILTAG_DETECTOR

#ifndef APRILTAG_DETECTOR_MAIN_
#define APRILTAG_DETECTOR_MAIN_

/*
 * AprilTag pose estimation demo on AI Thinker ESP32-CAM
 * Created by gvl610
 * Based on https://github.com/AprilRobotics/apriltag
 * with some modifications (for adaption and performance)
 */

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//                        Ensure ESP32 Wrover Module or other board with PSRAM is selected
//                        Partial images will be transmitted if image exceeds buffer size
//
//                        You must select partition scheme from the board menu that has at least 3MB APP space.
//                        Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//                        seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// Apriltag headers
// We choose 36h11 family to use in this demo, but you can use
// any family of your choice. Note that due to memory limitation,
// you might have to reduce the number of tag in `codedata` array
// in the tag family source file.
#include "apriltag.h"
#include "tag36h11.h" // Tag family. You can change
#include "common/image_u8.h"
#include "common/zarray.h"
#include "apriltag_pose.h" // For pose estimation
#include "common/matd.h"


#ifdef CAP_TO_SD
#include "SD_MMC.h"
#include "FS.h"
#endif

#include "../constants.h"

void init_apriltag_detector();

void start_apriltag_detector_loop();


#endif

#endif