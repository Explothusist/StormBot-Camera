# StormBot-Camera

Codebase for the Esp32-S3 Sense cameras on the University of Arkansas RIOT (Robotics Interdisciplinary Organization of Teams) robot built for the 2025 STORM (Student Tele-Operated Robotics Mission) competition Power Flash. The cameras are used to detect AprilTags for automatic alignment of the robot, and may, in the future, also be able to locate objects based on color.

We use esp32_camera and AprilTag3 to detect AprilTags, then send the coordinates of the tags over TTL serial. All processing is done on the camera's Esp32-S3 chip.

Used in parallel with Explothusist/StormBot-Codebase and Explothusist/StormBot-Esp32-Codebase. Our robot includes both a VEX v5 Brain and an Esp32 chip.

AprilTag3 Codebase: https://github.com/AprilRobotics/apriltag

VEX Codebase: https://github.com/Explothusist/StormBot-Codebase

Esp32 Codebase: https://github.com/Explothusist/StormBot-Esp32-Codebase

Uark RIOT website: https://riotrobotics.org/

STORM competition website: https://storm.soonerrobotics.org/about

## Build Instructions

This codebase was written using the Esp32-Camera and AprilTag3 libraries.

To obtain the AprilTag3 library, go to https://github.com/AprilRobotics/apriltag/releases/tag/v3.4.5 and download the source code in either format. Uncompress the folder and copy the contents EXCEPT FOR `CMakeLists.txt` into `components/apriltag/`. There should already be a modified `CMakeLists.txt` file in `components/apriltag/`.

To obtain the Esp32-Camera library, go to https://github.com/espressif/esp32-camera/releases/tag/v2.1.4 and download the source code in either format. Uncompress the folder and copy the contents INCLUDING `CMakeLists.txt` into `components/esp32-camera/`. There is NOT already a modified `CMakeLists.txt` file in `components/esp32-camera/`.