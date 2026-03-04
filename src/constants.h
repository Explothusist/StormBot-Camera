#ifndef STORMBOT_CAMERA_CONSTANTS_
#define STORMBOT_CAMERA_CONSTANTS_

// namespace constants {

//     constexpr int kMS_PER_FRAME = 50; // (1000/20) (FPS 20)

//     constexpr int kFRAME_WIDTH = 320; // If changing these, change mode in config
//     constexpr int kFRAME_HEIGHT = 240;

// };

#define FRAME_RATE      2   // 10  // 20
#define MS_PER_FRAME    490 // 95  // 50 // 1000 / 20
#define MS_EXTRA_DELAY  10  // 5
#define FRAME_WIDTH     320
#define FRAME_HEIGHT    240

#define TAG_BIT_ERROR_ALLOWED 1


#endif