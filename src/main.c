
// Import ESPIDF features
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "esp32-camera-2.1.4/driver/include/esp_camera.h"
#include "esp_camera.h"

#include "apriltag-3.4.5/apriltag.h"
// #include "apriltag.h"
#include "apriltag-3.4.5/tag36h11.h" // Import AprilTag Family Here
// #include "tag36h11.h" // Import AprilTag Family Here

#include "constants.h"


#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

camera_config_t config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sccb_sda   = SIOD_GPIO_NUM,
    .pin_sccb_scl   = SIOC_GPIO_NUM,

    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,

    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,

    .pixel_format   = PIXFORMAT_YUV422,   // IMPORTANT
    .frame_size     = FRAMESIZE_QVGA,      // 320x240
    // .frame_size     = FRAMESIZE_VGA,      // 640x480

    .jpeg_quality   = 12,
    .fb_count       = 1,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
};


void camera_loop(void* arg) {
    TickType_t last = xTaskGetTickCount();

    int pixel_count = FRAME_WIDTH * FRAME_HEIGHT;
    uint8_t* greyscale = (uint8_t*) heap_caps_malloc(pixel_count, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!greyscale) {
        ESP_LOGE("Memory", "Could not allocate uint8_t* for greyscale");
        vTaskDelete(NULL); // Kill itself
    }
    image_u8_t image = {
        .width  = FRAME_WIDTH,
        .height = FRAME_HEIGHT,
        .stride = FRAME_WIDTH,
        .buf    = greyscale
    }; // Width, Height, Stride (=Width), Buffer

    struct apriltag_detector* m_detector = apriltag_detector_create();
    struct apriltag_family* m_at_family = tag36h11_create(); // Swap AprilTag Family Here
    zarray_t* m_detections;

    // Detector Settings
    m_detector->quad_decimate = 2.0; // 2.0 (4x) or 3.0 (9x) // Halves the resolution for processing, but restores it for coordinates
    m_detector->quad_sigma = 0.0; // Sets applied Gaussian blur to 0, which helps with noise but costs CPU
    m_detector->nthreads = 1; // ESP32 is technically dual-core but AprilTag threading won’t help much (context switching, memory overhead) helps more when you can run 4-8 threads
    m_detector->refine_edges = 0; // Removes extra step to get sub-pixel accuracy on corners, used for pose estimation (we just don't need the accuracy and it costs CPU)
    apriltag_detector_add_family(m_detector, m_at_family);

    // Notes on resolution:
    //      640 x 480 - est.   <5  FPS
    //      320 x 240 - est.  5-12 FPS
    //      160 x 120 - est. 15-25 FPS
    // Base resolution: up to 1600 x 1200

    // We need to see about 40 in (110 cm)
    // Lower resolution, lower range

    // Possible setup:
    //      640 x 480 w/ qd 2.0 (320 x 240), 6-10 fps
    //          Range of 60-70 cm (24-27 in)
    //      320 x 240 w/o qd

    
    while (true) {
        // Acquire image here
        camera_fb_t* frame_buffer = esp_camera_fb_get();
        if (!frame_buffer) {
            ESP_LOGE("Camera", "Capture Failed");
        }else {
            // Copy the Y component of YUV224 to get a greyscale image
            for (int i = 0; i < pixel_count; i++) {
                greyscale[i] = frame_buffer->buf[i*2]; // Skip UV
            }

        // if (image == NULL) {
        //     fprintf(stderr, "Image could not be created.\n");
        // }else {
            m_detections = apriltag_detector_detect(m_detector, &image);

            for (int i = 0; i < zarray_size(m_detections); i++) {
                apriltag_detection_t* detect;
                zarray_get(m_detections, i, &detect);

                // Put detection in format to send
            }

            // Send over TTL

            apriltag_detections_destroy(m_detections);
        // }
        // image_u8_destroy(image);
        // image = NULL;
        
            esp_camera_fb_return(frame_buffer);
        }
        

        // Actually, just let it run at whatever speed
        // vTaskDelayUntil(&last, pdMS_TO_TICKS(constants::kMS_PER_FRAME));
    }
    apriltag_detector_destroy(m_detector); // These should never run?
    tag36h11_destroy(m_at_family);
    heap_caps_free(greyscale);
    // DO NOT RETURN
};

void app_main(void) {
    ESP_ERROR_CHECK(esp_camera_init(&config));
    
    TaskHandle_t m_camera_loop_handler;

    BaseType_t result = xTaskCreate(camera_loop, "camera_loop", 12288, NULL, 5, &m_camera_loop_handler);

    if (result != pdPASS) {
        ESP_LOGE("Setup", "Task Creation Failed");
    }

};