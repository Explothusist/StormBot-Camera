
#include "camera_init.h"

#include <Arduino.h>

void init_camera() {
#if DEBUG >= 1
    Serial.begin(115200);
#endif
#if DEBUG >= 2
    Serial.setDebugOutput(true);
#endif

    // Show some signs of life
#if DEBUG >= 1
    Serial.println("AprilTag pose estimation demo on ESP32-CAM");
    Serial.print("Init PSRAM... ");
#endif

    // Init PSRAM
    psramInit();
#if DEBUG >= 1
    Serial.println("done");
    Serial.print("Memory available in PSRAM: ");
    Serial.println(ESP.getFreePsram());
#endif

    // Create camera config structure
    camera_config_t config;

    // Set camera pins (as defined in cam_pins.h)
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    // Set clock frequency
    config.xclk_freq_hz = 20000000;

    // Set frame config
    config.frame_size = FRAMESIZE_SVGA; // You can change the resolution to fit your need
    config.pixel_format = PIXFORMAT_GRAYSCALE; // Required for AprilTag processing
    config.grab_mode = CAMERA_GRAB_LATEST; // Has to be in this mode, or detection will be lag
    config.fb_location = CAMERA_FB_IN_PSRAM;
    //config.jpeg_quality = 12;
    config.fb_count = 1; // Can't afford (and also not needed) to have 2

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    // Init camera
#if DEBUG >= 1
    Serial.print("Init camera... ");
#endif
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        // Cannot init camera
#if DEBUG >= 1
        Serial.printf("Camera init failed with error 0x%x\n", err);
#endif

        // Reset (cannot let it die here)
        ESP.restart();
    }

    // Specific configs for different camera
    sensor_t * s = esp_camera_sensor_get();
    // Initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1); // flip it back
        s->set_brightness(s, 1); // up the brightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM) || defined(CAMERA_MODEL_AI_THINKER)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
    s->set_vflip(s, 1);
#endif

    // Custom camera configs should go here
    s->set_brightness(s, 0);         // -2 to 2
    s->set_contrast(s, 0);             // -2 to 2
    s->set_saturation(s, 0);         // -2 to 2
    s->set_whitebal(s, 1);             // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);             // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);                // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);    // 0 = disable , 1 = enable
    s->set_aec2(s, 1);                     // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);             // -2 to 2
    s->set_aec_value(s, 168);        // 0 to 1200
    s->set_gain_ctrl(s, 1);            // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);             // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);    // 0 to 6
    s->set_bpc(s, 0);                        // 0 = disable , 1 = enable
    s->set_wpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);                // 0 = disable , 1 = enable
    s->set_lenc(s, 1);                     // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);                // 0 = disable , 1 = enable
    s->set_vflip(s, 1);                    // 0 = disable , 1 = enable
    s->set_dcw(s, 1);                        // 0 = disable , 1 = enable

    // Done init camera
#if DEBUG >= 1
    Serial.println("done");
    Serial.print("Memory available in PSRAM: ");
    Serial.println(ESP.getFreePsram());
#endif

    // Init SD card if needed
#ifdef CAP_TO_SD
#if DEBUG >= 1
    Serial.print("Init SD card... ");
#endif

    // Mount SD card
    if (SD_MMC.begin()) {
#if DEBUG >= 1
        Serial.println("done");
#endif
    } else {
#if DEBUG >= 1
        Serial.println("failed. SD card capture will be disabled!");
#endif
    }

    // FS object
    fs::FS &fs = SD_MMC;

    // Create "apriltag_detection" directory (if it has not yet existed)
    fs.mkdir("/apriltag_detection");

    // Determine the current run ID by looking into apriltag_detection directory
    // and find the directory with the largest number. Our ID will be that + 1
    String run_id = "0";
    File apriltag_detection_dir = fs.open("/apriltag_detection");
    File run_dir;
    while (run_dir = apriltag_detection_dir.openNextFile()) {
        if (run_dir.isDirectory()) {
            run_id = run_dir.name();
        }
    }

    // Create new directory
    run_id = "/apriltag_detection/" + String(run_id.toInt() + 1);
    fs.mkdir(run_id.c_str());

#if DEBUG >= 1
    Serial.print("Images will be saved to ");
    Serial.print(run_id);
    Serial.println("/");
#endif

    // Image id counter
    uint32_t frame_id = 0;
#endif
};
