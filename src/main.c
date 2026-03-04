
#include "camera_type.h" // Include this first because it contains #define flags

// Import ESPIDF features
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CAMERA_STREAMING
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#endif

// #include "esp32-camera-2.1.4/driver/include/esp_camera.h"
#include "esp_camera.h"

#ifdef CAMERA_APRILTAG_DETECTOR
#include "apriltag.h"
// #include "apriltag.h"
#include "tag36h11.h" // Import AprilTag Family Here
// #include "tag36h11.h" // Import AprilTag Family Here
#endif

#include <stdio.h>
#include "constants.h"

// ------------------------------ Change Wifi Credentials Here ------------------------------
#ifdef CAMERA_STREAMING
#define WIFI_SSID         "STA Guest"
#define WIFI_PASS         "STAGuest1!"
#endif

// Doesn't work on UARK Wi-Fi
// Doesn't work on UARK Guest Wi-Fi


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

camera_config_t config = { // The camera is an OV3660
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

    // .xclk_freq_hz   = 20000000,
    .xclk_freq_hz   = 16000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,

#ifdef CAMERA_APRILTAG_DETECTOR
    // .pixel_format   = PIXFORMAT_YUV422,   // IMPORTANT
    .pixel_format   = PIXFORMAT_GRAYSCALE,   // IMPORTANT
    .frame_size     = FRAMESIZE_QVGA,      // 320x240
    // .frame_size     = FRAMESIZE_VGA,      // 640x480

    .jpeg_quality   = 12,
    .fb_count       = 2,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    // .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
    .grab_mode      = CAMERA_GRAB_LATEST
#endif
#ifdef CAMERA_STREAMING
    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_QVGA,      // 320x240
    // .frame_size     = FRAMESIZE_VGA,      // 640x480
    // .frame_size     = FRAMESIZE_HD,       // 1280x720

    // .jpeg_quality   = 15,
    .jpeg_quality   = 20,
    // .fb_count       = 2,
    .fb_count       = 1,
    .fb_location    = CAMERA_FB_IN_PSRAM,
    // .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
    .grab_mode      = CAMERA_GRAB_LATEST
#endif
};


void camera_init(void) {
    ESP_LOGI("Setup", "Camera 1");
    esp_err_t err = esp_camera_init(&config);
    ESP_LOGI("Setup", "Camera 2");
    if (err != ESP_OK) {
        ESP_LOGE("Setup", "Camera init failed: %s", esp_err_to_name(err));

        esp_restart(); // The most graceful I can think of
        // return; // Or handle gracefully
    } else {
        ESP_LOGI("Setup", "Camera initialized successfully");
    }
    ESP_LOGI("Setup", "Camera 3");

    sensor_t* m_sensor = esp_camera_sensor_get();
    ESP_LOGI("Setup", "Camera 4");

    m_sensor->set_brightness(m_sensor, 0);     // -2 to 2
    m_sensor->set_contrast(m_sensor, 0);       // -2 to 2
    m_sensor->set_saturation(m_sensor, 0);     // -2 to 2
    m_sensor->set_whitebal(m_sensor, 1);       // 0 = disable , 1 = enable
    m_sensor->set_awb_gain(m_sensor, 1);       // 0 = disable , 1 = enable
    m_sensor->set_wb_mode(m_sensor, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    m_sensor->set_exposure_ctrl(m_sensor, 1);  // 0 = disable , 1 = enable
    m_sensor->set_aec2(m_sensor, 1);           // 0 = disable , 1 = enable
    m_sensor->set_ae_level(m_sensor, 0);       // -2 to 2
    m_sensor->set_aec_value(m_sensor, 168);    // 0 to 1200
    m_sensor->set_gain_ctrl(m_sensor, 1);      // 0 = disable , 1 = enable
    m_sensor->set_agc_gain(m_sensor, 0);       // 0 to 30
    m_sensor->set_gainceiling(m_sensor, (gainceiling_t)0);  // 0 to 6
    m_sensor->set_bpc(m_sensor, 0);            // 0 = disable , 1 = enable
    m_sensor->set_wpc(m_sensor, 1);            // 0 = disable , 1 = enable
    m_sensor->set_raw_gma(m_sensor, 1);        // 0 = disable , 1 = enable
    m_sensor->set_lenc(m_sensor, 1);           // 0 = disable , 1 = enable
    m_sensor->set_hmirror(m_sensor, 1);        // 0 = disable , 1 = enable
    m_sensor->set_vflip(m_sensor, 1);          // 0 = disable , 1 = enable
    m_sensor->set_dcw(m_sensor, 1);            // 0 = disable , 1 = enable
    ESP_LOGI("Setup", "Camera 5");

    sensor_t* camera = esp_camera_sensor_get();
    if (!camera) {
        ESP_LOGE("Camera", "Camera Not Detected");
    }else {
        ESP_LOGI("Camera", "Camera Setup Complete");
    }
    
    camera_fb_t *test = esp_camera_fb_get();
    if (!test) {
        ESP_LOGE("Camera", "Camera capture failed immediately");
    } else {
        ESP_LOGI("Camera", "Camera test capture OK, size=%d", test->len);
    }
    esp_camera_fb_return(test);
};

#ifdef CAMERA_APRILTAG_DETECTOR
void apriltag_loop(void* arg) {
    // TickType_t last = xTaskGetTickCount();

    // int pixel_count = FRAME_WIDTH * FRAME_HEIGHT;
    // uint8_t* greyscale = (uint8_t*) heap_caps_malloc(pixel_count, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // if (!greyscale) {
    //     ESP_LOGE("Memory", "Could not allocate uint8_t* for greyscale");
    //     vTaskDelete(NULL); // Kill itself
    // }
    // image_u8_t image = {
    //     .width  = FRAME_WIDTH,
    //     .height = FRAME_HEIGHT,
    //     .stride = FRAME_WIDTH,
    //     .buf    = greyscale
    // }; // Width, Height, Stride (=Width), Buffer

    // // struct apriltag_detector* m_detector = apriltag_detector_create();
    // apriltag_detector_t* m_detector = apriltag_detector_create();
    // // struct apriltag_family* m_at_family = tag36h11_create(); // Swap AprilTag Family Here
    // apriltag_family_t* m_at_family = tag36h11_create(); // Swap AprilTag Family Here
    // zarray_t* m_detections;

    // // Detector Settings
    // // m_detector->quad_decimate = 2.0; // 2.0 (4x) or 3.0 (9x) // Halves the resolution for processing, but restores it for coordinates
    // m_detector->quad_decimate = 1.0; // 2.0 (4x) or 3.0 (9x) // Halves the resolution for processing, but restores it for coordinates
    // m_detector->quad_sigma = 0.0; // Sets applied Gaussian blur to 0, which helps with noise but costs CPU
    // m_detector->nthreads = 1; // ESP32 is technically dual-core but AprilTag threading won’t help much (context switching, memory overhead) helps more when you can run 4-8 threads
    // m_detector->refine_edges = 0; // Removes extra step to get sub-pixel accuracy on corners, used for pose estimation (we just don't need the accuracy and it costs CPU)
    // m_detector->decode_sharpening = 0.25; // Stadard setting, Helps with noisy images
    // m_detector->debug = 0; // if 1, writes intermediate steps to disk for debug purposes
    // // apriltag_detector_add_family(m_detector, m_at_family);
    // apriltag_detector_add_family_bits(m_detector, m_at_family, TAG_BIT_ERROR_ALLOWED);

    // // Notes on resolution:
    // //      640 x 480 - est.   <5  FPS
    // //      320 x 240 - est.  5-12 FPS
    // //      160 x 120 - est. 15-25 FPS
    // // Base resolution: up to 1600 x 1200

    // // We need to see about 40 in (110 cm)
    // // Lower resolution, lower range

    // // Possible setup:
    // //      640 x 480 w/ qd 2.0 (320 x 240), 6-10 fps
    // //          Range of 60-70 cm (24-27 in)
    // //      320 x 240 w/o qd

    
    // while (true) {
    //     // Acquire image here
    //     camera_fb_t* frame_buffer = esp_camera_fb_get();
    //     if (!frame_buffer) {
    //         ESP_LOGE("Camera", "Capture Failed");
    //     }else {
    //         ESP_LOGI("Camera", "Frame Format: %d, Dimensions: %d x %d, Length: %d", frame_buffer->format, frame_buffer->width, frame_buffer->height, frame_buffer->len);

    //         // // Copy the Y component of YUV224 to get a greyscale image
    //         // for (int i = 0; i < pixel_count; i++) {
    //         //     greyscale[i] = frame_buffer->buf[i*2]; // Skip UV
    //         // }

    //         // esp_camera_fb_return(frame_buffer); // Done with actual image now

    //         // We're going simpler than that:
    //         image.buf = frame_buffer->buf;
    //         // image.stride = frame_buffer->width;

    //     // if (image == NULL) {
    //     //     fprintf(stderr, "Image could not be created.");
    //     // }else {
    //         m_detections = apriltag_detector_detect(m_detector, &image);
            
    //         esp_camera_fb_return(frame_buffer); // Warning: only if this was not already done

    //         ESP_LOGI("Targetting", "Object Frame: %d objects seen", zarray_size(m_detections));
    //         for (int i = 0; i < zarray_size(m_detections); i++) {
    //             apriltag_detection_t* detect;
    //             zarray_get(m_detections, i, &detect);

    //             // Put detection in format to send
    //             ESP_LOGI("Targetting", "Object Seen: center: x: %d, y: %d", detect->c[0], detect->c[1]);
    //         }

    //         // Send over TTL

    //         apriltag_detections_destroy(m_detections);
    //     // }
    //     // image_u8_destroy(image);
    //     // image = NULL;
    //     }
        
    //     ESP_LOGI("Heap", "Internal: %d, PSRAM: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    //     // Actually, just let it run at whatever speed
    //     // Actually, try for 10 FPS, no need to push higher
    //     vTaskDelayUntil(&last, pdMS_TO_TICKS(MS_PER_FRAME));
    //     vTaskDelay(pdMS_TO_TICKS(MS_EXTRA_DELAY)); // Wait 1 ms to make sure a wait happens
    //     // esp_task_wdt_reset(); // reset watchdog so it knows we are not stuck (the above line should do this, but is not working)
    // }

    apriltag_family_t* tf;
    apriltag_detector_t* td;
    
    tf = tag36h11_create();

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = 2.0;      // Faster on ESP32-S3
    td->quad_sigma = 0.0;
    td->nthreads = 2;             // S3 dual core
    td->debug = 0;
    td->refine_edges = 1;

    ESP_LOGI("TAG", "AprilTag detector initialized");

    while (1)
    {
        camera_fb_t *frame_buffer = esp_camera_fb_get();
        if (!frame_buffer) {
            ESP_LOGE("CAM", "Frame capture failed");
            continue;
        }

        if (frame_buffer->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGE("CAM", "Camera not in grayscale!");
            esp_camera_fb_return(frame_buffer);
            continue;
        }

        image_u8_t img = {
            .width  = frame_buffer->width,
            .height = frame_buffer->height,
            .stride = frame_buffer->width,
            .buf    = frame_buffer->buf
        };

        zarray_t *detections = apriltag_detector_detect(td, &img);

        int num = zarray_size(detections);
        ESP_LOGI("TAG", "Detections: %d", num);

        for (int i = 0; i < num; i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            ESP_LOGI("TAG",
                     "ID=%d Hamming=%d Center=(%.1f, %.1f)",
                     det->id,
                     det->hamming,
                     det->c[0],
                     det->c[1]);
        }

        apriltag_detections_destroy(detections);
        esp_camera_fb_return(frame_buffer);
    }
    // apriltag_detector_destroy(m_detector); // These should never run?
    // tag36h11_destroy(m_at_family);
    // heap_caps_free(greyscale);
    // // DO NOT RETURN
};
#endif

#ifdef CAMERA_STREAMING
// Uses ESP-IDF httpd library
// Run whenever there is a request
// Setup does not create video or anything, just sends an individual JPEG each frame forever
// '/stream' does not send html or create a webpage, it is used by index to stream the images
static esp_err_t stream_handler(httpd_req_t* request) { // static means visible in this file only (scoped to this file)
    // httpd_req_t contains http metadata about the incoming request
    // So at this point, someone has visited http://<ip-address>/stream
    camera_fb_t* frame_buffer = NULL;
    char part_buffer[64]; // Temporary storage for c-strings

    // Tell the client we are not sending one image, but many images each seperated by boundaries
    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";

    static const char* _STREAM_BOUNDARY = "\r\n--frame\r\n"; // Indicates boundary (and draws prior image)

    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n"; // Indicates image attributes

    httpd_resp_set_type(request, _STREAM_CONTENT_TYPE); // Sets the response type according to above

    while (true) { // Connection continues as long as client stays active and connected
        frame_buffer = esp_camera_fb_get();
        if (!frame_buffer) {
            ESP_LOGE("STREAM", "Camera Capture Failed");
            return ESP_FAIL; // Ends request (requires reload?)
        }

        esp_err_t still_connected = httpd_resp_send_chunk(request, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)); // Starts new frame
        if (still_connected != ESP_OK) {
            return ESP_FAIL; // Ends request, probably because client disconnected
        }

        // Sets up the header to tell the client what it is receiving
        size_t header_length = snprintf(part_buffer, sizeof(part_buffer), _STREAM_PART, frame_buffer->len);

        httpd_resp_send_chunk(request, part_buffer, header_length);

        // Send the image itself as a series of chars (a c-string)
        httpd_resp_send_chunk(request, (const char*)frame_buffer->buf, frame_buffer->len);

        httpd_resp_send_chunk(request, NULL, 0); // Specifies the end of the chunks
        // Only needed because we are sending individual chunks

        esp_camera_fb_return(frame_buffer); // We are done with the capture now

        vTaskDelay(pdMS_TO_TICKS(10)); // Wait for a moment to reset watchdogs and keep CPU below 100%
    }

    return ESP_OK;
};
// This one creates the actual html page which displays the images
static esp_err_t index_handler(httpd_req_t* request) {
    // httpd_req_t contains http metadata about the incoming request

    // Create a simple HTML webpage
    const char* html = 
        "<!DOCTYPE html>"
        "<html>"
            "<head>"
            "</head>"
            "<body>"
                "<h1>StormBot Camera</h1>"
                "<img src=\"/stream\" />"
            "</body>"
        "</html>";

    // Tell the client to expect raw HTML text
    httpd_resp_set_type(request, "text/html");
    // Send the basic webpage text once
    httpd_resp_send(request, html, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
};

void start_camera_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); // Start with default, then adjust specific

    config.server_port = 80; // Means that client does not need to specify a port 
    config.ctrl_port = 32768; // Private internal port
    config.max_uri_handlers = 8; // Number of unique requests to endpoints (/, /stream, different users)
    config.stack_size = 16384; // Memory allocated to each incoming request
    config.lru_purge_enable = true; // Drops older connections and caches

    httpd_handle_t server = NULL; // Reference to the server
    if (httpd_start(&server, &config) == ESP_OK) {
        // Server has successfully started, now we set up the pages (URI ~= page)
        httpd_uri_t index_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &index_uri); // Register the root page
        // Now whenever a browser requests '/', index_handler will be run

        httpd_uri_t stream_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = stream_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &stream_uri); // Register the stream page
        // Now whenver a browser requests '/stream', stream_handler will be run

        ESP_LOGI("HTTP", "Camera Server Started");
    }else {
        // Server failed to start
        ESP_LOGE("HTTP", "Failed to Start Camera Server");
    }
};

// Called every time a wifi event happens (callback based structure)
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START: // Fires when esp_wifi_start() completes
                ESP_LOGI("WIFI", "Connecting...");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED: // In case disconnection is temporary or just a glitch
                ESP_LOGI("WIFI", "Disconnected! Reconnecting...");
                esp_wifi_connect();
                break;
        }
    }else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data; // Interpret the void* event_data
                ESP_LOGI("WIFI", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
                start_camera_server();
        }
    }
};

// Does not guarantee static ip addresses or specific ip orders
void wifi_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init()); // NVS (Non-Volatile Storage)
    ESP_ERROR_CHECK(esp_netif_init()); // Initializes TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Creates event dispatcher

    esp_netif_create_default_wifi_sta(); // STA - client, AP - router
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT(); // Just use default settings
    ESP_ERROR_CHECK(esp_wifi_init(&config));

    // Register callbacks for the events we want to the wifi_event_handler() function above
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = { 0 };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID); // Grab the wifi network info so it knows what and how to connect
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // Prevents connecting to open networks (may remove temporarily)
    // wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true; // Tells the router "I am modern and flexible"
    wifi_config.sta.pmf_cfg.required = false;

    esp_wifi_set_mode(WIFI_MODE_STA); // STA - client, AP - router
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Actually loads config
    esp_wifi_start(); // Finally actually starts the wifi

    esp_wifi_set_ps(WIFI_PS_NONE); // Turns off power saving measures because we are streaming
};
#endif

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second debug delay

    // esp_log_level_set("*", ESP_LOG_VERBOSE);

    ESP_LOGI("Setup", "Staring Now");

    ESP_LOGI("Setup", "Internal RAM free: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI("Setup", "PSRAM free: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM)); // Does not error

    
    // while (1) {
    //     ESP_LOGI("Setup", "Internal RAM free: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    //     ESP_LOGI("Setup", "PSRAM free: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    //     ESP_LOGI("Loop", "HELLO from FreeRTOS task");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    ESP_LOGI("Setup", "Creating 6");
    ESP_LOGI("Setup", "Creating 5");
    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) == 0) { // Errors
        ESP_LOGE("Setup", "PSRAM Not Initialized");

        ESP_LOGI("Setup", "Creating 4");
        esp_restart(); // In hopes that this will fix the error mid competition
    }else {
        ESP_LOGI("Setup", "Creating 3");
        camera_init();
#ifdef CAMERA_STREAMING
        wifi_init();
#endif

        ESP_LOGI("Setup", "Creating 2");
#ifdef CAMERA_APRILTAG_DETECTOR
        TaskHandle_t m_apriltag_loop_handler;

        ESP_LOGI("Setup", "Creating 1");
        BaseType_t result = xTaskCreatePinnedToCore(apriltag_loop, "apriltag_loop", 12288, NULL, 5, &m_apriltag_loop_handler, 0); // On CPU0

        if (result != pdPASS) {
            ESP_LOGE("Setup", "Task Creation Failed");
        }
#endif
    }
};