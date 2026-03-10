#include "../camera_type.h"
#ifdef CAMERA_APRILTAG_DETECTOR

#include "detector_main.h"
#include "../constants.h"
#include "../camera_init.h"
#include <Arduino.h>
#include <vector>

apriltag_family_t *apriltag_family;
apriltag_detector_t *apriltag_detector_dec_1;
apriltag_detector_t *apriltag_detector_dec_2;
apriltag_detector_t *apriltag_detector_dec_4;


void scan_area(image_u8_t &image, int x, int y, int width, int height, apriltag_detector_t* detector, std::vector<TagDetection> &tag_list) {
    if (x < 0 || x >= image.width ||
        y < 0 || y >= image.height ||
        width < 0 || x + width > image.width || 
        height < 0 || y + height > image.height) { // Something is out of bounds
        return;
    }
    if (width < ABS_MIN_DETECT_TAG_SIZE_PIXELS || height < ABS_MIN_DETECT_TAG_SIZE_PIXELS) { // Too small of an area to scan
        return;
    }

    // apriltag_detector->quad_decimate = quad_decimate;
    image_u8_t partial_image = {
        .width = width,
        .height = height,
        .stride = image.stride,
        .buf = image.buf + x + y * image.stride
    };

    // Detect
    zarray_t *detections = apriltag_detector_detect(detector, &partial_image);
#if DEBUG >= 3
    Serial.println("done. Result:");
#endif

    for (int i = 0; i < zarray_size(detections); i++) {
        // Get detection
        apriltag_detection_t *detection;
        zarray_get(detections, i, &detection);

#if DEBUG >= 1
        // Print tag ID
        Serial.print("ID: ");
        Serial.println(detection->id);
        Serial.printf("");

        Serial.printf("ID=%d Hamming=%d Center=(%.1f, %.1f)",
                    detection->id,
                    detection->hamming,
                    detection->c[0],
                    detection->c[1]);
#endif
        TagDetection new_detection;
        new_detection.x = x + detection->c[0];
        new_detection.y = y + detection->c[1];
        // new_detection.width = (std::hypotf( // Sqaure roots are slow, so let's go bounding box instead
        //     detection->p[0][0] - detection->p[1][0],
        //     detection->p[0][1] - detection->p[1][1]
        // ) + std::hypotf(
        //     detection->p[2][0] - detection->p[3][0],
        //     detection->p[2][1] - detection->p[3][1]
        // )) * 0.5;
        // new_detection.height = (std::hypotf(
        //     detection->p[1][0] - detection->p[2][0],
        //     detection->p[1][1] - detection->p[2][1]
        // ) + std::hypotf(
        //     detection->p[3][0] - detection->p[0][0],
        //     detection->p[3][1] - detection->p[0][1]
        // )) * 0.5;
        float min_x = detection->p[0][0];
        float max_x = detection->p[0][0];
        float min_y = detection->p[0][1];
        float max_y = detection->p[0][1];

        for (int j = 1; j < 4; j++) {
            min_x = std::min(min_x, detection->p[j][0]);
            max_x = std::max(max_x, detection->p[j][0]);
            min_y = std::min(min_y, detection->p[j][1]);
            max_y = std::max(max_y, detection->p[j][1]);
        }
        new_detection.bounding_width = max_x - min_x;
        new_detection.bounding_height = max_y - min_y;

        for (int j = 0; j < 4; j++) {
            new_detection.corners[j][0] = x + detection->p[j][0];
            new_detection.corners[j][1] = y + detection->p[j][1];
        }

        new_detection.id = detection->id;
        tag_list.push_back(new_detection);

// #ifdef APRILTAG_ESTIMATE_TAG_POS
//         // Creating detection info object to feed into pose estimator
//         apriltag_detection_info_t info;
//         info.det = detection;
//         info.tagsize = TAG_SIZE;
//         info.fx = FX;
//         info.fy = FY;
//         info.cx = CX;
//         info.cy = CY;
//
//         // Estimate the pose
//         apriltag_pose_t pose;
//         double err = estimate_tag_pose(&info, &pose);
//        
//         // Print result (position of the tag in the camera's coordinate system)
//         //matd_print(pose.R, "%15f"); // Rotation matrix
//         //matd_print(pose.t, "%15f"); // Translation matrix
//
//         // Compute the yaw, pitch, and roll from the rotation matrix (and convert to degree)
//         double yaw = atan2(MATD_EL(pose.R, 1, 0), MATD_EL(pose.R, 0, 0)) * RAD_TO_DEG;
//         double pitch = atan2(-MATD_EL(pose.R, 2, 0), sqrt(pow(MATD_EL(pose.R, 2, 1), 2) + pow(MATD_EL(pose.R, 2, 2), 2))) * RAD_TO_DEG;
//         double roll = atan2(MATD_EL(pose.R, 2, 1), MATD_EL(pose.R, 2, 2)) * RAD_TO_DEG;
//
// #if DEBUG >= 1
//         // Print the yaw, pitch, and roll of the camera
//         printf("y,p,r: %15f, %15f, %15f\n", yaw, pitch, roll);
// #endif
//        
//         // Compute the transpose of the rotation matrix
//         matd_t *R_transpose = matd_transpose(pose.R);
//
//         // Negate the translation vector
//         for (int i = 0; i < pose.t->nrows; i++) {
//                 MATD_EL(pose.t, i, 0) = -MATD_EL(pose.t, i, 0);
//         }
//
//         // Compute the position of the camera in the tag's coordinate system
//         matd_t *camera_position = matd_multiply(R_transpose, pose.t);
// #if DEBUG >= 1
//         printf("x,y,z: %15f, %15f, %15f\n", MATD_EL(camera_position, 0, 0), MATD_EL(camera_position, 1, 0), MATD_EL(camera_position, 2, 0));
// #endif
//
//         // Free the matrices
//         matd_destroy(R_transpose);
//         matd_destroy(camera_position);
// #endif
    }

    // Cleaning up
#if DEBUG >= 3
    Serial.print("Memory available in PSRAM: ");
    Serial.println(ESP.getFreePsram());
    Serial.println("Cleaning up... ");
#endif
    // Free detection result object
    apriltag_detections_destroy(detections);

#if DEBUG >= 1
        // Display time needed per frame
        // And frame count (if CAP_TO_SD defined)
        float t = timeprofile_total_utime(detector->tp) / 1.0E3;
#ifdef CAP_TO_SD
        Serial.printf("t, id: %12.3f, %zu\n", t, frame_id - 1);
#else
        Serial.printf("t: %12.3f\n", t);
#endif
#endif
};

void apriltag_loop(void* param) {
    TagSetTracking* last_tags = static_cast<TagSetTracking*>(param);
    TagDetection old_tags[MAX_TAGS_TO_TRACK]; // For ROI Tracking
    TagDetection last_seen_tags[MAX_TAGS_TO_TRACK]; // For ROI trying to find lost tags
    float last_tag_velocity[MAX_TAGS_TO_TRACK][2];
    for (int i = 0; i < MAX_TAGS_TO_TRACK; i++) {
        old_tags[i].id = -1;
        last_seen_tags[i].id = -1;
    }
    
    std::vector<TagDetection> tag_list;
    tag_list.reserve(EXPECTED_MAX_TAG_COUNT);
    // tag_list.clear();
    int cycles_since_full_search = 0;
    int nearest_tag_decimate = 0; // 0 = none, 1, 2, 4

    // Main detecting loop (we just ignore the loop() function)
    while (true) {
        // Get a frame from camera
        camera_fb_t * frame_buffer = esp_camera_fb_get();
        if (!frame_buffer) {
#if DEBUG >= 3
            Serial.println("Failed to get frame! Retrying...");
#endif
            continue;
        }

    // Capture frame to SD card
#ifdef CAP_TO_SD
        String img_path = run_id + "/" + String(frame_id) + ".raw";

#if DEBUG >= 3
        Serial.print("Saving frame to ");
        Serial.print(img_path);
        Serial.print("... ");
#endif

        // Open file
        File frame_f = fs.open(img_path.c_str(), FILE_WRITE);

        if (!frame_f) {
#if DEBUG >= 3
            Serial.println("Failed to open file in writing mode");
#endif
        } else {
            frame_f.write(frame_buffer->buf, frame_buffer->len); // payload (image), payload length
#if DEBUG >= 3
            Serial.println("done");
#endif
        }

        // Close and increase frame_id
        frame_f.close();
        frame_id++;
#endif

        // Convert our framebuffer to detector's input format
#if DEBUG >= 3
        Serial.println("Converting frame to detector's input format... ");
#endif
        image_u8_t image = {
            .width = frame_buffer->width,
            .height = frame_buffer->height,
            .stride = frame_buffer->width,
            .buf = frame_buffer->buf
        };
#if DEBUG >= 3
        Serial.println("done");
        Serial.println("Detecting... ");
#endif

        tag_list.clear();

        if (cycles_since_full_search < CYCLES_BETWEEN_FULL_SEARCH || 
            (nearest_tag_decimate >= 2 && cycles_since_full_search < CYCLES_BETWEEN_FULL_SEARCH_NEAR_TAG) ||
            (nearest_tag_decimate >= 4 && cycles_since_full_search < CYCLES_BETWEEN_FULL_SEARCH_HUGE_TAG)) {
            if (nearest_tag_decimate > 0) {
                Serial.println("Nearest Decimate Working");
                if (nearest_tag_decimate > 1) {
                    Serial.println("Nearest Decimate Working Well");
                }
            }
            if (cycles_since_full_search > CYCLES_BETWEEN_FULL_SEARCH) {
                Serial.println("Delaying Full Scan");
            }
            nearest_tag_decimate = 0;
            if (old_tags[0].id >= 0 || last_seen_tags[0].id >= 0) { // If any tag was detected last two frames
#if DEBUG >= 1
                Serial.println("Running Tag ROI");
#endif
                for (int i = 0; i < MAX_TAGS_TO_TRACK; i++) {
                    TagDetection tag = old_tags[i];
                    float roi_area = TAG_ROI_AREA;
                    if (tag.id < 0) {
                        last_tag_velocity[i][0] = 0;
                        last_tag_velocity[i][1] = 0;
                        tag = last_seen_tags[i];
                        roi_area = LOST_TAG_ROI_AREA;
                        if (tag.id < 0) {
                            continue;
                        }
                        last_seen_tags[i].id = -1; // Only use this for one frame
#if DEBUG >= 1
                        Serial.println("Running Lost Tag ROI");
#endif
                    }else {
                        last_seen_tags[i] = tag;
                        if (tag.id == old_tags[i].id) {
                            last_tag_velocity[i][0] = tag.x - old_tags[i].x;
                            last_tag_velocity[i][1] = tag.y - old_tags[i].y;
                        }
                    }
                    float tag_size = std::min(tag.bounding_width, tag.bounding_height);
                    apriltag_detector_t* detector = apriltag_detector_dec_1;
                    if (tag_size > DROP_QUAD_DEC_TO_4_THRESHOLD) {
                        detector = apriltag_detector_dec_4;
                        nearest_tag_decimate = 4;
                    }else if (tag_size > DROP_QUAD_DEC_TO_2_THRESHOLD) {
                        detector = apriltag_detector_dec_2;
                        nearest_tag_decimate = 2;
                    }else {
                        nearest_tag_decimate = 1;
                        roi_area *= SMALL_TAG_ROI_MULT;
                    }
                    int x = atmt::clamp(static_cast<int>(tag.x) + static_cast<int>(last_tag_velocity[i][0] * VELOCITY_SCALING) - static_cast<int>(tag.bounding_width * roi_area * 0.5), 0, image.width);
                    int y = atmt::clamp(static_cast<int>(tag.y) + static_cast<int>(last_tag_velocity[i][1] * VELOCITY_SCALING) - static_cast<int>(tag.bounding_height * roi_area * 0.5), 0, image.height);
                    int width = atmt::clamp(static_cast<int>(tag.bounding_width * roi_area), 0, static_cast<int>(image.width - x));
                    int height = atmt::clamp(static_cast<int>(tag.bounding_height * roi_area), 0, static_cast<int>(image.height - y));
                    scan_area(image, x, y, width, height, detector, tag_list);
                }
            }else {
#if DEBUG >= 1
                Serial.println("Lost All Tags...");
#endif
                scan_area(image, 0, 0, frame_buffer->width, frame_buffer->height, apriltag_detector_dec_2, tag_list);
                cycles_since_full_search = CYCLES_BETWEEN_FULL_SEARCH;
            }
            cycles_since_full_search += 1;
        }else {
#if DEBUG >= 1
            Serial.println("Running full scan");
#endif
            if (nearest_tag_decimate >= 4) {
                scan_area(image, 0, 0, frame_buffer->width, frame_buffer->height, apriltag_detector_dec_4, tag_list);
            }else if (nearest_tag_decimate >= 1) { // We don't need to check the for far tags
                scan_area(image, 0, 0, frame_buffer->width, frame_buffer->height, apriltag_detector_dec_2, tag_list);
            }else {
                scan_area(image, 0, 0, frame_buffer->width, frame_buffer->height, apriltag_detector_dec_1, tag_list);
            }
            cycles_since_full_search = 0;
        }

        int counter = 0;
        while (!tag_list.empty() && counter < MAX_TAGS_TO_TRACK) {
            size_t max_i = 0;
            // float max_area = (tag_list[0].width + tag_list[0].height) * 0.5f;
            float max_area = (tag_list[0].bounding_width * tag_list[0].bounding_height);
            for (size_t i = 1; i < tag_list.size(); i++) {
                // float area = (tag_list[i].width + tag_list[i].height) * 0.5f;
                float area = (tag_list[i].bounding_width * tag_list[i].bounding_height);
                if (area > max_area) {
                    max_area = area;
                    max_i = i;
                }
            }

            last_tags->tags[counter]->write(tag_list[max_i]);
            old_tags[counter] = tag_list[max_i];
            // tag_list.erase(tag_list.begin() + max_i);
            // Instead of erase, since order doesn't matter, swap and pop
            tag_list[max_i] = tag_list.back();
            tag_list.pop_back();
            counter += 1;
        }
        while (counter < MAX_TAGS_TO_TRACK) {
            TagDetection zeroed{ };
            zeroed.id = -1;

            last_tags->tags[counter]->write(zeroed);
            old_tags[counter] = zeroed;
            counter += 1;
        }


        // Return camera framebuffer to the camera driver
        esp_camera_fb_return(frame_buffer);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
};

void serial_loop(void* param) {
    TagSetTracking* last_tags = static_cast<TagSetTracking*>(param);

    atmt::SerialReader serial_reader = atmt::SerialReader(SERIAL_ADDRESS, RX_PIN, TX_PIN);

    serial_reader.init();

    while (true) {
        serial_reader.periodic();

        while (serial_reader.availableMessages()) {
            uint8_t data[atmt::kMaxPacketSize];
            uint8_t length;
            uint8_t sender;
            serial_reader.getNextMessage(data, length, sender);

            if (sender == Address_VexBot && length == 1 && data[0] == static_cast<uint8_t>(Serial_GetLargestDetection)) {
                TagDetection last_tag_detection = last_tags->tags[0]->read();
                // uint8_t tag_data[sizeof(TagDetection)];
                // memcpy(tag_data, &last_tag_detection, sizeof(TagDetection));
                serial_reader.sendMessage(Address_VexBot, reinterpret_cast<uint8_t*>(&last_tag_detection), sizeof(TagDetection));
            }else if (sender == Address_VexBot && length == 1 && data[0] == static_cast<uint8_t>(Serial_GetAllDetections)) {
                uint8_t tag_data[sizeof(TagDetection) * MAX_TAGS_TO_TRACK];
                for (int i = 0; i < MAX_TAGS_TO_TRACK; i++) {
                    TagDetection tag = last_tags->tags[i]->read();
                    memcpy(tag_data + sizeof(TagDetection) * i, &tag, sizeof(TagDetection));
                }
                serial_reader.sendMessage(Address_VexBot, tag_data, sizeof(TagDetection) * MAX_TAGS_TO_TRACK);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
};


void init_apriltag_detector() {
    // Setup AprilTag detection
#if DEBUG >= 1
    Serial.print("Init AprilTag detector... ");
#endif

    // Create tag family object
    apriltag_family = tag36h11_create();

    // Create AprilTag detector object
    apriltag_detector_dec_1 = apriltag_detector_create();
    apriltag_detector_dec_2 = apriltag_detector_create();
    apriltag_detector_dec_4 = apriltag_detector_create();

    // Add tag family to the detector
    apriltag_detector_add_family_bits(apriltag_detector_dec_1, apriltag_family, 1);
    apriltag_detector_add_family_bits(apriltag_detector_dec_2, apriltag_family, 1);
    apriltag_detector_add_family_bits(apriltag_detector_dec_4, apriltag_family, 1);
    
    apriltag_detector_dec_1->quad_sigma = 0.0;
    apriltag_detector_dec_1->quad_decimate = 1.0;
    apriltag_detector_dec_1->refine_edges = 0;
    apriltag_detector_dec_1->decode_sharpening = 0.25;
    apriltag_detector_dec_1->nthreads = 1;
    apriltag_detector_dec_1->debug = 0;

    apriltag_detector_dec_2->quad_sigma = 0.0;
    apriltag_detector_dec_2->quad_decimate = 2.0;
    apriltag_detector_dec_2->refine_edges = 0;
    apriltag_detector_dec_2->decode_sharpening = 0.25;
    apriltag_detector_dec_2->nthreads = 1;
    apriltag_detector_dec_2->debug = 0;

    apriltag_detector_dec_4->quad_sigma = 0.0;
    apriltag_detector_dec_4->quad_decimate = 4.0;
    apriltag_detector_dec_4->refine_edges = 0;
    apriltag_detector_dec_4->decode_sharpening = 0.25;
    apriltag_detector_dec_4->nthreads = 1;
    apriltag_detector_dec_4->debug = 0;

    // Done init AprilTag detector
#if DEBUG >= 1
    Serial.println("done");
    Serial.print("Memory available in PSRAM: ");
    Serial.println(ESP.getFreePsram());
    Serial.println("Start detecting...");
#endif
};

void start_apriltag_detector_loop() {

    static TagSetTracking last_tags;
    // Initialize each ThreadsafeBuffer
    for (int i = 0; i < MAX_TAGS_TO_TRACK; i++) {
        last_tags.tags[i] = new atmt::ThreadsafeBuffer<TagDetection>();
    }

    TaskHandle_t apriltag_loop_handle;
    xTaskCreatePinnedToCore(
        apriltag_loop,
        "apriltag_loop",
        24588, // Massively overkill
        &last_tags,
        5,
        &apriltag_loop_handle,
        0
    );
    
    TaskHandle_t serial_loop_handle;
    xTaskCreatePinnedToCore(
        serial_loop,
        "serial_loop",
        4096,
        &last_tags,
        5,
        &serial_loop_handle,
        1
    );
};


#endif