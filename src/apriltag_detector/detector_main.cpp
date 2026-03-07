#include "../camera_type.h"
#ifdef CAMERA_APRILTAG_DETECTOR

#include "detector_main.h"
#include "../constants.h"
#include "../camera_init.h"
#include <Arduino.h>

apriltag_family_t *apriltag_familty;
apriltag_detector_t *apriltag_detector;

void init_apriltag_detector() {
    // Setup AprilTag detection
#if DEBUG >= 1
    Serial.print("Init AprilTag detector... ");
#endif

    // Create tag family object
    apriltag_familty = tag36h11_create();

    // Create AprilTag detector object
    apriltag_detector = apriltag_detector_create();

    // Add tag family to the detector
    apriltag_detector_add_family_bits(apriltag_detector, apriltag_familty, 1);
    
    // Tag detector configs
    // quad_sigma is Gaussian blur's sigma
    // quad_decimate: small number = faster but cannot detect small tags
    //                                big number = slower but can detect small tags (or tag far away)
    apriltag_detector->quad_sigma = 0.0;
    apriltag_detector->quad_decimate = 4.0;
    apriltag_detector->refine_edges = 0;
    apriltag_detector->decode_sharpening = 0.25;
    apriltag_detector->nthreads = 2; // The optimal (after many tries) is 2 thread on 2 cores ESP32
    apriltag_detector->debug = 0;

    // Done init AprilTag detector
#if DEBUG >= 1
    Serial.println("done");
    Serial.print("Memory available in PSRAM: ");
    Serial.println(ESP.getFreePsram());
    Serial.println("Start detecting...");
#endif
};

void start_apriltag_detector_loop() {
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

        // Detect
        zarray_t *detections = apriltag_detector_detect(apriltag_detector, &image);
#if DEBUG >= 3
        Serial.println("done. Result:");
#endif

        // Print result
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

#ifdef APRILTAG_ESTIMATE_TAG_POS
            // Creating detection info object to feed into pose estimator
            apriltag_detection_info_t info;
            info.det = detection;
            info.tagsize = TAG_SIZE;
            info.fx = FX;
            info.fy = FY;
            info.cx = CX;
            info.cy = CY;

            // Estimate the pose
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);
            
            // Print result (position of the tag in the camera's coordinate system)
            //matd_print(pose.R, "%15f"); // Rotation matrix
            //matd_print(pose.t, "%15f"); // Translation matrix

            // Compute the yaw, pitch, and roll from the rotation matrix (and convert to degree)
            double yaw = atan2(MATD_EL(pose.R, 1, 0), MATD_EL(pose.R, 0, 0)) * RAD_TO_DEG;
            double pitch = atan2(-MATD_EL(pose.R, 2, 0), sqrt(pow(MATD_EL(pose.R, 2, 1), 2) + pow(MATD_EL(pose.R, 2, 2), 2))) * RAD_TO_DEG;
            double roll = atan2(MATD_EL(pose.R, 2, 1), MATD_EL(pose.R, 2, 2)) * RAD_TO_DEG;

#if DEBUG >= 1
            // Print the yaw, pitch, and roll of the camera
            printf("y,p,r: %15f, %15f, %15f\n", yaw, pitch, roll);
#endif
            
            // Compute the transpose of the rotation matrix
            matd_t *R_transpose = matd_transpose(pose.R);

            // Negate the translation vector
            for (int i = 0; i < pose.t->nrows; i++) {
                    MATD_EL(pose.t, i, 0) = -MATD_EL(pose.t, i, 0);
            }

            // Compute the position of the camera in the tag's coordinate system
            matd_t *camera_position = matd_multiply(R_transpose, pose.t);
#if DEBUG >= 1
            printf("x,y,z: %15f, %15f, %15f\n", MATD_EL(camera_position, 0, 0), MATD_EL(camera_position, 1, 0), MATD_EL(camera_position, 2, 0));
#endif

            // Free the matrices
            matd_destroy(R_transpose);
            matd_destroy(camera_position);
#endif
        }

        // Cleaning up
#if DEBUG >= 3
        Serial.print("Memory available in PSRAM: ");
        Serial.println(ESP.getFreePsram());
        Serial.println("Cleaning up... ");
#endif
        // Free detection result object
        apriltag_detections_destroy(detections);

        // Return camera framebuffer to the camera driver
        esp_camera_fb_return(frame_buffer);

        // Display time needed per frame
        // And frame count (if CAP_TO_SD defined)
        float t =    timeprofile_total_utime(apriltag_detector->tp) / 1.0E3;
#if DEBUG >= 1
#ifdef CAP_TO_SD
        Serial.printf("t, id: %12.3f, %zu\n", t, frame_id - 1);
#else
        Serial.printf("t: %12.3f\n", t);
#endif
#endif
    }
};


#endif