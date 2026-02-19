#ifndef CAMERA_H
#define CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_camera.h"
#include "esp_err.h"

/**
 * Initialize the OV2640 camera in grayscale VGA mode.
 * Returns ESP_OK on success.
 */
esp_err_t camera_init(void);

/**
 * Capture a single frame. Returns the frame buffer pointer.
 * Caller MUST call camera_release_frame() when done with the buffer.
 * Returns NULL on failure.
 */
camera_fb_t *camera_capture_frame(void);

/**
 * Release a previously captured frame buffer back to the driver.
 */
void camera_release_frame(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_H
