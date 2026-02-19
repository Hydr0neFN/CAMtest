#include "camera.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "camera";

esp_err_t camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,

        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href  = CAM_PIN_HREF,
        .pin_pclk  = CAM_PIN_PCLK,

        .xclk_freq_hz = CAM_XCLK_FREQ_HZ,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format  = PIXFORMAT_GRAYSCALE,
        .frame_size    = FRAMESIZE_SVGA,      // 800x600 (fall back to FRAMESIZE_VGA if too slow)
        .jpeg_quality  = 0,                   // Not used for grayscale
        .fb_count      = 2,                   // Double-buffer in PSRAM
        .fb_location   = CAMERA_FB_IN_PSRAM,
        .grab_mode     = CAMERA_GRAB_LATEST,  // Always get the newest frame
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    // Apply sensor-level image orientation corrections — zero CPU cost.
    // Top-to-top breadboard mounting rotates one PCB 180° in-plane, which is
    // equivalent to vflip=1 AND hmirror=1 together (a 180° image rotation).
    // hmirror must be ON so the secondary's X axis runs left-to-right the same
    // way as the primary — critical for disparity to have the correct sign.
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
#ifdef CAM_ROLE_SECONDARY
        s->set_vflip(s, 1);    // Correct upside-down rows
        s->set_hmirror(s, 1);  // Correct left-right mirror from 180° rotation
        ESP_LOGI(TAG, "Secondary: vflip ON, hmirror ON (top-to-top mount)");
#else
        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);
        ESP_LOGI(TAG, "Primary: vflip OFF, hmirror OFF");
#endif
    }

    ESP_LOGI(TAG, "Camera initialized: GRAYSCALE SVGA 800x600, 2 frame buffers in PSRAM");
    return ESP_OK;
}

camera_fb_t *camera_capture_frame(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Frame capture failed");
        return NULL;
    }
    return fb;
}

void camera_release_frame(camera_fb_t *fb)
{
    if (fb) {
        esp_camera_fb_return(fb);
    }
}
