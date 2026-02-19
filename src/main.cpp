#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "config.h"
#include "camera.h"
#include "detector.h"
#include "triangulation.h"

// Compile-time role check — must define exactly one of CAM_ROLE_PRIMARY or
// CAM_ROLE_SECONDARY via build flags in platformio.ini.
#if !defined(CAM_ROLE_PRIMARY) && !defined(CAM_ROLE_SECONDARY)
  #error "Build flag missing: define -DCAM_ROLE_PRIMARY or -DCAM_ROLE_SECONDARY"
#endif

#define ONBOARD_LED  33  // AI-Thinker ESP32-CAM onboard LED (active low)

// ---------------------------------------------------------------------------
// UART packet format: secondary -> primary
//
// Fixed-size binary frame for minimum overhead:
//   Byte 0:      0xAA  (header / sync byte)
//   Byte 1:      blob_count  (0..MAX_BLOBS_TX)
//   Bytes 2..N:  MAX_BLOBS_TX slots * 6 bytes each:
//                  [cx_hi][cx_lo][cy_hi][cy_lo][pc_hi][pc_lo]
//
// Packet size = 2 + MAX_BLOBS_TX * 6 = 20 bytes
// At 115200 baud: ~20 * 10 / 115200 ≈ 1.7 ms — negligible vs frame time.
//
// NOTE: 0xAA can appear in blob data (e.g. cx = 170).  If sync is lost,
//       the primary discards bytes until it sees 0xAA, then reads a full
//       packet.  For a bench test this is fine.  If repeated sync loss
//       occurs, switch to a two-byte header (0xAA 0x55).
// ---------------------------------------------------------------------------
#define UART_PACKET_HEADER  0xAA
#define MAX_BLOBS_TX        3        // Blobs per packet (3 is plenty for test)
#define UART_PACKET_SIZE    (2 + MAX_BLOBS_TX * 6)   // = 20 bytes

typedef struct {
    uint16_t cx;
    uint16_t cy;
    uint16_t pixel_count;  // Capped at 65535 — fine for SVGA
} uart_blob_t;

// ---------------------------------------------------------------------------
// HardwareSerial for inter-camera link (primary side only)
// ---------------------------------------------------------------------------
#ifdef CAM_ROLE_PRIMARY
static HardwareSerial CamSerial(1);  // UART1, remapped to GPIO13 RX
#endif

// ---------------------------------------------------------------------------
// Secondary: pack and send blob data over Serial (GPIO1 / U0TXD)
// ---------------------------------------------------------------------------
#ifdef CAM_ROLE_SECONDARY
static void send_blobs_uart(const detection_result_t *result)
{
    int n = result->blob_count;
    if (n > MAX_BLOBS_TX) n = MAX_BLOBS_TX;

    Serial.write((uint8_t)UART_PACKET_HEADER);
    Serial.write((uint8_t)n);

    for (int i = 0; i < MAX_BLOBS_TX; i++) {
        uint8_t buf[6] = {0};
        if (i < n) {
            const blob_t *b = &result->blobs[i];
            buf[0] = (uint8_t)(b->cx >> 8);
            buf[1] = (uint8_t)(b->cx & 0xFF);
            buf[2] = (uint8_t)(b->cy >> 8);
            buf[3] = (uint8_t)(b->cy & 0xFF);
            uint16_t pc = (b->pixel_count > 65535u) ? 65535u
                                                     : (uint16_t)b->pixel_count;
            buf[4] = (uint8_t)(pc >> 8);
            buf[5] = (uint8_t)(pc & 0xFF);
        }
        Serial.write(buf, 6);
    }
}
#endif  // CAM_ROLE_SECONDARY

// ---------------------------------------------------------------------------
// Primary: receive a blob packet from secondary via CamSerial (UART1 / GPIO13)
// Returns true when a complete packet was parsed.
// ---------------------------------------------------------------------------
#ifdef CAM_ROLE_PRIMARY
static bool recv_blobs_uart(uart_blob_t out_blobs[], int *out_count)
{
    // Drain until we find the header byte
    while (CamSerial.available() > 0 && CamSerial.peek() != UART_PACKET_HEADER) {
        CamSerial.read();
    }

    if (CamSerial.available() < UART_PACKET_SIZE) {
        return false;  // Incomplete packet — try next frame
    }

    CamSerial.read();  // Consume header

    int n = CamSerial.read();
    if (n < 0 || n > MAX_BLOBS_TX) {
        // Corrupt count byte — flush and give up this packet
        while (CamSerial.available()) CamSerial.read();
        *out_count = 0;
        return false;
    }

    *out_count = n;
    for (int i = 0; i < MAX_BLOBS_TX; i++) {
        uint8_t buf[6];
        CamSerial.readBytes(buf, 6);
        if (i < n) {
            out_blobs[i].cx          = ((uint16_t)buf[0] << 8) | buf[1];
            out_blobs[i].cy          = ((uint16_t)buf[2] << 8) | buf[3];
            out_blobs[i].pixel_count = ((uint16_t)buf[4] << 8) | buf[5];
        }
    }
    return true;
}
#endif  // CAM_ROLE_PRIMARY

// ---------------------------------------------------------------------------
// FreeRTOS detection task — runs on core 0
// ---------------------------------------------------------------------------
static void detection_task(void *arg)
{
    uint32_t frame_num   = 0;
    int64_t  fps_timer   = esp_timer_get_time();
    uint32_t fps_count   = 0;
    float    current_fps = 0.0f;

    tracker_state_t tracker;
    tracker_reset(&tracker);

#ifdef CAM_ROLE_PRIMARY
    uart_blob_t secondary_blobs[MAX_BLOBS_TX];
    int         secondary_count = 0;
#endif

    while (1) {
        // --- Capture ---
        camera_fb_t *fb = camera_capture_frame();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Detect blobs ---
        detection_result_t result;
        detect_blobs(fb->buf, fb->width, fb->height, &result);
        camera_release_frame(fb);

        // --- Classify blobs with inter-frame tracking ---
        tracker_classify(&tracker, &result);

        // --- FPS (updated every second) ---
        fps_count++;
        int64_t now        = esp_timer_get_time();
        int64_t elapsed_us = now - fps_timer;
        if (elapsed_us >= 1000000LL) {
            current_fps = (float)fps_count * 1000000.0f / (float)elapsed_us;
            fps_count   = 0;
            fps_timer   = now;
        }

        frame_num++;

        // ================================================================
        // SECONDARY role: send blob data, no verbose serial
        // ================================================================
#ifdef CAM_ROLE_SECONDARY
        send_blobs_uart(&result);
        // Serial prints are intentionally suppressed here — GPIO1 is shared
        // between Serial debug output and the blob UART TX stream.
        // Uncomment ONLY when the secondary is NOT connected to the primary
        // (bench calibration mode):
        //
        // Serial.printf("SEC #%lu | FPS:%.1f | blobs:%d\n",
        //               (unsigned long)frame_num, current_fps,
        //               result.blob_count);
#endif

        // ================================================================
        // PRIMARY role: receive secondary data, triangulate, report
        // ================================================================
#ifdef CAM_ROLE_PRIMARY
        // Non-blocking read — use whatever is in the UART buffer
        recv_blobs_uart(secondary_blobs, &secondary_count);

        // Triangulate: match largest blob on each camera (simplest strategy)
        float distance_m = -1.0f;
        if (result.blob_count > 0 && secondary_count > 0) {
            // TODO: improve matching — use cy proximity (epipolar constraint)
            //       to handle scenes where each camera sees a different blob.
            distance_m = triangulate_distance(result.blobs[0].cx,
                                              secondary_blobs[0].cx);
        }

        // --- Serial report ---
        Serial.printf("\n--- Frame #%lu | FPS: %.1f | Brightness: %lu ---\n",
                      (unsigned long)frame_num,
                      current_fps,
                      (unsigned long)result.scene_brightness);

        if (result.blob_count == 0) {
            Serial.println("  No blobs");
        } else {
            Serial.printf("  Blobs: %d\n", result.blob_count);
            for (int i = 0; i < result.blob_count; i++) {
                blob_t *b = &result.blobs[i];
                Serial.printf(
                    "  [%d] pos=(%u,%u) size=%lu avg=%u class=%s dx=%d dy=%d\n",
                    i,
                    (unsigned)b->cx, (unsigned)b->cy,
                    (unsigned long)b->pixel_count,
                    (unsigned)blob_avg_brightness(b),
                    blob_class_str(b->classification),
                    (int)b->dx, (int)b->dy);
            }
        }

        if (secondary_count > 0) {
            Serial.printf("  Secondary: %d blob(s), blob[0] cx=%u\n",
                          secondary_count,
                          (unsigned)secondary_blobs[0].cx);
        } else {
            Serial.println("  Secondary: no data");
        }

        if (distance_m > 0.0f) {
            Serial.printf("  Distance: %.2f m\n", distance_m);
        } else {
            Serial.println("  Distance: N/A");
        }
#endif  // CAM_ROLE_PRIMARY
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(UART_BAUD);
    delay(500);

    gpio_set_direction((gpio_num_t)ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)ONBOARD_LED, 0);  // Active low — ON

#ifdef CAM_ROLE_PRIMARY
    // UART1 for receiving from secondary camera
    // RX = GPIO13, TX = -1 (we never transmit; avoids driving GPIO12 which
    // is the VDD_SDIO bootstrap pin — pulling it HIGH can cause boot issues)
    CamSerial.begin(UART_BAUD, SERIAL_8N1, UART_PRIMARY_RX_PIN, -1);
    Serial.println("=== PRIMARY CAM | Blob Detector + Stereo Triangulation ===");
#endif

#ifdef CAM_ROLE_SECONDARY
    Serial.println("=== SECONDARY CAM | Blob Sensor (UART TX) ===");
    // Warning: Serial TX (GPIO1) doubles as the blob packet wire.
    // Verbose Serial.print calls after this point will corrupt packets.
#endif

    Serial.printf("Resolution target: %dx%d SVGA\n", FRAME_WIDTH, FRAME_HEIGHT);
    Serial.printf("CPU: %lu MHz\n", (unsigned long)getCpuFrequencyMhz());
    Serial.printf("Brightness threshold: %d\n", BRIGHTNESS_THRESHOLD);
    Serial.printf("Blob size: %d - %d px\n", MIN_BLOB_PIXELS, MAX_BLOB_PIXELS);

    esp_err_t err = camera_init();
    if (err != ESP_OK) {
        Serial.printf("Camera FAILED (0x%x) — halting\n", err);
        while (1) {
            gpio_set_level((gpio_num_t)ONBOARD_LED, 1);
            delay(200);
            gpio_set_level((gpio_num_t)ONBOARD_LED, 0);
            delay(200);
        }
    }

    Serial.println("Camera OK. Starting detection task on core 0...");

    xTaskCreatePinnedToCore(
        detection_task,
        "detect",
        8192,   // Increased from 4096: tracker state + UART buffers
        NULL,
        5,
        NULL,
        0       // Core 0
    );
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
