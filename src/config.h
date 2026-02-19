#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Camera pin mapping — AI-Thinker ESP32-CAM board
// ---------------------------------------------------------------------------
#define CAM_PIN_PWDN     32
#define CAM_PIN_RESET    -1  // Not connected
#define CAM_PIN_XCLK      0
#define CAM_PIN_SIOD     26
#define CAM_PIN_SIOC     27

#define CAM_PIN_D7       35
#define CAM_PIN_D6       34
#define CAM_PIN_D5       39
#define CAM_PIN_D4       36
#define CAM_PIN_D3       21
#define CAM_PIN_D2       19
#define CAM_PIN_D1       18
#define CAM_PIN_D0        5

#define CAM_PIN_VSYNC    25
#define CAM_PIN_HREF     23
#define CAM_PIN_PCLK     22

#define CAM_XCLK_FREQ_HZ 20000000  // 20 MHz XCLK

// ---------------------------------------------------------------------------
// Frame settings — SVGA 800x600 (try first; fall back to VGA 640x480 if too slow)
// These values drive serial reporting and triangulation focal-length math.
// The actual buffer dimensions come from fb->width / fb->height at runtime.
// To fall back to VGA: change FRAMESIZE_SVGA -> FRAMESIZE_VGA in camera.cpp
//                      and set FRAME_WIDTH=640, FRAME_HEIGHT=480 here.
// ---------------------------------------------------------------------------
#define FRAME_WIDTH   800
#define FRAME_HEIGHT  600

// ---------------------------------------------------------------------------
// Blob detection tuning — scaled for SVGA (800x600 = 480,000 px)
// ---------------------------------------------------------------------------
#define BRIGHTNESS_THRESHOLD  200   // Pixel brightness to count as "bright" (0-255)
#define MIN_BLOB_PIXELS        16   // Ignore blobs smaller than this (noise)
#define MAX_BLOB_PIXELS     70000   // Ignore blobs larger than this (whole-frame wash)
#define MAX_BLOBS               16  // Max number of blobs to track per frame

// Region of interest — restrict detection to a vertical band (horizon area)
// Set both to 0 to use the full frame
#define ROI_Y_START    0            // Top row of ROI (0 = top of frame)
#define ROI_Y_END      0            // Bottom row of ROI (0 = use full frame)

// ---------------------------------------------------------------------------
// UART inter-camera link
// ---------------------------------------------------------------------------
// Secondary: transmits blob packets on GPIO1 (U0TXD — the default Serial TX).
//   NOTE: GPIO1 is shared with Serial debug output. On the secondary, do NOT
//   print verbose debug to Serial while the detection task is running, or blob
//   packet bytes will be corrupted. Use a logic analyzer for secondary debug.
//
// Primary: receives on GPIO13 via HardwareSerial(1).
//   GPIO13 is free on AI-Thinker ESP32-CAM when SD card is not initialised.
//   GPIO12 is configured as TX (unused) as HardwareSerial requires both pins.
#define UART_PRIMARY_RX_PIN    13
#define UART_PRIMARY_TX_PIN    12   // Unused — required by HardwareSerial API
#define UART_BAUD             115200

// ---------------------------------------------------------------------------
// Stereo triangulation geometry
// ---------------------------------------------------------------------------
// Baseline: physical separation (metres) between the two camera lenses.
// Adjust STEREO_BASELINE_M to your actual mount before trusting distance readings.
#define STEREO_BASELINE_M      0.15f   // 15 cm default — measure and update!

// OV2640 horizontal FOV. At SVGA the effective HFOV is approximately 62°.
// This is an approximation — calibrate against a known-distance target for accuracy.
#define STEREO_HFOV_DEG       62.0f

// Minimum disparity (pixels) to return a valid distance estimate.
// At SVGA with 15 cm baseline, reliable range is roughly 3–50 m.
#define STEREO_MIN_DISPARITY    1

// ---------------------------------------------------------------------------
// Blob tracker / classification thresholds
// All distances are Manhattan pixels (|dx| + |dy|) between matched blob
// centroids across consecutive frames.
// Scaled for SVGA (800px wide ≈ 1.25x VGA).
// ---------------------------------------------------------------------------
#define TRACKER_STATIC_THRESHOLD    4   // <= this -> STATIC_LIGHT
#define TRACKER_VEHICLE_THRESHOLD  12   // >= this -> VEHICLE  (else UNKNOWN)
#define TRACKER_MAX_MATCH_DIST     25   // Max px distance to match blob across frames
#define TRACKER_CONFIRM_FRAMES      3   // Consecutive frames of agreement to confirm class

// ---------------------------------------------------------------------------
// Future work (NOT implemented):
//   - Correlate blob inter-frame motion with accelerometer / hall-effect wheel
//     speed to subtract bike-motion parallax before classifying blobs.
//     This would distinguish "streetlamp drifting due to bike movement" from
//     "oncoming vehicle with its own lateral velocity".
// ---------------------------------------------------------------------------

#endif // CONFIG_H
