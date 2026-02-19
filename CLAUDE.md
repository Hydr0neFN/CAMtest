# CAMtest — Bike Auto High/Low Beam System

## Project Overview
Dual AI-Thinker ESP32-CAM stereo blob detector for automatic headlight beam switching.
Serial output only (no LED control yet). PlatformIO + Arduino framework.

## Hardware
- **Sensor**: "8225n v2.0" camera module (NOT OV2640, but esp32-camera driver compatible)
- **Board**: AI-Thinker ESP32-CAM, 240MHz CPU, 4MB PSRAM
- **Mount**: Top-to-top on breadboard → secondary needs vflip=1 + hmirror=1
- **Baseline**: 120.8mm lens-to-lens (`STEREO_BASELINE_M = 0.1208f`)
- **UART**: Secondary GPIO1 (U0TXD) → Primary GPIO13 (UART1 RX), TX=-1 on primary (GPIO12 bootstrap risk)

## Architecture
- `env:primary` = RIGHT camera: blob detect + UART RX + triangulation + serial report
- `env:secondary` = LEFT camera: blob detect + 20-byte binary UART TX on GPIO1
- SVGA 800x600 grayscale, ~2 FPS
- 8-connectivity connected-component labeling, blob merge (30px), edge filter (cy<3)
- N-frame hysteresis tracker (3 frames) with vote-state remapping via match_map[]
- **2D disparity** triangulation: `sqrt(dx²+dy²)` — works when bike leans into turns
- Matching requires positive X-disparity + 2D proximity scoring (handles roll)

## Key Bugs Previously Fixed
1. Vote state remapping: votes indexed by old-frame slot, centroids by new → match_map[] + temp arrays
2. Disparity sign: must be `x_secondary - x_primary` (left minus right)
3. GPIO12 bootstrap: TX pin = -1 avoids driving VDD_SDIO high
4. Phantom cy=1 blobs: vflip edge artifacts → filter cy<3 and cy>height-4
5. 4-connectivity blob splitting → upgraded to 8-connectivity + merge step

## File Structure
```
src/config.h           - Tuning constants, pins, stereo geometry
src/camera.h/cpp       - ESP32-CAM init (SVGA grayscale), vflip/hmirror per role
src/detector.h/cpp     - Blob detection (8-conn CC + merge), tracker with hysteresis
src/triangulation.h/cpp - 2D stereo distance estimation
src/main.cpp           - Role-based (#ifdef), UART protocol, FreeRTOS task core 0
platformio.ini         - env:primary + env:secondary, 240MHz, -O2, ccache
```

## Pending / Future Work
- LED control (LEDs not arrived)
- Accelerometer / hall-effect wheel speed (subtract parallax before classification)
- Calibrate HFOV (estimated 62°) against known-distance target
- ROI cropping to improve FPS
- Final bike mount baseline measurement
