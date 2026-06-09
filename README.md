# CAMtest

Dual **ESP32-CAM** stereo-vision experiment: triangulate the position of an
oncoming cyclist from their own light source, then aim an XY-servo spotlight at
the ground to flag where they are — making the rider easier to spot.

> **Status: shelved at feasibility evaluation.** The pipeline *does* detect and
> triangulate bright blobs, but light-spot detection proved too fragile in the
> real world — road reflections, stray light, and sensor noise generate too many
> false blobs to trust. Servo aiming was never wired up; the project stopped
> before actuator integration. Kept here as a reference / record of the attempt.

## Concept

```
  oncoming rider's light
          *  ───────────────►
         / \
        /   \   both cameras see the same blob at
       /     \  different horizontal pixels (disparity)
   [CAM L]  [CAM R]   ── 120.8 mm baseline ──
   secondary  primary
       │        │
       └──UART──┘
            │
       triangulate → distance
            │
       (planned) XY servo → spotlight on floor
```

Two ESP32-CAMs are mounted top-to-top on a breadboard. Each finds bright blobs
in its own frame; the horizontal pixel difference (disparity) of the same blob
across both views yields distance via stereo triangulation.

## How it works

- **Two roles, one codebase.** Built twice with different flags:
  - **`secondary`** (`-DCAM_ROLE_SECONDARY`) — the **left** camera. Detects
    blobs locally and transmits compact binary packets over its UART TX (GPIO1).
    Serial debug is suppressed: GPIO1 is shared with the blob stream, so printing
    would corrupt packet bytes.
  - **`primary`** (`-DCAM_ROLE_PRIMARY`) — the **right** camera. Runs its own
    detector, receives the secondary's blobs over `HardwareSerial(1)` (RX 13 /
    TX 12), matches blobs across both views, triangulates, and prints a per-frame
    report (frame #, FPS, scene brightness, blobs, matches, distance in metres).
- **Grayscale for speed.** Frames are captured as **800×600 (SVGA) grayscale**.
  Dropping color keeps CPU/FPS high while preserving resolution for usable
  disparity. (Fallback to VGA 640×480 documented in `config.h` if too slow.)
- **Blob detection.** 8-connectivity connected-component labeling with a merge
  step, a brightness threshold (200/255), and min/max size gating to reject
  single-pixel noise and full-frame washout. Up to 16 blobs tracked per frame.
- **Tracking / classification.** A 3-frame hysteresis tracker matches blobs
  across consecutive frames (Manhattan pixel distance) and classifies inter-frame
  motion as `STATIC_LIGHT`, `VEHICLE`, or `UNKNOWN`.
- **Triangulation.** 2D disparity model using the 120.8 mm baseline and a 62°
  horizontal FOV (OV2640 @ SVGA). Reliable range roughly 3–50 m.

## Hardware

| Item | Detail |
|------|--------|
| Boards | 2× AI-Thinker ESP32-CAM (OV2640, "8225n v2.0" module) |
| Mounting | Top-to-top on breadboard, ~120.8 mm lens-to-lens baseline |
| Link | UART @ 115200 — secondary TX (GPIO1) → primary RX (GPIO13) |
| Notes | Secondary is inverted: set `vflip=1` + `hmirror=1`. GPIO12 is a dummy TX (HardwareSerial needs both pins). GPIO13 is free only when the SD card is not initialised. |

## Build & flash (PlatformIO)

```bash
# Secondary (LEFT) first — no serial monitor, GPIO1 is the data link
pio run -e secondary -t upload

# Primary (RIGHT) — gives you the serial report
pio run -e primary -t upload
pio device monitor -b 115200
```

`fix_camera_lib.py` patches the `esp32-camera` library at build time (wired into
`platformio.ini`) — no manual step needed.

## Configuration (`src/config.h`)

| Constant | Value | Meaning |
|----------|-------|---------|
| `STEREO_BASELINE_M` | 0.1208 m | Lens-to-lens spacing |
| `FRAME_WIDTH` × `FRAME_HEIGHT` | 800 × 600 | SVGA grayscale |
| `BRIGHTNESS_THRESHOLD` | 200 / 255 | Pixel counts as "bright" |
| `MIN_BLOB_PIXELS` / `MAX_BLOB_PIXELS` | 16 / 70000 | Noise floor / washout ceiling |
| `MAX_BLOBS` | 16 | Blobs tracked per frame |
| `BLOB_MERGE_DIST` | 30 px | Centroid merge radius |
| `ROI_Y_START` / `ROI_Y_END` | 0 / 0 | Optional horizon band (0,0 = full frame) |
| `TRACKER_STATIC_THRESHOLD` / `..._VEHICLE_THRESHOLD` | ≤4 / ≥12 px | Inter-frame motion classes |
| `TRACKER_MAX_MATCH_DIST` | 25 px | Cross-frame blob match window |
| `TRACKER_CONFIRM_FRAMES` | 3 | Hysteresis persistence |
| `STEREO_HFOV_DEG` | 62.0° | OV2640 @ SVGA (calibrate for accuracy) |
| `STEREO_MIN_DISPARITY` | 1 px | Floor for a valid distance |
| `UART_PRIMARY_RX_PIN` / `_TX_PIN` | 13 / 12 | Primary UART1 (TX unused) |

## Layout

```
src/
  main.cpp               role-gated loop (primary report / secondary TX)
  camera.{h,cpp}         ESP32-CAM init & grayscale capture
  detector.{h,cpp}       8-connectivity CCL blob detection + hysteresis tracker
  triangulation.{h,cpp}  disparity → distance
  config.h               all tunables (table above)
fix_camera_lib.py        build-time esp32-camera patch
platformio.ini           board=esp32cam, framework=arduino, primary/secondary envs
sdkconfig.esp32cam       ESP-IDF sdkconfig
```

## Why it stopped — lessons

- **Light blobs are not unique.** Reflections off wet road, signs, parked cars,
  and other lights all read as the same bright blob the rider's lamp does.
- **Noise survives gating.** Even with size/brightness thresholds and 3-frame
  hysteresis, false blobs leaked through often enough to make a spotlight point
  at ghosts.
- **Stereo matching is brittle** when both cameras see multiple ambiguous blobs
  — disparity gets assigned to the wrong pair.

Conclusion: a single bright-spot cue is not a reliable signature for "oncoming
rider" in an uncontrolled outdoor scene. A more distinctive cue (modulated / IR
source, motion+shape, or an ML classifier) would be needed before the servo
spotlight stage is worth building.

## License

GPL-3.0 — see [LICENSE](LICENSE).
