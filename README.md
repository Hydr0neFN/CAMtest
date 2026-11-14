**English** · [繁體中文](README.zh-TW.md)

# CAMtest

A stereo-vision experiment: two ESP32-CAM boards try to triangulate the position
of an oncoming cyclist from their own light source. The plan was to aim an
XY-servo spotlight at the ground to flag where they are.

## At a glance

| | |
|---|---|
| Problem | Make an oncoming rider easier to spot: aim a floor spotlight at the rider's position. |
| What I built | Two ESP32-CAM boards in a stereo pair, blob detection on each board, a UART link, and stereo triangulation of the rider's light. |
| Result | The pipeline detects and triangulates bright blobs. Light-spot detection was too fragile in the real world. The servo aim was never wired up. |
| Stack | ESP32-CAM ×2 (OV2640), Arduino framework, PlatformIO, UART @ 115200, SVGA 800×600 grayscale |
| Status | archived — shelved at feasibility evaluation |

## How it works

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

Two ESP32-CAMs are mounted top-to-top on a breadboard. Each board finds bright
blobs in its own frame. The horizontal pixel difference (disparity) of the same
blob across both views gives the distance by stereo triangulation.

## The two boards

- **`secondary`** (`-DCAM_ROLE_SECONDARY`) — the **left** camera. It detects
  blobs locally and sends compact binary packets over its UART TX (GPIO1).
  Serial debug is off: GPIO1 is shared with the blob stream, so a print would
  corrupt packet bytes.
- **`primary`** (`-DCAM_ROLE_PRIMARY`) — the **right** camera. It runs its own
  detector, receives the secondary's blobs over `HardwareSerial(1)` (RX 13 /
  TX 12), matches blobs across both views, triangulates, and prints a per-frame
  report (frame #, FPS, scene brightness, blobs, matches, distance in metres).

## What was tried

The detector uses these settings:

- **Grayscale for speed.** Frames are captured as **800×600 (SVGA) grayscale**.
  No color keeps CPU/FPS high and keeps the resolution for a usable disparity.
  A fallback to VGA 640×480 is documented in `config.h` if the frame rate is
  too slow.
- **Blob detection.** 8-connectivity connected-component labeling with a merge
  step, a brightness threshold (200/255), and min/max size gating to reject
  single-pixel noise and full-frame washout. Up to 16 blobs are tracked per
  frame.
- **Tracking / classification.** A 3-frame hysteresis tracker matches blobs
  across consecutive frames (Manhattan pixel distance) and classifies the
  inter-frame motion as `STATIC_LIGHT`, `VEHICLE`, or `UNKNOWN`.
- **Triangulation.** A 2D disparity model uses the 120.8 mm baseline and a 62°
  horizontal FOV (OV2640 @ SVGA). Reliable range is roughly 3–50 m.

## Noise handling

Bright-spot detection has too many false positives outdoors. The pipeline
stacks filters in order. A blob must survive all of them before the code
reports it as a real target.

1. **Brightness threshold** (`detector.cpp`) — only pixels ≥ 200/255 count.
2. **Size gating** — blobs smaller than 16 px are noise. Blobs larger than
   70000 px are whole-frame washout. The code drops both.
3. **8-connectivity CCL + merge** — centroids within 30 px are merged. A phone
   flashlight's two LED dies would otherwise read as two targets.
4. **Sensor-edge rejection** — blobs in the first/last few rows (`cy < 3` or
   `cy > height-4`) are discarded. `vflip`/`hmirror` produce bright-line
   artifacts there.
5. **Own-headlight reflection filter** (`tracker_classify`) — a large bright
   blob in the bottom quarter of the frame is almost surely our own headlight
   bouncing off the road. The geometry is conclusive, so the code
   force-classifies it as `STATIC_LIGHT` with no voting.
6. **Inter-frame hysteresis** — a blob must agree on the same class for **3
   consecutive frames** (`TRACKER_CONFIRM_FRAMES`) before the class is trusted.
   Single-frame flickers never get confirmed.
7. **Cross-camera X/Y match — the main filter** (`main.cpp`, primary loop).
   The primary accepts a blob only if the secondary sees a blob with:
   - **positive X-disparity** — the left camera sees the blob *further right*
     than the right camera (`dx ≥ STEREO_MIN_DISPARITY`), which geometry
     requires for a real object in front; and
   - **close 2D proximity** — Manhattan distance `dx + |dy|` under ~200 px.

   A blob seen by only one camera, or with the wrong disparity sign, is
   rejected outright.
8. **Triangulation sanity bounds** (`triangulation.cpp`) — the distance must be
   in 0.5–80 m and the disparity must be ≥ 1 px.
9. **Scene-dark reset** — with no blobs present, the tracker clears its state.
   A stale centroid from a light that has left the frame cannot mismatch later.

The full list goes to
[docs/noise-handling.md](docs/noise-handling.md).

## The negative result

**The cross-camera X/Y match is the strongest layer, but false targets still
pass it.** When both cameras really see the same spurious bright spot — a
wet-road reflection, a sign, a second light source — it has positive disparity
*and* proximity, so the code triangulates it as a real target. No single-cue
filter separates "rider's lamp" from "any other bright thing both cameras can
see". This is the core reason the project was shelved.

Lessons:

- **Light blobs are not unique.** Reflections off wet road, signs, parked cars,
  and other lights all read as the same bright blob as the rider's lamp.
- **Noise survives gating.** Even with size/brightness thresholds and 3-frame
  hysteresis, false blobs leaked through often enough to make a spotlight point
  at ghosts.
- **Stereo matching is brittle** when both cameras see multiple ambiguous
  blobs. Disparity gets assigned to the wrong pair.

Conclusion: a single bright-spot cue is not a reliable signature for "oncoming
rider" in an uncontrolled outdoor scene. A more distinctive cue (modulated / IR
source, motion+shape, or an ML classifier) would be needed before the servo
spotlight stage is worth building.

The full write-up is in [docs/why-it-stopped.md](docs/why-it-stopped.md).

## Hardware

| Item | Detail |
|------|--------|
| Boards | 2× AI-Thinker ESP32-CAM (OV2640, "8225n v2.0" module) |
| Mounting | Top-to-top on breadboard, ~120.8 mm lens-to-lens baseline |
| Link | UART @ 115200 — secondary TX (GPIO1) → primary RX (GPIO13) |
| Notes | Secondary is inverted: set `vflip=1` + `hmirror=1`. GPIO12 is a dummy TX (HardwareSerial needs both pins). GPIO13 is free only when the SD card is not initialised. |

## Build / run

`fix_camera_lib.py` patches the `esp32-camera` library at build time (wired
into `platformio.ini`). You need no manual step.

```bash
# Secondary (LEFT) first — no serial monitor, GPIO1 is the data link
pio run -e secondary -t upload

# Primary (RIGHT) — gives you the serial report
pio run -e primary -t upload
pio device monitor -b 115200
```

All tunables are in `src/config.h`. The table of constants is in
[docs/configuration.md](docs/configuration.md).

## Repo layout

```
CMakeLists.txt
LICENSE                  GPL-3.0
README.md                English main
README.zh-TW.md          Traditional Chinese
docs/                    detail moved out of this README
fix_camera_lib.py        build-time esp32-camera patch
include/                 PlatformIO stub
lib/                     PlatformIO stub
platformio.ini           board=esp32cam, framework=arduino, primary/secondary envs
sdkconfig.esp32cam       ESP-IDF sdkconfig
src/
  main.cpp               role-gated loop (primary report / secondary TX)
  camera.{h,cpp}         ESP32-CAM init & grayscale capture
  detector.{h,cpp}       8-connectivity CCL blob detection + hysteresis tracker
  triangulation.{h,cpp}  disparity → distance
  config.h               all tunables
test/                    PlatformIO stub
.vscode/
```

## Limits and open items

- **Servo aim was never wired up.** The project stopped before actuator
  integration. The XY-servo spotlight is planned only.
- **The main filter leaks.** A false bright spot that both cameras see passes
  the X/Y match (see [the negative result](#the-negative-result)).
- **The FOV needs calibration.** Calibrate `STEREO_HFOV_DEG` for accuracy.
- **The baseline is a breadboard number.** The two boards sit top-to-top on a
  breadboard, about 120.8 mm lens-to-lens.
- **No measurement is recorded.** The old README gives no measured false-blob
  rate or accuracy figure.

## License

GPL-3.0 — see [LICENSE](LICENSE).
