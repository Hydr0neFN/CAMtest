**English** · [繁體中文](configuration.zh-TW.md)

# Configuration (`src/config.h`)

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

For the board layout, see the [README](../README.md#hardware).
