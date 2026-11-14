[English](configuration.md) · **繁體中文**

# 設定檔（`src/config.h`）

| 常數 | 數值 | 意義 |
|----------|-------|---------|
| `STEREO_BASELINE_M` | 0.1208 m | 鏡頭中心間距（基線） |
| `FRAME_WIDTH` × `FRAME_HEIGHT` | 800 × 600 | SVGA 灰階 |
| `BRIGHTNESS_THRESHOLD` | 200 / 255 | 判定為「亮點」的像素閾值 |
| `MIN_BLOB_PIXELS` / `MAX_BLOB_PIXELS` | 16 / 70000 | 雜訊下限 / 過曝上限 |
| `MAX_BLOBS` | 16 | 每訊框追蹤亮點上限 |
| `BLOB_MERGE_DIST` | 30 px | 質心合併半徑 |
| `ROI_Y_START` / `ROI_Y_END` | 0 / 0 | 選用的地平線範圍（0,0 = 全畫面） |
| `TRACKER_STATIC_THRESHOLD` / `..._VEHICLE_THRESHOLD` | ≤4 / ≥12 px | 訊框間運動分類門檻 |
| `TRACKER_MAX_MATCH_DIST` | 25 px | 跨訊框亮點匹配距離上限 |
| `TRACKER_CONFIRM_FRAMES` | 3 | 遲滯要連續確認的訊框數 |
| `STEREO_HFOV_DEG` | 62.0° | OV2640 @ SVGA（可校準以提升精度） |
| `STEREO_MIN_DISPARITY` | 1 px | 有效距離的最小視差 |
| `UART_PRIMARY_RX_PIN` / `_TX_PIN` | 13 / 12 | Primary UART1（TX 未使用） |

硬體配置請回到 [README](../README.zh-TW.md#硬體)。
