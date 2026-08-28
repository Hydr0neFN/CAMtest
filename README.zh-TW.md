[English](README.md) · **繁體中文**

# CAMtest

雙 **ESP32-CAM** 立體視覺實驗：透過迎面而來的自行車騎士自身光源進行三角測距定位，接著控制 XY 伺服馬達聚光燈對準地面標示其所在位置 — 讓騎士更容易被看見。

> **專案狀態：在可行性評估階段擱置。** 處理管線*確實*能偵測亮點區塊（bright blobs）並進行三角測距，但亮點偵測在真實世界中被證實過於脆弱 — 路面反光、雜散光與感測器雜訊產生了過多難以信任的假亮點。伺服馬達瞄準機制從未實際接線實作；本專案在整合致動器前即已中止。保留於此作為嘗試的參考與記錄。

## 概念

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

兩組 ESP32-CAM 以頂部對頂部的方式安裝在麵包板上。兩者各自在拍攝畫面中尋找亮點區塊（blob）；同一亮點在兩個視角之間的水平像素差（視差 disparity）透過立體三角測量計算出距離。

## 運作原理

- **雙重角色，單一程式碼庫。** 使用不同旗標進行兩次建置：
  - **`secondary`** (`-DCAM_ROLE_SECONDARY`) — **左側**相機。在本地端偵測亮點，並透過 UART TX (GPIO1) 傳輸精簡的二進位封包。序列埠除錯訊息已被抑制：因為 GPIO1 與亮點資料串流共用，若輸出除錯訊息會破壞封包位元組。
  - **`primary`** (`-DCAM_ROLE_PRIMARY`) — **右側**相機。執行自身的偵測器，透過 `HardwareSerial(1)` (RX 13 / TX 12) 接收來自 secondary 的亮點資料，比對兩側視角的亮點並進行三角測距，最後輸出每訊框報告（訊框編號、FPS、場景亮度、亮點數、匹配數、公尺距離）。
- **採用灰階以提升速度。** 畫面擷取為 **800×600 (SVGA) 灰階**。捨棄色彩資訊可在保持解析度以獲得足夠視差的同時，維持高 CPU 效能與高 FPS。（若速度太慢，`config.h` 中有記載降階至 VGA 640×480 的備案。）
- **亮點偵測。** 採用 8-相連連通元件標記（8-connectivity connected-component labeling, CCL）搭配合併步驟、亮度閾值（200/255）以及最小／最大尺寸過濾門檻，以剔除單像素雜訊與全畫面過曝白曝。每訊框最多追蹤 16 個亮點。
- **追蹤／分類。** 3 訊框遲滯追蹤器透過曼哈頓像素距離（Manhattan pixel distance）比對連續訊框間的亮點，並將訊框間運動分類為 `STATIC_LIGHT`、`VEHICLE` 或 `UNKNOWN`。
- **三角測距。** 利用 120.8 mm 基線與 62° 水平視野（FOV，OV2640 @ SVGA）建立 2D 視差模型。可靠測距範圍約為 3–50 公尺。

## 硬體

| 項目 | 詳細說明 |
|------|--------|
| 開發板 | 2× AI-Thinker ESP32-CAM (OV2640, "8225n v2.0" 模組) |
| 安裝方式 | 在麵包板上頂部對頂部安裝，鏡頭間距基線約 120.8 mm |
| 連線 | UART @ 115200 — secondary TX (GPIO1) → primary RX (GPIO13) |
| 備註 | Secondary 呈上下顛倒安裝：需設定 `vflip=1` + `hmirror=1`。GPIO12 為虛擬 TX（HardwareSerial 需要設定雙腳位）。GPIO13 僅在 SD 卡未初始化時才可用。 |

## 建置與燒錄 (PlatformIO)

```bash
# Secondary (LEFT) first — no serial monitor, GPIO1 is the data link
pio run -e secondary -t upload

# Primary (RIGHT) — gives you the serial report
pio run -e primary -t upload
pio device monitor -b 115200
```

`fix_camera_lib.py` 會在建置時自動修補 `esp32-camera` 函式庫（已整合進 `platformio.ini`）— 無需手動操作。

## 設定檔 (`src/config.h`)

| 常數 | 數值 | 意義 |
|----------|-------|---------|
| `STEREO_BASELINE_M` | 0.1208 m | 鏡頭中心間距（基線） |
| `FRAME_WIDTH` × `FRAME_HEIGHT` | 800 × 600 | SVGA 灰階 |
| `BRIGHTNESS_THRESHOLD` | 200 / 255 | 判定為「亮點」的像素閾值 |
| `MIN_BLOB_PIXELS` / `MAX_BLOB_PIXELS` | 16 / 70000 | 雜訊下限 / 過曝白曝上限 |
| `MAX_BLOBS` | 16 | 每訊框追蹤亮點上限 |
| `BLOB_MERGE_DIST` | 30 px | 質心合併半徑 |
| `ROI_Y_START` / `ROI_Y_END` | 0 / 0 | 選用之地平線範圍（0,0 = 全畫面） |
| `TRACKER_STATIC_THRESHOLD` / `..._VEHICLE_THRESHOLD` | ≤4 / ≥12 px | 訊框間運動分類門檻 |
| `TRACKER_MAX_MATCH_DIST` | 25 px | 跨訊框亮點匹配搜尋視窗 |
| `TRACKER_CONFIRM_FRAMES` | 3 | 遲滯持續確認訊框數 |
| `STEREO_HFOV_DEG` | 62.0° | OV2640 @ SVGA（可校準以提升精度） |
| `STEREO_MIN_DISPARITY` | 1 px | 有效距離之最小視差下限 |
| `UART_PRIMARY_RX_PIN` / `_TX_PIN` | 13 / 12 | Primary UART1（TX 未使用） |

## 檔案配置

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

## 雜訊處理 — 嘗試過的方法

亮點偵測在戶外環境下會充斥大量偽陽性（誤判），因此處理管線堆疊了數道濾波器。這些濾波機制循序執行；亮點必須通過全部檢查才會被回報為真實目標。

1. **亮度閾值** (`detector.cpp`) — 僅計算像素值 ≥ 200/255 的像素。藉此濾除昏暗背景。
2. **尺寸過濾** — 小於 16 px 的亮點視為雜訊；大於 70000 px 則為全畫面過曝白曝。兩者皆予以捨棄。
3. **8-相連 CCL + 合併** — 連通元件標記將亮像素分群；質心距離在 30 px 以內者予以合併（否則手機手電筒的雙 LED 晶粒會被判讀為兩個目標）。
4. **感測器邊緣剔除** — 位於最初/最後幾行（`cy < 3` 或 `cy > height-4`）的亮點將被捨棄；`vflip`/`hmirror` 會在邊緣產生亮線假影。
5. **自身車燈反光濾波** (`tracker_classify`) — 位於畫面底部四分之一處的大型亮點，幾乎必然是自身車燈照射路面的反光。幾何特徵明確，因此直接強制歸類為 `STATIC_LIGHT`，不參與投票判定。
6. **訊框間遲滯機制** — 亮點透過最近質心與前一訊框進行比對，且必須**連續 3 個訊框**（`TRACKER_CONFIRM_FRAMES`）判定為相同類別，該類別才會被採信。單一訊框的閃爍雜訊絕不會被確認。
7. **跨相機 X/Y 比對 — 核心過濾機制** (`main.cpp`, primary loop)。即「比對 X 與 Y 座標，唯有吻合才視為真實目標」的步驟。主要相機的亮點僅在滿足以下條件且次要相機存在對應亮點時才會被接受：
   - **正向 X 視差（positive X-disparity）** — 左側（secondary）相機看到的亮點位置必須比右側（primary）相機*更偏右*（`dx ≥ STEREO_MIN_DISPARITY`），此為前方真實物體的幾何必要條件；以及
   - **2D 空間相近（close 2D proximity）** — 曼哈頓距離 `dx + |dy|` 小於約 200 px，確保兩側視角看到的是同一個物體。

   僅被單一相機看見、或視差正負號錯誤的亮點將直接被剔除。僅此單一檢查即可消除大部分單相機的反光雜訊。
8. **三角測距合理性邊界** (`triangulation.cpp`) — 計算出的距離必須落在 0.5–80 公尺之間，且視差必須 ≥ 1 px，否則該匹配視為無效。
9. **場景全黑重設** — 當畫面中完全沒有亮點時，追蹤器會清除自身狀態，避免已離開畫面的光源殘留質心導致後續誤配。

**仍有漏網之魚。** 跨相機 X/Y 比對雖是最強的一道防線，但仍有部分虛假目標能穿透：當兩台相機確實同時看到同一個假性亮點時 — 例如濕滑路面的反光、路標看板、第二光源 — 它便會滿足正視差*以及*距離相近的條件，進而被三角測距判定為真實目標。單一特徵濾波無法將「騎士車燈」與「兩台相機都能看見的其他發光物」區分開來，這正是本專案被擱置的核心原因。

## 中止原因 — 經驗總結

- **光點特徵不具唯一性。** 濕滑路面的反光、路標、路邊停放的車輛以及其他燈光，讀取出來的亮點特徵皆與騎士車燈無異。
- **雜訊仍能穿透門檻過濾。** 即使具備尺寸／亮度閾值與 3 訊框遲滯判定，虛假亮點仍頻繁穿透，導致聚光燈指向不存在的幽靈目標。
- **立體視覺匹配極為脆弱。** 當兩台相機同時看到多個模糊亮點時 — 視差很容易被指派給錯誤的亮點配對。

結論：在不受控的戶外場景中，單一亮點特徵並非判斷「迎面而來的騎士」的可靠特徵。在具備更具辨識度的特徵（調變／紅外線光源、運動軌跡+形狀特徵，或機器學習分類器）之前，實作伺服馬達聚光燈階段尚不具實用價值。

## 授權條款

GPL-3.0 — 詳見 [LICENSE](LICENSE)。
