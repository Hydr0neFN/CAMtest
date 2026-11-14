[English](README.md) · **繁體中文**

# CAMtest

立體視覺實驗：兩塊 ESP32-CAM 想靠迎面騎士自己的光源，用三角測距算出他的位置。原本計畫再用 XY 伺服聚光燈打在地上，標出他在哪。

## 一眼看完

| | |
|---|---|
| 遇到的問題 | 讓迎面來的騎士更容易被看見：原本的目標是讓地上的聚光燈直接打在他的位置上。 |
| 做了什麼 | 兩塊 ESP32-CAM 組成一個立體相機，各自偵測亮點，用 UART 互傳資料，再對騎士的光源做立體三角測距。 |
| 結果 | 管線能偵測亮點也能三角測距，但亮點偵測在真實環境下太脆弱。伺服馬達瞄準從頭到尾沒接上。 |
| 技術 | ESP32-CAM ×2（OV2640）、Arduino framework、PlatformIO、UART @ 115200、SVGA 800×600 灰階 |
| 狀態 | 已封存 — 在可行性評估階段擱置 |

## 運作方式

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

兩塊 ESP32-CAM 頂部對頂部裝在麵包板上。每塊板子各自在自己的畫面裡找亮點區塊。同一個亮點在兩個視角下的水平像素差（視差 disparity），用立體三角測量換算成距離。

## 兩塊板子

- **`secondary`**（`-DCAM_ROLE_SECONDARY`）—— **左**側相機。在本地端偵測亮點，透過 UART TX（GPIO1）送出精簡的二進位封包。因為 GPIO1 和亮點資料串流共用，除錯訊息一律不印，否則封包位元組會被弄壞。
- **`primary`**（`-DCAM_ROLE_PRIMARY`）—— **右**側相機。自己跑一份偵測器，用 `HardwareSerial(1)`（RX 13 / TX 12）接收 secondary 的亮點，比對兩邊視角、做三角測距，最後印出每訊框報表（訊框編號、FPS、場景亮度、亮點數、匹配數、公尺距離）。

## 試過哪些做法

偵測器用的設定如下：

- **灰階換速度。** 畫面擷取為 **800×600（SVGA）灰階**。去掉色彩能保住 CPU 效能和 FPS，解析度夠，視差才堪用。畫面速度太慢的話，`config.h` 裡寫了降成 VGA 640×480 的備案。
- **亮點偵測。** 8-相連連通元件標記（CCL）外加合併步驟、亮度閾值（200/255），再用最小／最大尺寸擋掉單像素雜訊和整畫面過曝。每訊框最多追蹤 16 個亮點。
- **追蹤／分類。** 3 訊框遲滯追蹤器用曼哈頓像素距離比對連續訊框的亮點，再把訊框間位移分類成 `STATIC_LIGHT`、`VEHICLE` 或 `UNKNOWN`。
- **三角測距。** 2D 視差模型，用 120.8 mm 基線和 62° 水平視野（OV2640 @ SVGA）。可靠距離大約 3–50 m。

## 雜訊處理

戶外拍亮點，誤判多得嚇人。管線把幾層濾波器依序疊起來，亮點要全部通過才算數。

1. **亮度閾值**（`detector.cpp`）—— 只算 ≥ 200/255 的像素。
2. **尺寸過濾** —— 小於 16 px 當雜訊，大於 70000 px 當整畫面過曝，兩種都丟掉。
3. **8-相連 CCL + 合併** —— 質心相距 30 px 以內的合成一個，不然手機手電筒的兩顆 LED 會被算成兩個目標。
4. **感測器邊緣剔除** —— 落在最前或最後幾行（`cy < 3` 或 `cy > height-4`）的亮點直接丟掉，`vflip`/`hmirror` 會讓那裡的邊緣出現亮線。
5. **自身車燈反光過濾**（`tracker_classify`）—— 畫面底部四分之一出現大亮點，幾乎一定是自己的車燈打到路面。幾何位置很明確，所以不投票，直接歸成 `STATIC_LIGHT`。
6. **訊框間遲滯** —— 亮點要連續 **3 個訊框**（`TRACKER_CONFIRM_FRAMES`）都判成同一個類別，這個類別才採信。只閃一訊框的雜訊永遠過不了。
7. **跨相機 X/Y 比對 —— 主要防線**（`main.cpp` primary 迴圈）。primary 的亮點要符合以下條件才會被收下：
   - **正向 X 視差** —— 左側相機會看到這個亮點比右側*更偏右*（`dx ≥ STEREO_MIN_DISPARITY`），前方真有物體時，幾何上一定是這樣；而且
   - **2D 位置接近** —— 曼哈頓距離 `dx + |dy|` 小於約 200 px。

   只有單眼看到、或視差正負號不對的亮點，一律剔除。
8. **三角測距合理範圍**（`triangulation.cpp`）—— 算出來的距離要落在 0.5–80 m，視差要 ≥ 1 px。
9. **全黑重設** —— 畫面裡沒有亮點時，追蹤器清空狀態。已經離開畫面的燈光，殘留的質心不會留到後面誤配。

完整清單請看 [docs/noise-handling.zh-TW.md](docs/noise-handling.zh-TW.md)。

## 負面結果

**跨相機 X/Y 比對是整套最強的一層，但還是有假目標過得去。** 兩台相機真的同時看到同一個假亮點時（濕路面反光、路標、第二個光源），正視差*和*距離接近兩個條件都成立，於是被三角測距當成真目標。單一線索的濾波分不出「騎士的車燈」和「兩台相機都看得到的任何亮東西」。專案就是卡在這裡才擱置。

學到的事：

- **亮點沒有唯一性。** 濕路面反光、路標、停放中的車、其他燈光，讀出來的亮點都跟騎士的車燈一樣。
- **雜訊還是會鑽過門檻。** 尺寸／亮度閾值加上 3 訊框遲滯都上了，假亮點還是常常漏進來，聚光燈會打到根本不存在的東西上。
- **立體匹配很脆。** 兩台相機同時看到好幾個分不清的亮點時，視差很容易配到錯的那一對。

結論：在不受控的戶外場景，單一亮點不足以當作「迎面騎士」的特徵。要等到有更明確的線索（調變／紅外光源、運動加外形，或 ML 分類器），伺服聚光燈這一段才值得做。

完整說明在 [docs/why-it-stopped.zh-TW.md](docs/why-it-stopped.zh-TW.md)。

## 硬體

| 項目 | 詳細說明 |
|------|--------|
| 開發板 | 2× AI-Thinker ESP32-CAM (OV2640, "8225n v2.0" 模組) |
| 安裝方式 | 在麵包板上頂部對頂部安裝，鏡頭間距基線約 120.8 mm |
| 連線 | UART @ 115200 — secondary TX (GPIO1) → primary RX (GPIO13) |
| 備註 | Secondary 是倒著裝的：`vflip=1` + `hmirror=1` 都要設。GPIO12 是空接的 TX（HardwareSerial 兩個腳位都要給）。GPIO13 只有在 SD 卡沒有初始化的時候才空得出來。 |

## 建置與燒錄

`fix_camera_lib.py` 會在建置時修補 `esp32-camera` 函式庫（已經接進 `platformio.ini`），不用另外手動處理。

```bash
# Secondary (LEFT) first — no serial monitor, GPIO1 is the data link
pio run -e secondary -t upload

# Primary (RIGHT) — gives you the serial report
pio run -e primary -t upload
pio device monitor -b 115200
```

所有可調參數都在 `src/config.h`，常數表整理在 [docs/configuration.zh-TW.md](docs/configuration.zh-TW.md)。

## 檔案配置

```
CMakeLists.txt
LICENSE                  GPL-3.0
README.md                英文主檔
README.zh-TW.md          繁體中文
docs/                    從 README 搬出來的細節
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

## 限制與未完成的事

- **伺服馬達瞄準沒接。** 專案在整合致動器之前就停了，XY 伺服聚光燈只停在計畫階段。
- **主要防線會漏。** 兩台相機都看得到的假亮點會通過 X/Y 比對（見[負面結果](#負面結果)）。
- **視野角要校準。** 要準確的話，得校準 `STEREO_HFOV_DEG`。
- **基線是麵包板上的數字。** 兩塊板子在麵包板上頭對頭裝，鏡頭到鏡頭約 120.8 mm。
- **沒有實測數據。** 舊 README 沒有記下假亮點的誤判率，也沒有準確度數字。

## 授權條款

GPL-3.0 — 詳見 [LICENSE](LICENSE)。
