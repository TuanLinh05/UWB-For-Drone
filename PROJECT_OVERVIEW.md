# TỔNG QUAN DỰ ÁN: UWB FOR DRONE
**Hệ thống Định vị Không gian Thời gian thực Độ chính xác cao cho Thiết bị Bay Không người lái (UAV/Drone)**

---

## 1. Giới Thiệu Chung (Project Introduction)

**UWB For Drone** là một giải pháp định vị trong nhà (Indoor Positioning System - IPS) và môi trường GPS-denied (khu vực mất hoặc nhiễu sóng vệ tinh) toàn diện, ứng dụng công nghệ sóng vô tuyến siêu băng rộng (**Ultra-Wideband - UWB**). 

Hệ thống cho phép xác định tọa độ 2D/3D của Drone (hoặc robot di động) trong không gian thời gian thực với sai số mức **centimet (10 - 20 cm)**, tần số cập nhật cao (**30Hz - 50Hz**), và độ trễ cực thấp.

### Điểm nổi bật về mặt công nghệ:
1. **Phần cứng chuyên dụng kép**: Kết hợp vi điều khiển STM32F103 xử lý thời gian thực định thời micro-giây cho RF UWB Decawave DW1000, và ESP32-S3 đảm nhiệm cầu nối truyền thông không dây băng thông rộng.
2. **Thuật toán đo khoảng cách DS-TWR**: Đo khoảng cách hai chiều đối xứng (Double-Sided Two-Way Ranging) với 4 trạm Anchor, loại trừ triệt để sai số trôi xung nhịp (clock drift) mà không cần dây đồng bộ phần cứng.
3. **Bộ lọc thích nghi chuyển động (Motion-Adaptive Filtering)**: Tự động phân biệt trạng thái Drone đang bay (Dynamic) hay đứng yên/hovering (Static) để điều chỉnh hệ số lọc, loại bỏ nhiễu phản xạ đa đường (multipath) và hiện tượng nhảy khoảng cách (outliers).
4. **Giao thức truyền thông nhị phân (Binary Telemetry Protocol)**: Tối ưu băng thông truyền UART/WiFi, có mã kiểm tra toàn vẹn CRC16-CCITT cho từng chu kỳ đo.
5. **Trạm giám sát mặt đất trực quan (Web-based GCS)**: Giao diện web thời gian thực trên nền React + Vite + TypeScript, tích hợp thuật toán giải vị trí Trilateration và bộ lọc Kalman 2D.

---

## 2. Kiến Trúc Toàn Hệ Thống (System Architecture)

Hệ thống hoạt động theo mô hình phân tầng từ phần cứng vô tuyến đến trạm mặt đất:

```
[Anchor 1] (STM32 + DW1000) \
[Anchor 2] (STM32 + DW1000)  -- (Sóng RF UWB ~3.5-6.5 GHz / DS-TWR) --> [Drone TAG] (STM32 + DW1000)
[Anchor 3] (STM32 + DW1000) /                                                  |
[Anchor 4] (STM32 + DW1000) /                                                  | (UART Binary Stream 115200)
                                                                               v
                                                                   [ESP32-S3 Wireless Bridge]
                                                                               |
                                                                               | (Wi-Fi 802.11n / WebSocket :81)
                                                                               v
                                                                   [Ground Control Web GUI]
                                                                    (Trilateration + 2D Kalman)
```

---

## 3. Các Phân Hệ Chi Tiết (Component Breakdown)

### 3.1. Phân hệ Định vị UWB STM32 (`STM32_UWB/`)
Chạy trên vi điều khiển STM32F103C8T6 (ARM Cortex-M3 @ 72MHz) giao tiếp với chip thu phát Decawave DW1000 qua giao thức SPI tốc độ cao (18MHz/36MHz).

- **Trạm mốc cố định (Anchors - `Anchor/`, `Anchor_2/`, `Anchor_3/`, `Anchor_4/`)**:
  - Đặt tại 4 góc của khu vực bay với tọa độ không gian đã được xác định trước.
  - Hoạt động ở chế độ chờ phản hồi (Responder/Listener), lắng nghe gói tin từ TAG và phản hồi chính xác theo mốc thời gian phần cứng DW1000.
- **Trạm di động trên Drone (TAG - `TAG/`)**:
  - Là Initiator điều phối toàn bộ chu kỳ đo (Ranging Task).
  - Khởi tạo chu kỳ đo DS-TWR tuần tự qua 4 Anchor ở tốc độ lên tới 50Hz.
  - Tích hợp pipeline lọc khoảng cách thích nghi (`motion_adaptive_range.h`, `legacy_adaptive_tracking.h`, `range_filter.h`):
    - Tự động bù sai số trễ ăng-ten (Antenna Delay).
    - Bù phi tuyến tính dựa trên công suất xung đầu (First Path Power - FPP).
    - Lọc trung vị và lọc chuyển động thích ứng để chống nhiễu môi trường kín.
  - Đóng gói dữ liệu dạng nhị phân (`telemetry.c`) và truyền sang ESP32-S3 qua UART non-blocking ring buffer.

### 3.2. Cầu Nối Không Dây ESP32-S3 (`ESP32S3_TAG/`)
Chạy trên vi điều khiển ESP32-S3 Dual-Core Xtensa LX7:

- **Bộ phân tích UART (`uart_protocol.cpp`)**:
  - Máy trạng thái hữu hạn (FSM) xử lý từng byte dữ liệu từ STM32 TAG, buffer mở rộng 2048 bytes chống nghẽn khi mạng Wi-Fi tải cao.
  - Định dạng gói tin nhị phân chuẩn:
    ```
    [SOF: 0xAA 0x55] [VER: 1B] [TYPE: 1B] [LEN: 2B] [SEQ: 4B] [TIME_MS: 4B] [PAYLOAD: N bytes] [CRC16: 2B]
    ```
  - Kiểm tra tính hợp lệ bằng thuật toán mã kiểm tra CCITT-FALSE (Polynomial `0x1021`, Khởi tạo `0xFFFF`).
- **Cầu nối mạng WebSocket (`ws_bridge.cpp`)**:
  - Chuyển đổi gói tin nhị phân thành định dạng JSON nhẹ.
  - Phát broadcast qua WebSocket Server (Port 81) tới các client (trình duyệt Web GUI, hệ thống điều khiển bay Mission Planner / ROS).
- **Quản lý Wi-Fi không chặn (`wifi_manager.cpp`)**:
  - Duy trì kết nối Wi-Fi nền, tự động kết nối lại khi mất sóng mà không block luồng xử lý UART.

### 3.3. Trạm Giám Sát Mặt Đất Web GUI (`GUI test/`)
Xây dựng trên nền tảng hiện đại **React 18 + Vite + TypeScript + Recharts + Lucide Icons**:

- **Tính toán tọa độ thời gian thực (Trilateration Engine - `trilateration.ts`)**:
  - Giải bài toán hình học định vị không gian từ 4 bán kính khoảng cách tới 4 Anchor.
  - Tích hợp bộ giải ma trận Levenberg-Marquardt / Least Squares giảm thiểu sai số.
- **Bộ lọc Kalman 2D (`kalman2d.ts`)**:
  - Dự đoán và làm mịn quỹ đạo bay của Drone, bù đắp độ trễ và triệt tiêu rung lắc ảo.
- **Giao diện trực quan hóa**:
  - Bản đồ không gian 2D/3D hiển thị vị trí các Anchor và vệt bay (flight trail) của Drone.
  - Bảng chỉ số vi sai RF: Tần số đo thực tế (Hz), tỷ lệ mất gói (Packet Drop Rate), chất lượng tín hiệu FPP (dBm), cảnh báo CRC Error.
  - Trình phân tích độ trôi tĩnh và phân tích công suất tín hiệu (`fppBiasAnalysis.ts`, `staticCalibrationAnalysis.ts`).
- **Hệ thống Replay Harness (`replayLog.ts`, `replayRunner.ts`)**:
  - Ghi nhận lại toàn bộ dữ liệu thô trong suốt chuyến bay để phát lại mô phỏng offline phục vụ nghiên cứu và kiểm thử thuật toán.

### 3.4. Thư Viện Decawave Driver (`DW1000_decadriver/`)
- Cung cấp toàn bộ các thanh ghi cấp thấp, driver SPI bare-metal và FreeRTOS.
- Chức năng đọc CIR (Channel Impulse Response) để phân tích chất lượng kênh truyền RF và phát hiện tình trạng khuất tầm nhìn (NLOS - Non-Line-of-Sight).

### 3.5. Bộ Công Cụ Hiệu Chuẩn Python (`tools/`)
- `analyze_fpp_bias.py`: Phân tích quan hệ giữa First Path Power và độ lệch khoảng cách nhằm xây dựng đường cong bù sai số quang học cho DW1000.
- `calibration_from_log.py` & `calibration_from_wizard_csv.py`: Tự động tính toán giá trị trễ ăng-ten (Antenna Delay Tuning) từ dữ liệu thực nghiệm.

---

## 4. Đặc Tả Kỹ Thuật (System Specifications)

| Tiêu chuẩn | Thông số kỹ thuật |
| :--- | :--- |
| **Băng tần RF** | UWB Channel 2 / 5 (3.99 GHz / 6.48 GHz) |
| **Chip thu phát** | Decawave DW1000 |
| **Tốc độ truyền dữ liệu RF** | 6.81 Mbps (hoặc 110 kbps/850 kbps cấu hình) |
| **Giao thức đo khoảng cách** | DS-TWR (Double-Sided Two-Way Ranging) |
| **Số lượng Anchor hỗ trợ** | 4 Anchors (mở rộng lên tới 8 Anchors) |
| **Tần số cập nhật vị trí** | 30 Hz - 50 Hz |
| **Độ chính xác đo khoảng cách** | ± 5 cm - 10 cm (sau hiệu chuẩn) |
| **Độ chính xác vị trí không gian** | ± 10 cm - 20 cm |
| **Giao tiếp STM32 -> ESP32** | UART 115200 bps (Binary Packet + CRC16) |
| **Giao tiếp ESP32 -> Web GUI** | Wi-Fi 2.4GHz / WebSocket Port 81 (JSON Broadcast) |
| **Frontend Ground Station** | React 18, Vite, TypeScript, Canvas 2D/3D |

---

## 5. Cấu Trúc Mã Nguồn (Repository Layout)

```
UWB For Drone/
├── DW1000_decadriver/      # Thư viện Decawave DW1000 baremetal & FreeRTOS
│   ├── DW1000_baremetal_lib/
│   └── DW1000_freeRTOS_Lib/
├── ESP32S3_TAG/            # Firmware Cầu nối Wi-Fi/WebSocket ESP32-S3
│   └── TAG/
│       ├── TAG.ino         # File chính: Cấu hình Wi-Fi & vòng lặp truyền thông
│       ├── uart_protocol.* # Bộ giải mã FSM Binary Packet & CRC16
│       ├── wifi_manager.*  # Bộ kết nối Wi-Fi tự phục hồi non-blocking
│       └── ws_bridge.*     # WebSocket server phát JSON
├── GUI test/               # Trạm mặt đất giám sát thời gian thực
│   ├── src/
│   │   ├── components/     # UI Components hiển thị biểu đồ & tọa độ
│   │   ├── hooks/          # React hooks quản lý kết nối WebSocket
│   │   └── lib/            # Thuật toán Trilateration, Kalman 2D, Replay Harness
│   ├── package.json        # Danh mục thư viện Node.js
│   └── vite.config.ts      # Cấu hình Vite bundler
├── STM32_UWB/              # Firmware C STM32CubeIDE
│   ├── Anchor/             # Mã nguồn Anchor 1
│   ├── Anchor_2/           # Mã nguồn Anchor 2
│   ├── Anchor_3/           # Mã nguồn Anchor 3
│   ├── Anchor_4/           # Mã nguồn Anchor 4
│   ├── TAG/                # Mã nguồn TAG (DS-TWR Master, Adaptive Filtering)
│   │   └── Core/Src/
│   │       ├── tag_ranging.c           # Quản lý chu kỳ đo DS-TWR
│   │       ├── motion_adaptive_range.c # Bộ lọc thích nghi chuyển động
│   │       └── telemetry.c             # Gửi gói nhị phân qua UART
│   └── HostTests/          # Unit tests thuật toán chạy trên PC
├── tools/                  # Script Python hiệu chuẩn ăng-ten và phân tích FPP
├── start_gui.bat           # Phím tắt chạy nhanh Web GUI cho Windows
├── .gitignore              # Bộ lọc các tệp rác & file build tạm
├── README.md               # Hướng dẫn nhanh cho GitHub
└── PROJECT_OVERVIEW.md     # Tài liệu kiến trúc chuyên sâu toàn diện
```

---

## 6. Hướng Dẫn Vận Hành Hệ Thống (Step-by-Step Deployment)

1. **Hiệu chuẩn phần cứng (Calibration)**:
   - Đo khoảng cách cố định thực tế giữa TAG và từng Anchor (ví dụ 1.00m, 2.00m).
   - Chạy script trong `tools/calibration_from_wizard_csv.py` để lấy giá trị trễ ăng-ten (`ANTENNA_DELAY`).
   - Cập nhật giá trị vào `STM32_UWB/TAG/Core/Inc/uwb_calibration.h`.
2. **Nạp Firmware STM32**:
   - Dùng STM32CubeIDE nạp lần lượt các project `Anchor`, `Anchor_2`, `Anchor_3`, `Anchor_4` cho 4 mạch Anchor.
   - Nạp project `TAG` cho mạch gắn trên Drone.
3. **Cấu hình & Nạp ESP32-S3**:
   - Mở `ESP32S3_TAG/TAG/TAG.ino` điền SSID và Mật khẩu Wi-Fi của trạm mặt đất.
   - Nạp vào ESP32-S3 bằng Arduino IDE hoặc PlatformIO.
4. **Khởi chạy Web GUI**:
   - Nhấp đúp file `start_gui.bat` (hoặc chạy `npm run dev` trong thư mục `GUI test`).
   - Mở trình duyệt, nhập địa chỉ IP của ESP32-S3 để kết nối WebSocket.
   - Thiết lập tọa độ 4 Anchor trên giao diện Web; hệ thống sẽ ngay lập tức vẽ tọa độ thời gian thực của Drone.