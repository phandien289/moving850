# 🚀 ESP32-S3 3-Wheel Omni Robot Controller & Trajectory Simulator

Hệ thống điều khiển Robot 3 bánh toàn hướng (Omnidirectional Robot) thời gian thực, độ trễ siêu thấp (<2ms) ứng dụng vi điều khiển **ESP32-S3 Dual-Core**, động cơ bước vi bước cao (8000 pulse/rev), truyền thông WebSockets và nạp code không dây qua OTA.

Dự án tích hợp nền tảng điều khiển & mô phỏng chuyên sâu:
1. **Web Controller Tích Hợp**: Nạp sẵn trong bộ nhớ Flash của ESP32-S3, truy cập trực tiếp qua WiFi SoftAP để giám sát và điều khiển.
2. **Omni-3 Robotics Lab (`index.html`)**: Trình mô phỏng vật lý 2D Canvas 60 FPS, phân tích động học, telemetry thời gian thực và hệ thống **Lập Trình Quỹ Đạo Tự Động (Trajectory Mission Planner)**.

---

## 📑 Mục Lục
- [1. Tính Năng Nổi Bật](#1-tính-năng-nổi-bật)
- [2. Kiến Trúc Hệ Thống & Dual-Core](#2-kiến-trúc-hệ-thống--dual-core)
- [3. Hệ Thống Lập Trình Quỹ Đạo Của Web (Mission Planner)](#3-hệ-thống-lập-trình-quỹ-đạo-của-web-mission-planner)
- [4. Sơ Đồ Đấu Nối Phần Cứng (Pinout)](#4-sơ-đồ-đấu-nối-phần-cứng-pinout)
- [5. Mô Hình Động Học Nghịch (Inverse Kinematics)](#5-mô-hình-động-học-nghịch-inverse-kinematics)
- [6. Giao Thức Truyền Thông WebSockets](#6-giao-thức-truyền-thông-websockets)
- [7. Cài Đặt & Nạp Firmware](#7-cài-đặt--nạp-firmware)
- [8. Hướng Dẫn Sử Dụng & Vận Hành](#8-hướng-dẫn-sử-dụng--vận-hành)
- [9. Cấu Trúc Thư Mục](#9-cấu-trúc-thư-mục)
- [10. Lưu Ý An Toàn Phần Cứng](#10-lưu-ý-an-toàn-phần-cứng)

---

## 1. Tính Năng Nổi Bật

- 🎯 **Lập Trình Quỹ Đạo Đa Chặng (Autonomous Trajectory Planner)**: Thiết lập chuỗi nhiệm vụ di chuyển chính xác theo mét, góc xoay theo độ và thời gian chờ dừng nghỉ trực tiếp trên giao diện web.
- ⚡ **Dual-Core Asymmetric Multiprocessing (AMP)**: Tách biệt hoàn toàn tác vụ mạng (Core 0) và vòng lặp phát xung thời gian thực (Core 1).
- 📶 **Độ Trễ Cực Thấp (< 2ms)**:
  - Vô hiệu hóa tiết kiệm điện Wi-Fi (`WiFi.setSleep(false)` & `esp_wifi_set_ps(WIFI_PS_NONE)`), công suất phát tối đa 19.5 dBm.
  - WebSocket Server (Port 81) truyền nhận gói tin tức thời, chống nghẽn hàng đợi (`bufferedAmount < 512`).
  - Giao diện Web tối ưu hóa bộ bắt sự kiện cảm ứng (`setPointerCapture`).
- 💎 **Độ Phân Giải Vi Bước Cao**: 8000 Pulse/Rev trên từng động cơ bước, vận hành êm ái, định vị chính xác.
- 🛡️ **Bảo Vệ Đa Tầng**:
  - **Watchdog an toàn (500ms)**: Tự động hãm phanh khẩn cấp nếu mất tín hiệu điều khiển hoặc rớt mạng.
  - **S-Curve Acceleration/Deceleration**: Tăng tốc và hãm quán tính mượt mà, triệt tiêu hiện tượng trượt bánh.
  - **Khóa trục / Thả lỏng tự do (ENA Control)**: Chuyển đổi trạng thái Trở kháng cao (Hi-Z) hoặc kích LOW cho tầng cách ly quang (Optocoupler) của Stepper Driver.
- 🔄 **Nạp Code Không Dây (Dual OTA)**:
  - **Web OTA (`/update`)**: Nạp file `.bin` trực tiếp qua trình duyệt với thanh tiến độ.
  - **PlatformIO OTA (`[env:ota]`)**: Nạp code từ máy tính qua WiFi (`espota`).

---

## 2. Kiến Trúc Hệ Thống & Dual-Core

ESP32-S3 sở hữu 2 nhân Xtensa® 32-bit LX7 xung nhịp 240MHz:

```
                  ┌──────────────────────────────────────────────┐
                  │                 ESP32-S3                     │
                  └──────────────────────┬───────────────────────┘
                                         │
                 ┌───────────────────────┴───────────────────────┐
                 ▼                                               ▼
  ┌─────────────────────────────┐                 ┌─────────────────────────────┐
  │           CORE 0            │                 │           CORE 1            │
  │     (Giao tiếp & Mạng)      │                 │  (Thời gian thực / Động cơ) │
  ├─────────────────────────────┤                 ├─────────────────────────────┤
  │ • WiFi SoftAP (Kênh 6)      │                 │ • Inverse Kinematics 3 Omni │
  │ • HTTP WebServer (Port 80)  │                 │ • S-Curve Ramp Generator    │
  │ • WebSockets (Port 81)      │    Biến đồng    │ • Real-Time Pulse Generator │
  │ • ArduinoOTA / Web OTA      │ ── bộ nguyên tử ─▶ (Độ rộng xung ~6µs)        │
  │ • Network Safety Watchdog   │   (volatile)    │ • Vòng lặp phát xung độc quyền│
  └─────────────────────────────┘                 └─────────────────────────────┘
                 ▲                                               │
                 │ WiFi (WS Port 81)                             ▼ 8000 pulse/rev
         [Trình duyệt Web]                          [3x Stepper Driver & Motor]
```

- **Core 0 (`networkTask`)**: Chạy ở Priority 3, đảm nhiệm toàn bộ kết nối Wi-Fi, HTTP Server và WebSocket Server.
- **Core 1 (`motorTask`)**: Chạy ở Priority 2, vô hiệu hóa Core 1 Watchdog (`disableCore1WDT()`) để tập trung 100% thời gian xử lý cho vòng lặp phát xung vi bước cực mịn mà không bị ngắt quãng.

---

## 3. Hệ Thống Lập Trình Quỹ Đạo Của Web (Mission Planner)

Giao diện **Omni-3 Robotics Lab** (`index.html`) tích hợp một công cụ lập trình và chạy quỹ đạo chuyển động tự động dành cho robot 3 bánh Omni:

### 3.1. Các Khối Lệnh Chuyển Động (Action Map)
Người dùng có thể xây dựng kịch bản nhiệm vụ bằng cách ghép nối linh hoạt các bước hành động:
- **Tịnh tiến tự do 8 hướng**:
  - `forward` (Tiến thẳng $+Y$): Đơn vị mét ($m$).
  - `backward` (Lùi thẳng $-Y$): Đơn vị mét ($m$).
  - `left` (Trượt ngang trái $-X$): Đơn vị mét ($m$) — *Tính năng đặc trưng của bánh Omni, di chuyển vuông góc mà không cần xoay hướng đầu robot*.
  - `right` (Trượt ngang phải $+X$): Đơn vị mét ($m$).
  - Chuyển động chéo $45^\circ$: `fwd_left`, `fwd_right`, `back_left`, `back_right` (vector vận tốc chuẩn hóa tỷ lệ $\frac{\sqrt{2}}{2} \approx 0.7071$).
- **Xoay tròn định lượng**:
  - `rot_ccw` (Xoay ngược chiều kim đồng hồ): Góc xoay tính bằng độ ($^\circ$).
  - `rot_cw` (Xoay theo chiều kim đồng hồ): Góc xoay tính bằng độ ($^\circ$).
- **Dừng nghỉ kỹ thuật**:
  - `wait`: Tạm dừng tại vị trí hiện tại trong $t$ giây trước khi thực hiện bước kế tiếp.

### 3.2. Thuật Toán Làm Mịn Vận Tốc S-Curve Hình Sin ($v_s = 0, v_e = 0$)
Để khắc phục triệt để hiện tượng bánh xe bị trượt quán tính (ví dụ: yêu cầu quay $90^\circ$ nhưng xe trượt đà tới $100^\circ$), hệ thống áp dụng hàm nội suy vận tốc hình sin trong hàm `getSmoothVelocityFactor(current, total)`:
- **Giai đoạn tăng tốc (25% đầu quãng đường)**: Vận tốc tăng dần theo hàm cosin từ mức khởi động $0.10$ lên $1.0$:
  $$\text{Ramp}_{\text{acc}} = \frac{1}{2}\left(1 - \cos\left(\pi \cdot \frac{p}{0.25}\right)\right)$$
- **Giai đoạn vận tốc ổn định (40% giữa)**: Duy trì vận tốc tối đa $V_{\text{max}}$.
- **Giai đoạn giảm tốc hãm phanh (35% cuối)**: Giảm mượt mà từ $1.0$ về tốc độ bò an toàn ($0.15$):
  $$\text{Ramp}_{\text{dec}} = \frac{1}{2}\left(1 + \cos\left(\pi \cdot \frac{p - 0.65}{0.35}\right)\right)$$
- **Kết quả**: Robot xuất phát êm ái, phanh hãm đúng điểm đích với độ chính xác cao và triệt tiêu biến dạng cơ khí.

### 3.3. Thời Gian Nghỉ Chuyển Bước (Step Delay / Settle Time)
- Cho phép người dùng tùy chỉnh thời gian nghỉ giữa 2 bước chuyển tiếp (mặc định $0.5$ giây).
- Giúp khung xe và động cơ triệt tiêu toàn bộ dao động quán tính trước khi nhận lệnh đổi hướng đột ngột (đặc biệt quan trọng khi đổi góc di chuyển $90^\circ$ hoặc đảo chiều $180^\circ$).

### 3.4. Quỹ Đạo Mẫu Dựng Sẵn (Quick Presets)
Hệ thống cung cấp sẵn các bài toán di chuyển điển hình:
1. **Tiến 0.5m & Trượt Trái 2m**: Khai thác khả năng trượt ngang không cần xoay trục của cơ cấu 3 Omni.
2. **Hình Vuông 1m ($\square$)**: Tiến $1m$ $\rightarrow$ Trượt phải $1m$ $\rightarrow$ Lùi $1m$ $\rightarrow$ Trượt trái $1m$.
3. **Tam Giác Đều 1m ($\triangle$)**: Kết hợp tiến thẳng và xoay thân $120^\circ$ tại mỗi đỉnh tam giác.
4. **Zíc-zắc 3 Chặng ($\text{Z}$)**: Kiểm tra khả năng lách chướng ngại vật ở góc chéo.

### 3.5. Chế Độ Lặp Chu Kỳ (Loop Mission)
- Tích hợp tùy chọn **🔁 Lặp chu kỳ (Loop)** cho phép robot tự động lặp lại toàn bộ lộ trình liên tục mà không cần can thiệp thủ công, phù hợp ứng dụng AGV tuần tra, vận chuyển hàng tự động trong nhà kho.
- Nút điều khiển: **CHẠY (Run)**, **TẠM DỪNG (Pause)** và **HỦY (Stop)** với thanh tiến độ trực quan hiển thị tỷ lệ % hoàn thành từng bước.

---

## 4. Sơ Đồ Đấu Nối Phần Cứng (Pinout)

### Kiểu đấu nối: **Nối Dương Chung (Common Anode)**
Tất cả các chân **PU+**, **DR+**, **MF+** (hoặc **ENA+**) của 3 Driver động cơ bước được đấu chụm vào nguồn **3.3V** của ESP32-S3.

| Thành Phần | Chức Năng Driver | Chân ESP32-S3 | Ghi Chú |
| :--- | :--- | :--- | :--- |
| **Động Cơ 1**<br>*(Bánh Sau / 90°)* | **PU-** (Pulse / Xung)<br>**DR-** (Direction / Chiều)<br>**MF-** (Enable / Khóa) | **GPIO 4**<br>**GPIO 5**<br>**GPIO 6** | Điều khiển chuyển động tiến-lùi & xoay |
| **Động Cơ 2**<br>*(Bánh Trái / 210°)* | **PU-** (Pulse / Xung)<br>**DR-** (Direction / Chiều)<br>**MF-** (Enable / Khóa) | **GPIO 7**<br>**GPIO 15**<br>**GPIO 16** | Kết hợp điều hướng tam giác 120° |
| **Động Cơ 3**<br>*(Bánh Phải / 330°)* | **PU-** (Pulse / Xung)<br>**DR-** (Direction / Chiều)<br>**MF-** (Enable / Khóa) | **GPIO 17**<br>**GPIO 18**<br>**GPIO 8** | Kết hợp điều hướng tam giác 120° |
| **Nguồn Chung** | **PU+**, **DR+**, **MF+** | **3V3** | Mức logic 3.3V bảo vệ mạch cách ly quang |

> [!NOTE]
> Các chân GPIO được bố trí an toàn: Tránh hoàn toàn dải Flash SPI (GPIO 26–32), Octal PSRAM (GPIO 33–37) và các chân Native USB (GPIO 19, 20).

---

## 5. Mô Hình Động Học Nghịch (Inverse Kinematics)

Ba bánh xe Omni được đặt cách đều nhau $120^\circ$ quanh tâm robot:
- Bánh 1: $90^\circ$ (Trục ngang sau)
- Bánh 2: $210^\circ$ (Trái trước)
- Bánh 3: $330^\circ$ (Phải trước)

Mối quan hệ giữa vận tốc thân xe $(V_x, V_y, \omega)$ và vận tốc 3 bánh $(V_1, V_2, V_3)$:

$$\begin{bmatrix} V_1 \\ V_2 \\ V_3 \end{bmatrix} = \begin{bmatrix} -1 & 0 & L \\ \frac{1}{2} & -\frac{\sqrt{3}}{2} & L \\ \frac{1}{2} & \frac{\sqrt{3}}{2} & L \end{bmatrix} \begin{bmatrix} V_x \\ V_y \\ \omega \end{bmatrix}$$

Hệ số bù lệch lái (*Steering Trim*) được tính vào thành phần vận tốc góc $\omega$ để đảm bảo xe chạy thẳng tuyệt đối:
$$\omega_{\text{thực tế}} = -(\omega_{\text{nhập}} + \text{trim} \cdot V_y)$$

---

## 6. Giao Thức Truyền Thông WebSockets

Client kết nối trực tiếp đến WebSocket Server tại cổng 81: `ws://192.168.4.1:81/`

| Lệnh (Web Client ➔ Robot) | Ý Nghĩa | Định Dạng Ví Dụ |
| :--- | :--- | :--- |
| `M:vx,vy,w` | Vận tốc di chuyển $[-1.0, 1.0]$ | `M:0.50,-0.80,0.10` |
| `S:rpm` | Tốc độ định mức RPM $[10, 350]$ | `S:70` |
| `D:decel` | Tốc độ hãm phanh $[0.005, 0.05]$ | `D:0.035` |
| `T:trim` | Hệ số bù lệch lái $[-0.2, 0.2]$ | `T:0.01` |
| `E:1` hoặc `E:0` | Bật (Khóa trục) / Tắt (Thả lỏng trục) | `E:1` |
| `I:1` hoặc `I:0` | Đảo cực tính logic chân EN | `I:0` |

| Phản hồi (Robot ➔ Web Client) | Ý Nghĩa |
| :--- | :--- |
| `STATE:E=1,INV=0` | Trạng thái kích hoạt động cơ và cực tính chân EN |

---

## 7. Cài Đặt & Nạp Firmware

### Yêu Cầu
- [PlatformIO IDE](https://platformio.org/) trên VS Code hoặc Antigravity IDE.

### 1. Nạp qua cáp USB Type-C (Lần đầu)
```bash
# Biên dịch và nạp firmware qua cổng Serial/CDC
pio run -e 4d_systems_esp32s3_gen4_r8n16 -t upload

# Mở Serial Monitor quan sát log (115200 baud)
pio device monitor -b 115200
```

### 2. Nạp không dây qua OTA (Over-The-Air)
**Cách 1: Nạp qua PlatformIO CLI**
1. Kết nối Wi-Fi vào mạng `ESP32-OMNI-ROBOT` (Mật khẩu: `12345678`).
2. Chạy lệnh:
```bash
pio run -e ota -t upload
```

**Cách 2: Nạp qua trình duyệt Web OTA**
1. Biên dịch file `.bin`:
```bash
pio run -e 4d_systems_esp32s3_gen4_r8n16
# File firmware sau khi build: .pio/build/4d_systems_esp32s3_gen4_r8n16/firmware.bin
```
2. Truy cập `http://192.168.4.1/update` trên trình duyệt.
3. Kéo thả file `firmware.bin` và bấm **Bắt đầu nạp firmware**. Robot sẽ tự động nạp và khởi động lại sau 3 giây.

---

## 8. Hướng Dẫn Sử Dụng & Vận Hành

### Bước 1: Kết nối Wi-Fi
- **SSID**: `ESP32-OMNI-ROBOT`
- **Mật khẩu**: `12345678`
- **IP Mặc định**: `192.168.4.1`

### Bước 2: Vận hành chế độ lập trình quỹ đạo (`index.html`)
1. Mở file [index.html](file:///d:/Antigravity%20IDE/saves/moving850/index.html) trên trình duyệt.
2. Bấm nút **ESP32 ⚙️** trên góc phải để nhập địa chỉ WebSocket `ws://192.168.4.1:81/` và bấm **Kết Nối** (hoặc để mặc định để chạy thử nghiệm độc lập trên Canvas Simulator).
3. Chọn thẻ **🎯 QUỸ ĐẠO (Mission)**:
   - Thêm từng bước hoặc chọn một mẫu quỹ đạo có sẵn (Hình vuông, tam giác, zíc-zắc...).
   - Bật tùy chọn **✨ Chuyển động êm (v_s=0, v_e=0)** để kích hoạt hồ sơ vận tốc S-Curve hình sin.
   - Bấm nút **CHẠY**: Quan sát robot thật di chuyển đồng bộ với robot ảo trên màn hình Canvas 2D.
4. Bấm **TẠM DỪNG** hoặc **HỦY** bất kỳ lúc nào để ngắt chu trình khẩn cấp.

---

## 9. Cấu Trúc Thư Mục

```
moving850/
├── .agents/                 # Cấu hình Agentic AI
├── .vscode/                 # Cấu hình IDE VSCode / PlatformIO
├── include/                 # Header files nội bộ
├── src/
│   └── main.cpp             # Mã nguồn ESP32-S3 (Dual-Core, Web Server, Motor Task)
├── index.html               # Omni-3 Robotics Lab (Trình mô phỏng, Telemetry & Lập trình quỹ đạo)
├── platformio.ini           # Cấu hình môi trường PlatformIO (USB CDC & OTA)
└── README.md                # Tài liệu hướng dẫn dự án
```

---

## 10. Lưu Ý An Toàn Phần Cứng

> [!WARNING]
> - **Nguồn cấp Driver**: Cấp nguồn riêng $12\text{V} - 24\text{V}$ đủ tải dòng điện cho Driver động cơ bước. Không dùng chân 5V/3.3V của ESP32 để cấp nguồn cuộn kháng động cơ.
> - **Tản nhiệt**: Cần gắn tản nhiệt hoặc quạt cho Driver động cơ bước khi vận hành ở tốc độ cao hoặc dòng tải lớn.
> - **Thả lỏng trục cơ khí**: Khi bảo trì hoặc canh chỉnh khung xe, kích hoạt chế độ **ENA: THẢ TỰ DO** trên giao diện để bảo vệ Driver và tránh sinh nhiệt thừa.
