#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_wifi.h>
#include <ArduinoOTA.h>
#include <Update.h>

/**
 * ==============================================================================
 * DỰ ÁN ROBOT 3 BÁNH OMNI - GIAO DIỆN TAY CẦM PS4 DUAL-CORE KHÔNG TRỄ (ESP32-S3)
 * ==============================================================================
 * - MCU: ESP32-S3 DevKit (Dual-Core Xtensa LX7 @ 240MHz)
 * - Vi bước: 8000 Pulse/Rev
 * - Kiểu đấu: Nối dương chung (PU+, DR+, MF+ nối 3V3 của ESP32-S3)
 * 
 * PHÂN BỐ CHÂN 3 ĐỘNG CƠ:
 *   + ĐỘNG CƠ 1 (Bánh sau / 90°):   PU- = GPIO 4,  DR- = GPIO 5,  MF- = GPIO 6
 *   + ĐỘNG CƠ 2 (Bánh trái / 210°): PU- = GPIO 7,  DR- = GPIO 15, MF- = GPIO 16
 *   + ĐỘNG CƠ 3 (Bánh phải / 330°): PU- = GPIO 17, DR- = GPIO 18, MF- = GPIO 8
 * 
 * GIẢI PHÁP TRIỆT TIÊU HOÀN TOÀN ĐỘ TRỄ & HIỆN TƯỢNG CHẠY LIÊN TỤC:
 *   1. PHÂN CHIA 2 CORE ĐỘC LẬP:
 *      - Core 0: Chuyên trách WiFi + WebSocket (Port 81) + WebServer. Không bao giờ bị nghẽn!
 *      - Core 1: Chuyên trách phát xung động cơ thời gian thực, không bị ngắt quãng.
 *   2. TẮT TIẾT KIỆM PIN WIFI (WiFi.setSleep(false)): Giảm ping từ 200ms xuống 1ms.
 *   3. CHỐNG DỒN HÀNG ĐỢI (BUFFEREDAMOUNT): Bỏ qua các tọa độ cũ khi mạng bận, chỉ gửi vị trí mới nhất.
 *   4. POINTER EVENTS + SETPOINTERCAPTURE: Chuẩn cảm ứng bắt dính ngón tay, buông ra là phanh ngay lập tức.
 *   5. WATCHDOG AN TOÀN 180MS: Mất kết nối hoặc thả tay là dừng phanh êm ái.
 * ==============================================================================
 */

// Cấu hình Wi-Fi Access Point
const char* AP_SSID = "ESP32-OMNI-ROBOT";
const char* AP_PASS = "12345678";

WebServer server(80);
WebSocketsServer webSocket(81);

// Cấu trúc chân động cơ
struct StepperMotor {
  uint8_t pinPulse; // PU-
  uint8_t pinDir;   // DR-
  uint8_t pinEna;   // MF-
};

StepperMotor motors[3] = {
  {4,  5,  6 }, // Motor 1 (Bánh 1 - 90°)
  {7,  15, 16}, // Motor 2 (Bánh 2 - 210°)
  {17, 18, 8 }  // Motor 3 (Bánh 3 - 330°)
};

const unsigned long PULSES_PER_REV = 8000;

// Biến mục tiêu vận tốc từ Joystick (-1.0 đến 1.0)
volatile float targetVx = 0.0f; // Trục X (Trái / Phải)
volatile float targetVy = 0.0f; // Trục Y (Tiến / Lùi)
volatile float targetW  = 0.0f; // Xoay tròn (CW / CCW)
volatile int   targetRpm = 70;  // Tốc độ tối đa (30 - 150 RPM)
volatile bool  motorsEnabled = true;
volatile bool  invertEnPolarity = false; // false: Dương chung (HIGH = Khóa, LOW = Thả) | true: Âm chung / EN+ cắm GPIO (LOW = Khóa, HIGH = Thả)
volatile unsigned long lastCommandTime = 0;

void applyEnPinState() {
  for (int i = 0; i < 3; i++) {
    bool shouldDisable = (!invertEnPolarity) ? !motorsEnabled : motorsEnabled;
    if (shouldDisable) {
      // 🔓 MỞ TRỤC (THẢ TỰ DO):
      // Kéo chân xuống LOW (0V) -> Dòng 5V chạy qua Opto về GND -> Opto SÁNG -> Thả lỏng trục
      pinMode(motors[i].pinEna, OUTPUT);
      digitalWrite(motors[i].pinEna, LOW);
    } else {
      // 🔒 KHÓA TRỤC (XE CHẠY):
      // Đặt chân về INPUT (Trở kháng cao Hi-Z) -> Ngắt mạch hoàn toàn (I = 0) -> Opto TẮT -> Xe chạy!
      pinMode(motors[i].pinEna, INPUT);
    }
  }
}

// WATCHDOG AN TOÀN: 500ms không nhận gói tin mới -> Mới kích hoạt dừng phanh (chống giật khựng do trễ mạng WiFi)
const unsigned long SAFETY_TIMEOUT_MS = 500;

// Tham số làm mịn gia tốc & hãm phanh
volatile float accelRate = 0.040f;    // Tăng tốc lanh lợi (~120ms)
volatile float decelRate = 0.035f;    // Hãm phanh dứt khoát êm ái (~140ms)
volatile float steeringTrim = 0.0f;   // Bù lệch bánh khi chạy thẳng

// Giao diện Web PS4 Gamepad (HTML + CSS + JavaScript WebSocket)
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>PS4 WebSocket Omni Controller</title>
  <style>
    :root {
      --ps-blue: #0070d1;
      --ps-cyan: #00f0ff;
      --ps-dark: #0a0c13;
      --ps-card: #151824;
      --ps-border: #232738;
      --ps-thumb: #222634;
      --ps-thumb-active: #2e3549;
    }
    * {
      box-sizing: border-box;
      user-select: none;
      -webkit-user-select: none;
      touch-action: none;
      margin: 0;
      padding: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    }
    body {
      background: radial-gradient(circle at center, #171b2b, var(--ps-dark));
      color: #fff;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: space-between;
      padding: 12px;
      overflow: hidden;
    }

    /* PS4 LIGHTBAR & HEADER */
    .ps4-topbar {
      width: 100%;
      max-width: 500px;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 8px;
    }
    .lightbar {
      width: 180px;
      height: 6px;
      background: var(--ps-cyan);
      border-radius: 4px;
      box-shadow: 0 0 16px var(--ps-cyan), 0 0 30px var(--ps-blue);
      transition: background 0.3s;
    }
    .status-panel {
      width: 100%;
      display: flex;
      justify-content: space-between;
      align-items: center;
      background: var(--ps-card);
      padding: 8px 16px;
      border-radius: 14px;
      border: 1px solid var(--ps-border);
      font-size: 0.8rem;
      font-weight: 600;
    }
    .speed-tag {
      color: var(--ps-cyan);
    }

    /* KHUNG TAY CẦM PS4 */
    .gamepad-container {
      width: 100%;
      max-width: 500px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 10px 0;
    }

    /* ANALOG JOYSTICK */
    .joystick-zone {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 10px;
    }
    .joystick-label {
      font-size: 0.75rem;
      font-weight: 700;
      letter-spacing: 1px;
      color: #8f9bb3;
    }
    .joystick-base {
      width: 150px;
      height: 150px;
      background: radial-gradient(circle, #1c2030, #10121a);
      border-radius: 50%;
      border: 3px solid #282e42;
      box-shadow: inset 0 6px 14px rgba(0,0,0,0.8), 0 4px 14px rgba(0,0,0,0.4);
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      touch-action: none;
    }
    .joystick-base::after {
      content: '';
      position: absolute;
      width: 50px;
      height: 50px;
      border-radius: 50%;
      border: 1px dashed rgba(255,255,255,0.1);
    }
    .joystick-thumb {
      width: 68px;
      height: 68px;
      background: radial-gradient(circle at 35% 35%, var(--ps-thumb-active), var(--ps-thumb));
      border-radius: 50%;
      border: 2px solid #3c445c;
      box-shadow: 0 8px 20px rgba(0,0,0,0.7), inset 0 2px 4px rgba(255,255,255,0.2);
      position: absolute;
      cursor: grab;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: transform 0.05s ease-out;
    }
    .joystick-thumb::before {
      content: '';
      width: 40px;
      height: 40px;
      border-radius: 50%;
      border: 2px solid rgba(0, 240, 255, 0.35);
      background: rgba(0,0,0,0.25);
    }

    /* CENTER CONSOLE: TOUCHPAD & NÚT DỪNG */
    .center-console {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 12px;
    }
    .touchpad-btn {
      width: 110px;
      height: 70px;
      background: #181b28;
      border: 2px solid var(--ps-border);
      border-radius: 12px;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      cursor: pointer;
      box-shadow: 0 4px 12px rgba(0,0,0,0.5);
      transition: 0.1s;
    }
    .touchpad-btn:active {
      transform: scale(0.94);
      border-color: #ef4444;
      background: rgba(239, 68, 68, 0.2);
    }
    .touchpad-btn span {
      font-size: 0.7rem;
      font-weight: 700;
      color: #ef4444;
      margin-top: 4px;
    }

    /* BẢNG CÀI ĐẶT PHÍA DƯỚI */
    .settings-drawer {
      width: 100%;
      max-width: 500px;
      background: var(--ps-card);
      border: 1px solid var(--ps-border);
      border-radius: 16px;
      padding: 12px 16px;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .slider-group {
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .slider-title {
      display: flex;
      justify-content: space-between;
      font-size: 0.75rem;
      font-weight: 600;
      color: #94a3b8;
    }
    input[type=range] {
      width: 100%;
      height: 6px;
      border-radius: 3px;
      background: #232738;
      outline: none;
      -webkit-appearance: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 18px;
      height: 18px;
      border-radius: 50%;
      background: var(--ps-cyan);
      box-shadow: 0 0 8px var(--ps-cyan);
      cursor: pointer;
    }
    .btn-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 4px;
    }
    .ps-btn {
      padding: 8px;
      font-size: 0.75rem;
      font-weight: 700;
      border-radius: 10px;
      border: 1px solid var(--ps-border);
      background: rgba(255,255,255,0.05);
      color: #fff;
      cursor: pointer;
    }
    .ps-btn:active {
      background: var(--ps-blue);
    }
    /* ENABLE (EN) DEDICATED PANEL */
    .en-control-card {
      background: rgba(255, 255, 255, 0.03);
      border: 1px solid var(--ps-border);
      border-radius: 14px;
      padding: 10px 12px;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .en-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.75rem;
      font-weight: 700;
      color: #94a3b8;
    }
    .en-status-pill {
      font-size: 0.7rem;
      padding: 3px 8px;
      border-radius: 6px;
      font-weight: 700;
      letter-spacing: 0.5px;
    }
    .en-active {
      background: rgba(16, 185, 129, 0.15);
      color: #10b981;
      border: 1px solid rgba(16, 185, 129, 0.4);
    }
    .en-inactive {
      background: rgba(234, 179, 8, 0.15);
      color: #eab308;
      border: 1px solid rgba(234, 179, 8, 0.4);
    }
    .ps-btn-en {
      padding: 10px 6px;
      font-size: 0.78rem;
      font-weight: 700;
      border-radius: 10px;
      border: 1px solid var(--ps-border);
      background: rgba(255,255,255,0.05);
      color: #fff;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: all 0.2s;
    }
    .ps-btn-en.active-lock {
      background: rgba(16, 185, 129, 0.22);
      border-color: #10b981;
      color: #10b981;
      box-shadow: 0 0 12px rgba(16, 185, 129, 0.35);
    }
    .ps-btn-en.active-free {
      background: rgba(234, 179, 8, 0.22);
      border-color: #eab308;
      color: #eab308;
      box-shadow: 0 0 12px rgba(234, 179, 8, 0.35);
    }
    .en-invert-row {
      display: flex;
      align-items: center;
      gap: 8px;
      padding-top: 6px;
      border-top: 1px dashed rgba(255, 255, 255, 0.08);
      font-size: 0.72rem;
      color: #8f9bb3;
      cursor: pointer;
    }
    .en-invert-row input[type=checkbox] {
      cursor: pointer;
      width: 15px;
      height: 15px;
      accent-color: var(--ps-cyan);
    }
  </style>
</head>
<body>

  <div class="ps4-topbar">
    <div class="lightbar" id="wsLightbar"></div>
    <div class="status-panel">
      <div>🎮 <span id="robotMode">ĐANG KẾT NỐI...</span></div>
      <div>EN: <span id="enBadge" style="color: #10b981; font-weight: 700;">🔒 KHÓA</span></div>
      <div>TỐC ĐỘ: <span id="speedVal" class="speed-tag">70 RPM</span></div>
      <a href="/update" style="color: var(--ps-cyan); text-decoration: none; font-size: 0.75rem; border: 1px solid var(--ps-border); padding: 3px 8px; border-radius: 8px; font-weight: 700;">📡 NẠP OTA</a>
    </div>
  </div>

  <!-- BỘ ĐÔI ANALOG JOYSTICK TAY CẦM PS4 -->
  <div class="gamepad-container">
    <!-- LEFT STICK: DI CHUYỂN 360° -->
    <div class="joystick-zone">
      <div class="joystick-base" id="leftBase">
        <div class="joystick-thumb" id="leftThumb"></div>
      </div>
      <div class="joystick-label">🕹️ DI CHUYỂN 360° (L3)</div>
    </div>

    <!-- TOUCHPAD GIỮA: PHANH DỪNG KHẨN CẤP -->
    <div class="center-console">
      <div class="touchpad-btn" id="stopPad">
        <div style="font-size: 1.3rem;">🛑</div>
        <span>PHANH GẤP</span>
      </div>
    </div>

    <!-- RIGHT STICK: XOAY TRÒN CW / CCW -->
    <div class="joystick-zone">
      <div class="joystick-base" id="rightBase">
        <div class="joystick-thumb" id="rightThumb"></div>
      </div>
      <div class="joystick-label">🔄 XOAY TRÒN (R3)</div>
    </div>
  </div>

  <!-- BẢNG CÀI ĐẶT -->
  <div class="settings-drawer">
    <div class="slider-group">
      <div class="slider-title">
        <span>Tốc độ tối đa</span>
        <span id="rpmLabel" style="color: var(--ps-cyan);">70 RPM</span>
      </div>
      <input type="range" id="rpmSlider" min="30" max="150" value="70" step="5">
    </div>

    <div class="slider-group">
      <div class="slider-title">
        <span>Độ hãm phanh khi nhả cần (Decel)</span>
        <span id="decelLabel" style="color: var(--ps-cyan);">Rất êm (Chuẩn)</span>
      </div>
      <input type="range" id="decelSlider" min="8" max="35" value="20" step="1">
    </div>

    <div class="slider-group">
      <div class="slider-title">
        <span>Cân chỉnh lệch thẳng (Trim)</span>
        <span id="trimLabel" style="color: var(--ps-cyan);">0% (Chuẩn)</span>
      </div>
      <input type="range" id="trimSlider" min="-12" max="12" value="0" step="1">
    </div>

    <!-- BỘ NÚT ĐIỀU KHIỂN KHÓA / MỞ EN CHO DRIVER SERVO -->
    <div class="en-control-card">
      <div class="en-header">
        <span>⚡ ĐIỀU KHIỂN ENABLE (EN)</span>
        <span id="enStatusText" class="en-status-pill en-active">ĐANG KHÓA (CHẠY)</span>
      </div>
      <div class="btn-row">
        <button id="btnEnLock" class="ps-btn-en active-lock" onclick="setEn(1)">
          🔒 KHÓA EN (CHẠY ĐƯỢC)
        </button>
        <button id="btnEnFree" class="ps-btn-en" onclick="setEn(0)">
          🔓 MỞ EN (THẢ TỰ DO)
        </button>
      </div>
      <label class="en-invert-row">
        <input type="checkbox" id="chkInvertEn" onchange="toggleInvertEn(this.checked)">
        <span>Đảo mức kích EN (Tích vào nếu cắm EN+ vào GPIO)</span>
      </label>
    </div>
  </div>

  <script>
    let ws = null;
    let isConnected = false;
    let curVx = 0.0, curVy = 0.0, curW = 0.0;
    let isDriving = false;

    // KẾT NỐI WEBSOCKET CỰC NHANH (PORT 81)
    function connectWebSocket() {
      const host = window.location.hostname || '192.168.4.1';
      ws = new WebSocket('ws://' + host + ':81/');

      ws.onopen = () => {
        isConnected = true;
        document.getElementById('robotMode').innerText = 'KẾT NỐI SIÊU TỐC (1ms)';
        document.getElementById('robotMode').style.color = '#10b981';
        document.getElementById('wsLightbar').style.background = '#00f0ff';
      };

      ws.onmessage = (e) => {
        if (typeof e.data === 'string' && e.data.startsWith('STATE:')) {
          const parts = e.data.substring(6).split(',');
          for (const p of parts) {
            const [k, v] = p.split('=');
            if (k === 'E') updateEnUI(v === '1');
            if (k === 'INV') {
              const chk = document.getElementById('chkInvertEn');
              if (chk) chk.checked = (v === '1');
            }
          }
        }
      };

      ws.onclose = () => {
        isConnected = false;
        document.getElementById('robotMode').innerText = 'MẤT KẾT NỐI - ĐANG THỬ LẠI...';
        document.getElementById('robotMode').style.color = '#ef4444';
        document.getElementById('wsLightbar').style.background = '#ef4444';
        setTimeout(connectWebSocket, 800);
      };

      ws.onerror = () => {
        if (ws) ws.close();
      };
    }
    connectWebSocket();

    // Gửi có kiểm soát hàng đợi - chỉ bỏ qua nếu hàng đợi thực sự bị ứ đọng (> 512 bytes)
    function sendWS(msg) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        if (ws.bufferedAmount < 512) {
          ws.send(msg);
        }
      }
    }

    // Gửi cưỡng bức (dùng cho lệnh DỪNG khẩn cấp, không bao giờ được bỏ qua)
    function sendWSForce(msg) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(msg);
      }
    }

    // Vòng lặp gửi tọa độ liên tục (chu kỳ 45ms ~ 22 FPS, tối ưu băng thông WiFi ESP32)
    setInterval(() => {
      if (isDriving) {
        sendWS(`M:${curVx.toFixed(2)},${curVy.toFixed(2)},${curW.toFixed(2)}`);
      }
    }, 45);

    // HÀM DỪNG LẬP TỨC KHI BUÔNG TAY
    function stopAll() {
      curVx = 0.0;
      curVy = 0.0;
      curW  = 0.0;
      isDriving = false;

      // Gửi lệnh dừng M:0,0,0 liên tiếp 3 lần để triệt tiêu hoàn toàn rớt gói
      sendWSForce('M:0,0,0');
      setTimeout(() => sendWSForce('M:0,0,0'), 25);
      setTimeout(() => sendWSForce('M:0,0,0'), 60);

      document.getElementById('robotMode').innerText = 'HÃM PHANH DỪNG';
      document.getElementById('robotMode').style.color = '#eab308';
      setTimeout(() => {
        if (!isDriving && isConnected) {
          document.getElementById('robotMode').innerText = 'SẴN SÀNG (WS)';
          document.getElementById('robotMode').style.color = '#10b981';
        }
      }, 250);
    }

    // Kiểm tra dừng an toàn khi nhả 1 trong 2 cần gạt
    function checkShouldStop() {
      if (Math.abs(curVx) < 0.04 && Math.abs(curVy) < 0.04 && Math.abs(curW) < 0.04) {
        stopAll();
      } else {
        sendWSForce(`M:${curVx.toFixed(2)},${curVy.toFixed(2)},${curW.toFixed(2)}`);
      }
    }

    // KHỞI TẠO CẦN GẠT ANALOG BẰNG POINTER EVENTS (Chuẩn tuyệt đối trên mọi điện thoại & trình duyệt)
    function initJoystick(baseId, thumbId, onMove, onRelease) {
      const base = document.getElementById(baseId);
      const thumb = document.getElementById(thumbId);
      let activePointerId = null;
      const maxDist = 42;

      function updateStick(clientX, clientY) {
        const rect = base.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        let dx = clientX - centerX;
        let dy = clientY - centerY;
        const dist = Math.hypot(dx, dy);

        if (dist > maxDist) {
          dx = (dx / dist) * maxDist;
          dy = (dy / dist) * maxDist;
        }

        thumb.style.transform = `translate(${dx}px, ${dy}px)`;

        // Vùng chết Deadzone 8% chống trôi ngón tay
        let normDist = dist / maxDist;
        let normX = 0, normY = 0;
        if (normDist > 0.08) {
          normX = dx / maxDist;
          normY = -dy / maxDist; // Kéo lên là Y dương (tiến)
        }
        onMove(normX, normY);
      }

      base.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        if (activePointerId !== null) return;
        activePointerId = e.pointerId;
        try {
          base.setPointerCapture(e.pointerId);
        } catch (_) {}
        thumb.style.transition = 'none';
        updateStick(e.clientX, e.clientY);
      });

      base.addEventListener('pointermove', (e) => {
        if (e.pointerId !== activePointerId) return;
        updateStick(e.clientX, e.clientY);
      });

      function onPointerEnd(e) {
        if (e.pointerId !== activePointerId) return;
        try {
          base.releasePointerCapture(e.pointerId);
        } catch (_) {}
        activePointerId = null;

        // Bật lò xo về tâm dứt khoát
        thumb.style.transition = 'transform 0.1s cubic-bezier(0.175, 0.885, 0.32, 1.275)';
        thumb.style.transform = 'translate(0px, 0px)';
        onRelease();
      }

      base.addEventListener('pointerup', onPointerEnd);
      base.addEventListener('pointercancel', onPointerEnd);
    }

    // 1. CẦN TRÁI (L3): DI CHUYỂN 360 ĐỘ
    initJoystick('leftBase', 'leftThumb', 
      (nx, ny) => {
        isDriving = true;
        curVx = nx;
        curVy = ny;
        document.getElementById('robotMode').innerText = 'L3: RUN 360°';
        document.getElementById('robotMode').style.color = '#00f0ff';
      }, 
      () => {
        curVx = 0.0;
        curVy = 0.0;
        checkShouldStop();
      }
    );

    // 2. CẦN PHẢI (R3): XOAY TRÒN (Kéo phải -> Xoay phải, Kéo trái -> Xoay trái)
    initJoystick('rightBase', 'rightThumb', 
      (nx, ny) => {
        isDriving = true;
        curW = nx; // Đã đảo chiều: nx dương (kéo phải) -> xoay phải, nx âm (kéo trái) -> xoay trái
        document.getElementById('robotMode').innerText = 'R3: ROTATE';
        document.getElementById('robotMode').style.color = '#00f0ff';
      }, 
      () => {
        curW = 0.0;
        checkShouldStop();
      }
    );

    // Phanh Touchpad giữa
    document.getElementById('stopPad').addEventListener('pointerdown', (e) => {
      e.preventDefault();
      stopAll();
    });

    // Sliders
    document.getElementById('rpmSlider').addEventListener('input', (e) => {
      const v = e.target.value;
      document.getElementById('rpmLabel').innerText = v + ' RPM';
      document.getElementById('speedVal').innerText = v + ' RPM';
      sendWSForce('S:' + v);
    });

    document.getElementById('decelSlider').addEventListener('input', (e) => {
      const v = parseInt(e.target.value);
      let text = 'Bình thường';
      if (v <= 12) text = 'Cực êm (Trôi nhẹ)';
      else if (v <= 24) text = 'Rất êm (Chuẩn)';
      else text = 'Dừng dứt khoát';
      document.getElementById('decelLabel').innerText = text;
      sendWSForce('D:' + (v * 0.001).toFixed(4));
    });

    document.getElementById('trimSlider').addEventListener('input', (e) => {
      const v = parseInt(e.target.value);
      if (v === 0) document.getElementById('trimLabel').innerText = '0% (Chuẩn)';
      else if (v > 0) document.getElementById('trimLabel').innerText = '+' + v + '% (Bù Phải)';
      else document.getElementById('trimLabel').innerText = v + '% (Bù Trái)';
      sendWSForce('T:' + (v * 0.01).toFixed(3));
    });

    // Hàm điều khiển Khóa EN / Mở EN
    function setEn(state) {
      sendWSForce('E:' + state);
      updateEnUI(state === 1);
    }

    function toggleInvertEn(inverted) {
      sendWSForce('I:' + (inverted ? '1' : '0'));
    }

    function updateEnUI(isLocked) {
      const btnLock = document.getElementById('btnEnLock');
      const btnFree = document.getElementById('btnEnFree');
      const pill = document.getElementById('enStatusText');
      const badge = document.getElementById('enBadge');

      if (isLocked) {
        if (btnLock) btnLock.classList.add('active-lock');
        if (btnFree) btnFree.classList.remove('active-free');
        if (pill) {
          pill.innerText = 'ĐANG KHÓA (CHẠY)';
          pill.className = 'en-status-pill en-active';
        }
        if (badge) {
          badge.innerText = '🔒 KHÓA';
          badge.style.color = '#10b981';
        }
      } else {
        if (btnLock) btnLock.classList.remove('active-lock');
        if (btnFree) btnFree.classList.add('active-free');
        if (pill) {
          pill.innerText = 'ĐANG MỞ (THẢ TỰ DO)';
          pill.className = 'en-status-pill en-inactive';
        }
        if (badge) {
          badge.innerText = '🔓 MỞ (THẢ)';
          badge.style.color = '#eab308';
        }
      }
    }
  </script>
</body>
</html>
)rawhtml";

/**
 * Xử lý các gói tin WebSocket thời gian thực (độ trễ < 2ms)
 */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT && length > 0) {
    char msg[64];
    size_t len = (length < sizeof(msg) - 1) ? length : (sizeof(msg) - 1);
    memcpy(msg, payload, len);
    msg[len] = '\0';

    if (msg[0] == 'M') { // Di chuyển: M:vx,vy,w
      float vx = 0, vy = 0, w = 0;
      if (sscanf(msg, "M:%f,%f,%f", &vx, &vy, &w) == 3) {
        if (fabs(vx) < 0.01f && fabs(vy) < 0.01f && fabs(w) < 0.01f) {
          targetVx = 0.0f;
          targetVy = 0.0f;
          targetW  = 0.0f;
        } else {
          targetVx = constrain(vx, -1.0f, 1.0f);
          targetVy = constrain(vy, -1.0f, 1.0f);
          targetW  = constrain(w,  -1.0f, 1.0f);
        }
        lastCommandTime = millis();
      }
    } 
    else if (msg[0] == 'S') { // Tốc độ RPM: S:70
      int spd = atoi(msg + 2);
      if (spd >= 10 && spd <= 350) targetRpm = spd;
      lastCommandTime = millis();
    } 
    else if (msg[0] == 'D') { // Decel: D:0.020
      float d = atof(msg + 2);
      if (d >= 0.005f && d <= 0.05f) decelRate = d;
    } 
    else if (msg[0] == 'T') { // Trim: T:0.02
      float t = atof(msg + 2);
      if (t >= -0.2f && t <= 0.2f) steeringTrim = t;
    } 
    else if (msg[0] == 'E') { // Enable: E:1 (Khóa trục/Chạy) hoặc E:0 (Mở/Thả tự do)
      motorsEnabled = (msg[2] == '1');
      applyEnPinState();
      char resp[32];
      snprintf(resp, sizeof(resp), "STATE:E=%d,INV=%d", motorsEnabled ? 1 : 0, invertEnPolarity ? 1 : 0);
      webSocket.broadcastTXT(resp);
    }
    else if (msg[0] == 'I') { // Đảo cực tính EN: I:1 hoặc I:0
      invertEnPolarity = (msg[2] == '1');
      applyEnPinState();
      char resp[32];
      snprintf(resp, sizeof(resp), "STATE:E=%d,INV=%d", motorsEnabled ? 1 : 0, invertEnPolarity ? 1 : 0);
      webSocket.broadcastTXT(resp);
    }
  }
  else if (type == WStype_CONNECTED) {
    char resp[32];
    snprintf(resp, sizeof(resp), "STATE:E=%d,INV=%d", motorsEnabled ? 1 : 0, invertEnPolarity ? 1 : 0);
    webSocket.sendTXT(num, resp);
  }
}

// Giao diện Web nạp OTA qua trình duyệt (/update)
const char UPDATE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Cập nhật Firmware OTA</title>
  <style>
    :root {
      --ps-blue: #0070d1;
      --ps-cyan: #00f0ff;
      --ps-dark: #0a0c13;
      --ps-card: #151824;
      --ps-border: #232738;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body {
      background: radial-gradient(circle at center, #171b2b, var(--ps-dark));
      color: #fff;
      min-height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
      padding: 16px;
    }
    .card {
      width: 100%;
      max-width: 440px;
      background: var(--ps-card);
      border: 1px solid var(--ps-border);
      border-radius: 20px;
      padding: 24px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.7);
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    h2 { font-size: 1.25rem; color: var(--ps-cyan); text-align: center; }
    p { font-size: 0.8rem; color: #94a3b8; line-height: 1.5; text-align: center; }
    .file-dropzone {
      border: 2px dashed #2e3549;
      border-radius: 14px;
      padding: 24px 16px;
      text-align: center;
      cursor: pointer;
      background: rgba(255,255,255,0.02);
      transition: 0.2s;
    }
    .file-dropzone:hover { border-color: var(--ps-cyan); background: rgba(0,240,255,0.04); }
    input[type=file] { display: none; }
    .btn {
      width: 100%;
      padding: 12px;
      background: linear-gradient(135deg, var(--ps-blue), #00b4d8);
      border: none;
      border-radius: 12px;
      color: #fff;
      font-weight: 700;
      font-size: 0.9rem;
      cursor: pointer;
      box-shadow: 0 4px 14px rgba(0,112,209,0.4);
      transition: 0.2s;
    }
    .btn:disabled { opacity: 0.4; cursor: not-allowed; box-shadow: none; }
    .btn-back {
      background: transparent;
      border: 1px solid var(--ps-border);
      color: #94a3b8;
      box-shadow: none;
    }
    .progress-wrap {
      width: 100%;
      height: 10px;
      background: #1f2438;
      border-radius: 5px;
      overflow: hidden;
      display: none;
    }
    .progress-bar {
      width: 0%;
      height: 100%;
      background: linear-gradient(90deg, var(--ps-blue), var(--ps-cyan));
      transition: width 0.1s;
    }
    #statusText {
      font-size: 0.85rem;
      text-align: center;
      font-weight: 600;
    }
  </style>
</head>
<body>
  <div class="card">
    <h2>🚀 NẠP FIRMWARE OTA (WIFI)</h2>
    <p>Chọn file <b>firmware.bin</b> từ máy tính hoặc điện thoại để nạp không dây vào ESP32-S3.</p>

    <label class="file-dropzone" for="firmwareFile">
      <div style="font-size: 2.2rem; margin-bottom: 8px;">📦</div>
      <div id="fileLabel" style="font-size: 0.85rem; color: #cbd5e1; font-weight: 600;">Bấm vào đây để chọn file .bin</div>
      <input type="file" id="firmwareFile" accept=".bin" onchange="onFileSelected()">
    </label>

    <div class="progress-wrap" id="progressWrap">
      <div class="progress-bar" id="progressBar"></div>
    </div>
    <div id="statusText"></div>

    <button class="btn" id="uploadBtn" onclick="uploadFirmware()" disabled>BẮT ĐẦU NẠP FIRMWARE</button>
    <button class="btn btn-back" onclick="location.href='/'">QUAY LẠI TAY CẦM PS4</button>
  </div>

  <script>
    function onFileSelected() {
      const fileInput = document.getElementById('firmwareFile');
      const fileLabel = document.getElementById('fileLabel');
      const uploadBtn = document.getElementById('uploadBtn');
      if (fileInput.files.length > 0) {
        const file = fileInput.files[0];
        fileLabel.innerText = file.name + ' (' + (file.size / 1024).toFixed(1) + ' KB)';
        uploadBtn.disabled = false;
      }
    }

    function uploadFirmware() {
      const fileInput = document.getElementById('firmwareFile');
      if (fileInput.files.length === 0) return;

      const file = fileInput.files[0];
      const formData = new FormData();
      formData.append('update', file);

      const xhr = new XMLHttpRequest();
      xhr.open('POST', '/update', true);

      const progressWrap = document.getElementById('progressWrap');
      const progressBar = document.getElementById('progressBar');
      const statusText = document.getElementById('statusText');
      const uploadBtn = document.getElementById('uploadBtn');

      uploadBtn.disabled = true;
      progressWrap.style.display = 'block';
      statusText.style.color = '#00f0ff';
      statusText.innerText = 'Đang truyền firmware vào ESP32...';

      xhr.upload.onprogress = (e) => {
        if (e.lengthComputable) {
          const percent = Math.round((e.loaded / e.total) * 100);
          progressBar.style.width = percent + '%';
          statusText.innerText = `Tiến độ: ${percent}%`;
        }
      };

      xhr.onload = () => {
        if (xhr.status === 200) {
          statusText.style.color = '#10b981';
          statusText.innerText = '✅ Nạp thành công! Robot đang khởi động lại (khoảng 3 giây)...';
          setTimeout(() => { location.href = '/'; }, 4000);
        } else {
          statusText.style.color = '#ef4444';
          statusText.innerText = '❌ Nạp thất bại: ' + xhr.responseText;
          uploadBtn.disabled = false;
        }
      };

      xhr.onerror = () => {
        statusText.style.color = '#ef4444';
        statusText.innerText = '❌ Lỗi kết nối WiFi!';
        uploadBtn.disabled = false;
      };

      xhr.send(formData);
    }
  </script>
</body>
</html>
)rawhtml";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

/**
 * Task Mạng chạy độc quyền trên Core 0: Xử lý WiFi, WebServer, WebSocket & OTA
 */
void networkTask(void *pvParameters) {
  while (true) {
    ArduinoOTA.handle();
    webSocket.loop();
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1)); // Nhường 1ms cho WiFi stack phần cứng trên Core 0
  }
}

/**
 * Task Động cơ chạy độc quyền trên Core 1: Chuyên trách phát xung liên tục
 */
void motorTask(void *pvParameters) {
  uint32_t nextStepUs[3] = {0, 0, 0};
  uint32_t stepIntervalUs[3] = {0, 0, 0};
  bool motorActive[3] = {false, false, false};

  float currentVx = 0.0f;
  float currentVy = 0.0f;
  float currentW  = 0.0f;

  const float SQRT3_2 = 0.866025404f;

  while (true) {
    // WATCHDOG AN TOÀN: Nếu sau 180ms không có gói tin WebSocket -> Tự động dừng phanh ngay!
    if (millis() - lastCommandTime > SAFETY_TIMEOUT_MS) {
      targetVx = 0.0f;
      targetVy = 0.0f;
      targetW  = 0.0f;
    }

    // Làm mịn vận tốc S-Curve
    auto rampValue = [](float current, float target, float accel, float decel) -> float {
      if (fabs(target) > fabs(current)) {
        if (target > current) {
          current += accel;
          if (current > target) current = target;
        } else {
          current -= accel;
          if (current < target) current = target;
        }
      } else {
        if (target > current) {
          current += decel;
          if (current > target) current = target;
        } else {
          current -= decel;
          if (current < target) current = target;
        }
        if (target == 0.0f && fabs(current) < 0.02f) {
          current = 0.0f;
        }
      }
      return current;
    };

    currentVx = rampValue(currentVx, targetVx, accelRate, decelRate);
    currentVy = rampValue(currentVy, targetVy, accelRate, decelRate);
    currentW  = rampValue(currentW,  targetW,  accelRate, decelRate);

    // Nếu xe đã dừng hẳn hoặc bị Disable
    if (!motorsEnabled || (currentVx == 0.0f && currentVy == 0.0f && currentW == 0.0f)) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // Động học 3 bánh Omni:
    float vx = -currentVx; // Đã đảo dấu trục X
    float vy = currentVy;
    float w  = -(currentW + (steeringTrim * vy)); // Đảo dấu chiều xoay: Xoay trái -> CCW, Xoay phải -> CW

    float v1 = -vx + w;
    float v2 =  0.5f * vx - SQRT3_2 * vy + w;
    float v3 =  0.5f * vx + SQRT3_2 * vy + w;

    float speeds[3] = {v1, v2, v3};
    uint32_t now = micros();

    for (int i = 0; i < 3; i++) {
      float spd = speeds[i];
      if (fabs(spd) < 0.03f) {
        motorActive[i] = false;
      } else {
        motorActive[i] = true;
        digitalWrite(motors[i].pinDir, (spd >= 0) ? HIGH : LOW);

        float freq = fabs(spd) * ((float)targetRpm * (float)PULSES_PER_REV) / 60.0f;
        if (freq < 80.0f) freq = 80.0f;
        stepIntervalUs[i] = (uint32_t)(1000000.0f / freq);
      }
    }

    // Vòng lặp phát xung thời gian thực (~2.5ms mỗi chu kỳ)
    for (int loop = 0; loop < 80; loop++) {
      now = micros();
      bool steppedAny = false;

      for (int i = 0; i < 3; i++) {
        if (motorActive[i] && (int32_t)(now - nextStepUs[i]) >= 0) {
          nextStepUs[i] += stepIntervalUs[i];
          if ((int32_t)(now - nextStepUs[i]) > (int32_t)stepIntervalUs[i]) {
            nextStepUs[i] = now + stepIntervalUs[i];
          }
          digitalWrite(motors[i].pinPulse, LOW);
          steppedAny = true;
        }
      }

      if (steppedAny) {
        delayMicroseconds(6); // Độ rộng xung 6us
        for (int i = 0; i < 3; i++) {
          digitalWrite(motors[i].pinPulse, HIGH);
        }
      }

      delayMicroseconds(10);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);

  // Vô hiệu hóa Task Watchdog trên Core 1 để motorTask chạy độc quyền không bị panic
  disableCore1WDT();

  // Cấu hình các chân điều khiển động cơ
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].pinPulse, OUTPUT);
    pinMode(motors[i].pinDir,   OUTPUT);

    digitalWrite(motors[i].pinPulse, HIGH);
    digitalWrite(motors[i].pinDir,   HIGH);
  }
  applyEnPinState(); // Khởi tạo các chân EN ở chế độ Trở kháng cao (Hi-Z)

  // Khởi tạo WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                 // TẮT TIẾT KIỆM PIN WIFI -> ĐỘ TRỄ DƯỚI 2MS
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Đặt công suất phát sóng Wi-Fi tối đa (19.5dBm)
  WiFi.softAP(AP_SSID, AP_PASS, 6);     // Phát ở Kênh 6 (tránh trùng kênh 1 với router gia đình)
  esp_wifi_set_ps(WIFI_PS_NONE);        // Cấm radio ngủ

  IPAddress myIP = WiFi.softAPIP();

  Serial.println("\n========================================================");
  Serial.println(" ROBOT 3 BANH OMNI - DUAL-CORE KHÔNG TRỄ (ESP32-S3)     ");
  Serial.println("========================================================");
  Serial.print(" WiFi SSID : "); Serial.println(AP_SSID);
  Serial.print(" Mat khau  : "); Serial.println(AP_PASS);
  Serial.print(" Dia chi IP: http://"); Serial.println(myIP);
  Serial.println("========================================================");

  // Cấu hình Web Router phục vụ HTML giao diện
  server.on("/", handleRoot);

  // Cấu hình Web OTA phục vụ nạp code trực tiếp từ trình duyệt
  server.on("/update", HTTP_GET, []() {
    server.send(200, "text/html", UPDATE_HTML);
  });

  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "CAP NHAT THAT BAI!" : "CAP NHAT THANH CONG!");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("\n[Web OTA] Bat dau nap firmware: %s\n", upload.filename.c_str());
      motorsEnabled = false; // Ngắt động cơ an toàn khi nạp
      applyEnPinState();
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[Web OTA] Nap thanh cong: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();

  // Khởi tạo ArduinoOTA (cho phép nạp code từ PlatformIO qua mạng WiFi)
  ArduinoOTA.setHostname("ESP32-OMNI-ROBOT");
  ArduinoOTA.onStart([]() {
    motorsEnabled = false; // Ngắt động cơ an toàn khi nạp
    applyEnPinState();
    Serial.println("\n[ArduinoOTA] Bat dau nap firmware qua WiFi...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[ArduinoOTA] Hoan tat! Dang khoi dong lai...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[ArduinoOTA] Tien do: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[ArduinoOTA] Loi[%u]: ", error);
  });
  ArduinoOTA.begin();
  Serial.println("[OK] ArduinoOTA da san sang nhan code qua WiFi!");

  // Khởi động WebSocket Server trên Port 81
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[OK] WebSocket Server da khoi dong tren cong 81!");

  // Task Mạng chạy độc quyền trên Core 0 (cùng Core với phần cứng WiFi lwIP)
  xTaskCreatePinnedToCore(
    networkTask,
    "NetworkTask",
    4096,
    NULL,
    3, // Độ ưu tiên 3 (trên idle, dưới wifi interrupt)
    NULL,
    0  // CORE 0
  );

  // Task Động cơ chạy độc quyền trên Core 1 (chuyên phát xung thời gian thực)
  xTaskCreatePinnedToCore(
    motorTask,
    "MotorTask",
    4096,
    NULL,
    2, // Độ ưu tiên 2
    NULL,
    1  // CORE 1
  );

  Serial.println("[OK] Da khoi tao song song 2 Core: Core 0 (Network) & Core 1 (Motor)!");
}

void loop() {
  // loopTask trên Core 1 ngủ, dành 100% tài nguyên Core 1 cho motorTask
  vTaskDelay(pdMS_TO_TICKS(100));
}