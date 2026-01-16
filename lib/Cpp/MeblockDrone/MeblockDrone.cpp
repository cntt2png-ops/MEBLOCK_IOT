#include <Arduino.h>
// *************************************************************  Library  ************************************************************* //

#include <WiFi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_WE.h>

// ===================== MeblockDrone runtime configuration =====================
static uint8_t g_pinMA = 5;
static uint8_t g_pinMB = 3;
static uint8_t g_pinMC = 1;
static uint8_t g_pinMD = 7;
static uint8_t g_i2cSDA = 8;
static uint8_t g_i2cSCL = 9;
static const char* g_apSsid = "MEBlock Drone V1";
static const char* g_apPass = "12345678";
// ===============================================================================

#define MPU9250_ADDR 0x68

// *************************************************************  Parameters & Variables  ************************************************************* //

// Motors 
#define LEDC_TIMER_10_BIT 10
#define LEDC_BASE_FREQ 20000

// Objects 
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
Adafruit_BMP280 bmp;
Preferences preferences;
Madgwick MadgwickFilter;

// Timer 
unsigned long lastInnerTime = 0;
unsigned long lastOuterTime = 0;
unsigned long lastBaroTime  = 0;
unsigned long lastDNSTime = 0;

const float innerHz = 400.0;    // Inner loop rate (Madgwick + inner PID)
const float outerHz = 100.0;    // Outer loop rate (angle PID / altitude)
const float baroHz  = 25.0;     // Barometer update rate

const float innerDt = 1.0 / innerHz;
const float outerDt = 1.0 / outerHz;
const float baroDt  = 1.0 / baroHz;

// Initials
float targetRoll = 0 ,targetPitch = 0 ,targetYaw = 0;
float currentRoll = 0 ,currentPitch = 0 ,currentYaw = 0;
float yaw_setpoint = 0.0; // deg
float yaw_rate_target = 0;
float yaw_ref = 0.0f;
float baroAltitude = 0;
float currentAltitude = 0;
float altitude_setpoint = 0.0; // meter
float altitude_rate_target = 0.0;
float altitude_baseline = 0.0;
float max_altitude = 0.0;
float min_altitude = 0.0;
float g = 9.80665; // m/s^2

int baseSpeed = 480;
int integralLimit = 10;
float range_altitude = 15;
float maxRateChange = 1; // m/s

// -------- Speed tuning (exposed to Web UI) --------
float maxAngleDeg = 30.0f;        // deg, max roll/pitch angle command
float maxYawRateDegS = 90.0f;     // deg/s, max yaw rate command
float maxAltRateMps = 5.0f;       // m/s, max climb/descent rate command
float rpResponse = 1.0f;          // roll/pitch response scale
float yawResponse = 1.0f;         // yaw response scale
float altResponse = 1.0f;         // altitude rate response scale
float rpExpo = 0.0f;              // 0..1 (higher = softer around center)
float yawExpo = 0.0f;             // 0..1
float altExpo = 0.0f;             // 0..1
float kff_roll = 0.5; // FeedForward Roll
float kff_pitch = 0.5; // FeedForward Pitch
float kff_yaw = 0.5; // FeedForward Yaw
float kff_altitude = 0.9; // FeedForward Altitude 

float altitude = 0;
float velocityZ = 0;
float alpha = 0.95;
float beta = 0.75;
float previousBaro = 0;

float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
float mx_bias = 0, my_bias = 0, mz_bias = 0;
float mx_scale = 1, my_scale = 1, mz_scale = 1;
float magXmin =  1000, magXmax = -1000;
float magYmin =  1000, magYmax = -1000;
float magZmin =  1000, magZmax = -1000;

// PID Control
struct PID_t {
  float P;
  float I;
  float D;
};

PID_t pidRoll_rate, pidPitch_rate, pidYaw_rate, pidAltitude_rate, pidRoll_angle, pidPitch_angle, pidYaw_angle, pidAltitude_m, pidAltitude_height;
float trimRoll, trimPitch, trimYaw, trimAltitude;

// Outer loop PID (deg and m control)
float rollKp_angle = 10.0 ,rollKi_angle = 0.0 ,rollKd_angle = 0.0;  
float rollError_angle, rollPrevError_angle = 0, rollIntegral_angle = 0;
float rollOutput_angle; 

float pitchKp_angle = 9.0 ,pitchKi_angle = 0.5 ,pitchKd_angle = 0.0; 
float pitchError_angle, pitchPrevError_angle = 0, pitchIntegral_angle = 0;
float pitchOutput_angle;

float yawKp_angle = 4.0 ,yawKi_angle = 0.0 ,yawKd_angle = 0.0; 
float yawError_angle, yawPrevError_angle = 0, yawIntegral_angle = 0;
float yawOutput_angle;

float altitudeKp_m = 1.0 ,altitudeKi_m = 0.0 ,altitudeKd_m = 0.4;
float altitudeError_m, altitudePrevError_m = 0, altitudeIntegral_m = 0;
float altitudeOutput_m;

// Inner loop PID (rate control ==> deg/s and m/s)
float rollKp_rate = 1.2 ,rollKi_rate = 0.0 ,rollKd_rate = 0.03; 
float rollError_rate, rollPrevError_rate = 0, rollIntegral_rate = 0;
float rollOutput_rate; 

float pitchKp_rate = 1.2 ,pitchKi_rate = 0.0 ,pitchKd_rate = 0.04; 
float pitchError_rate, pitchPrevError_rate = 0, pitchIntegral_rate = 0;
float pitchOutput_rate;

float yawKp_rate = 3.0 ,yawKi_rate = 0.0 ,yawKd_rate = 0.03; 
float yawError_rate, yawPrevError_rate = 0, yawIntegral_rate = 0;
float yawOutput_rate;

float altitudeKp_rate = 45.0 ,altitudeKi_rate = 0.00 ,altitudeKd_rate = 1.5;
float altitudeError_rate, altitudePrevError_rate = 0, altitudeIntegral_rate = 0;
float altitudeOutput_rate;

// Filters
float gyroX_filtered = 0; 
float gyroY_filtered = 0;
float gyroZ_filtered = 0;
float accX_filtered  = 0;
float accY_filtered  = 0;
float accZ_filtered  = 0;
float magX_filtered  = 0;
float magY_filtered  = 0;
float magZ_filtered  = 0;
float alt_filtered  = 0;

// EMA filter
float emaGyroX = 0;
float emaGyroY = 0;
float emaGyroZ = 0;
float emaAccX = 0;
float emaAccY = 0;
float emaAccZ = 0;
float emaMagX = 0;
float emaMagY = 0;
float emaMagZ = 0;
float emaAlt = 0;
float emaVZ = 0;

float alphaGyro = 0.8f; 
float alphaAcc  = 0.7f;  
float alphaMag  = 0.9f; 
float alphaAlt  = 0.5f;
float alphaVZ  = 0.6f; 

// Median filter 
float gyroX_buf[9] = {0}, gyroY_buf[9] = {0}, gyroZ_buf[9] = {0};
float accX_buf[9]  = {0}, accY_buf[9]  = {0}, accZ_buf[9]  = {0};
float magX_buf[3]  = {0}, magY_buf[3]  = {0}, magZ_buf[3]  = {0};
float alt_buf[7]  = {0};
float vz_buf[5]  = {0};
size_t gyroX_idx = 0, gyroY_idx = 0, gyroZ_idx = 0;
size_t accX_idx  = 0, accY_idx  = 0, accZ_idx  = 0;
size_t magX_idx  = 0, magY_idx  = 0, magZ_idx  = 0;
size_t alt_idx  = 0;
size_t vz_idx  = 0;

// State
bool swlPressed = false;
bool swrPressed = false;
bool lastArmButton = false;
bool Armed = false;
bool headlessMode = false; 
bool OnFlying = false;
bool initial_altitude = false;
bool initial_yaw = false;
volatile bool calibrateAccelGyroRequested = false;
volatile bool calibrateMagRequested = false;
bool AccelGyroisCalibrating = false;
bool AccelGyrocalibrationDone = false;
bool MagisCalibrating = false;
bool MagcalibrationDone = false;

// WiFi AP config
IPAddress apIP(192,168,4,1);
const byte DNS_PORT = 53;

AsyncWebServer server(80);
DNSServer dnsServer;
bool webServerRunning = false;
bool routesRegistered = false;  // register routes only once (avoid duplicates)

// ESP Now 
String getMacAddress() {
  return WiFi.macAddress();
}

typedef struct JoyData {
  int XL, YL;
  bool SWL;
  int XR, YR;
  bool SWR;
  uint8_t webCommand;
} JoyData;

JoyData incomingJoystickData;
// *************************************************************  HTML  ************************************************************* //

String htmlForm(const char* message = "") {
  String mac = getMacAddress();
  mac.toUpperCase();

  String html = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>MEBlock Drone Web Tuning V1</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    :root{
      --bg:#0e2a52;
      --panel:#123a6b;
      --card:#123a6b;
      --line:#2e5b99;
      --text:#ffffff;
      --muted:rgba(255,255,255,.72);
      --accent:#2f6bff;
      --accent2:#1f49b8;
      --bad:#ff4d6d;
      --good:#2ee59d;
      --btn:#1a4a8d;
      --btn2:#1e5aa8;
    }
    *{box-sizing:border-box}
    body{
      margin:0;
      font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
      background:var(--bg);
      color:var(--text);
    }
    .wrap{max-width:1180px;margin:0 auto;padding:14px}
    .top{
      display:flex;gap:10px;align-items:center;justify-content:space-between;
      padding:10px 12px;border:1px solid var(--line);border-radius:12px;background:var(--panel);
    }
    .title{font-weight:800;letter-spacing:.2px}
    .sub{color:var(--muted);font-size:12px}
    .badge{font-size:12px;border:1px solid var(--line);border-radius:999px;padding:6px 10px;background:#0b1430;color:var(--muted)}
    .msg{
      margin-top:10px;border:1px solid var(--line);border-radius:12px;background:#0b1430;
      padding:10px 12px;color:var(--muted);display:none
    }

    .tabs{
      margin-top:12px;display:flex;gap:8px;flex-wrap:wrap;
    }
    .tabbtn{
      border:1px solid var(--line);
      background:#0b1430;
      color:var(--muted);
      padding:8px 10px;
      border-radius:10px;
      cursor:pointer;
      font-weight:700;
    }
    .tabbtn.active{
      background:var(--btn2);
      color:#fff;
      border-color:rgba(47,107,255,.55);
    }

    .grid{margin-top:12px;display:grid;grid-template-columns:1fr;gap:12px;align-items:start}
    @media(min-width:760px){ .grid{grid-template-columns: minmax(0,1fr) 380px;} }
    .rightStack{display:flex;flex-direction:column;gap:12px}

    .card{
      border:1px solid var(--line);
      border-radius:12px;
      background:var(--card);
      padding:12px;
    }
    .card h3{
      margin:0 0 10px 0;
      font-size:14px;
      letter-spacing:.2px;
      color:#fff;
    }
    .section{margin-top:12px}
    .section h4{margin:0 0 8px 0;color:var(--muted);font-size:12px;font-weight:800;text-transform:uppercase;letter-spacing:.6px}
    .fields{display:grid;grid-template-columns:1fr 1fr;gap:10px}
    @media(min-width:860px){ .fields{grid-template-columns:1fr 1fr 1fr;} }
    label{display:block;font-size:12px;color:var(--muted);margin-bottom:6px}
    .pname{font-weight:900;color:#fff;letter-spacing:.2px}
    input[type="number"]{
      width:100%;
      padding:10px 10px;
      border-radius:10px;
      border:1px solid var(--line);
      outline:none;
      background:#0b1430;
      color:#fff;
      font-weight:700;
    }
    input[type="number"]:focus{border-color:rgba(47,107,255,.7)}
    .row{display:flex;gap:10px;flex-wrap:wrap}
    .btn{
      border:1px solid var(--line);
      background:var(--btn);
      color:#fff;
      padding:10px 12px;
      border-radius:10px;
      cursor:pointer;
      font-weight:800;
    }
    .btn.primary{background:var(--accent2);border-color:rgba(47,107,255,.6)}
    .btn.danger{background:#3b1a26;border-color:rgba(255,77,109,.55)}
    .hint{color:var(--muted);font-size:12px;line-height:1.35}
    .mono{font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;}
    .teleGrid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:8px}
    .teleItem{padding:10px;border:1px solid var(--line);border-radius:10px;background:#0b1430}
    .teleItem .k{color:var(--muted);font-size:12px}
    .teleItem .v{font-size:18px;font-weight:900;margin-top:6px}
    .hide{display:none}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div>
        <div class="title">MEBlock Drone Web Tuning V1</div>
        <div class="sub">MAC: <span class="mono" id="mac">{{MAC}}</span></div>
      </div>
      <div class="badge mono" id="ap">AP: 192.168.4.1</div>
    </div>

    <div class="msg" id="msg">{{MSG}}</div>

    <div class="tabs">
      <button class="tabbtn active" data-tab="pid">PID</button>
      <button class="tabbtn" data-tab="stability">Độ ổn định</button>
      <button class="tabbtn" data-tab="speed">Tốc độ</button>
      <button class="tabbtn" data-tab="load">Tải</button>
    </div>

    <div class="grid">
      <div class="left">
        <div class="card">
        <form id="tuneForm" action="/save" method="POST">
          <!-- PID -->
          <div class="tab" id="tab-pid">
            <h3>PID (tổng)</h3>

            <div class="section">
              <h4>Rate (quay)</h4>
              <div class="hint">P: phản ứng nhanh • I: giảm lệch kéo dài • D: giảm rung/overshoot (D quá lớn dễ “rít”).</div>
              <div class="fields">
                <div><label>Roll P</label><input name="pRoll_rate" type="number" step="0.001"></div>
                <div><label>Roll I</label><input name="iRoll_rate" type="number" step="0.001"></div>
                <div><label>Roll D</label><input name="dRoll_rate" type="number" step="0.001"></div>

                <div><label>Pitch P</label><input name="pPitch_rate" type="number" step="0.001"></div>
                <div><label>Pitch I</label><input name="iPitch_rate" type="number" step="0.001"></div>
                <div><label>Pitch D</label><input name="dPitch_rate" type="number" step="0.001"></div>

                <div><label>Yaw P</label><input name="pYaw_rate" type="number" step="0.001"></div>
                <div><label>Yaw I</label><input name="iYaw_rate" type="number" step="0.001"></div>
                <div><label>Yaw D</label><input name="dYaw_rate" type="number" step="0.001"></div>

                <div><label>Altitude Rate P</label><input name="pAltitude_rate" type="number" step="0.001"></div>
                <div><label>Altitude Rate I</label><input name="iAltitude_rate" type="number" step="0.001"></div>
                <div><label>Altitude Rate D</label><input name="dAltitude_rate" type="number" step="0.001"></div>
              </div>
            </div>

            <div class="section">
              <h4>Angle (góc)</h4>
              <div class="fields">
                <div><label>Roll P</label><input name="pRoll_angle" type="number" step="0.001"></div>
                <div><label>Roll I</label><input name="iRoll_angle" type="number" step="0.001"></div>
                <div><label>Roll D</label><input name="dRoll_angle" type="number" step="0.001"></div>

                <div><label>Pitch P</label><input name="pPitch_angle" type="number" step="0.001"></div>
                <div><label>Pitch I</label><input name="iPitch_angle" type="number" step="0.001"></div>
                <div><label>Pitch D</label><input name="dPitch_angle" type="number" step="0.001"></div>

                <div><label>Yaw P</label><input name="pYaw_angle" type="number" step="0.001"></div>
                <div><label>Yaw I</label><input name="iYaw_angle" type="number" step="0.001"></div>
                <div><label>Yaw D</label><input name="dYaw_angle" type="number" step="0.001"></div>

                <div><label>Altitude (m) P</label><input name="pAltitude_m" type="number" step="0.001"></div>
                <div><label>Altitude (m) I</label><input name="iAltitude_m" type="number" step="0.001"></div>
                <div><label>Altitude (m) D</label><input name="dAltitude_m" type="number" step="0.001"></div>
              </div>
            </div>
          </div>

          <!-- Stability -->
          <div class="tab hide" id="tab-stability">
            <h3>Độ ổn định</h3>
            <div class="hint">Dùng để chỉnh chống trôi lệch khi thả joystick, cân bằng trung tính.</div>
            <div class="section">
              <h4>Trim (chống trôi)</h4>
              <div class="hint">
                Trim dùng để bù lệch cơ khí/cân bằng. Giá trị <b>dương</b>/<b>âm</b>:
                Roll (+) nghiêng phải / (-) nghiêng trái • Pitch (+) ngửa (lùi) / (-) chúc (tới) • Yaw (+) quay phải / (-) quay trái • Alt (+) tăng lực nâng / (-) giảm.
              </div>
              <div class="fields">
                <div><label>Trim Roll</label><input name="trimRoll" type="number" step="0.01"></div>
                <div><label>Trim Pitch</label><input name="trimPitch" type="number" step="0.01"></div>
                <div><label>Trim Yaw</label><input name="trimYaw" type="number" step="0.01"></div>
                <div><label>Trim Altitude</label><input name="trimAltitude" type="number" step="0.01"></div>
              </div>
            </div>

            <div class="section">
              <h4>Ổn định &amp; chống rung</h4>
              <div class="fields">
                <div>
                  <label>Integral Limit</label>
                  <input name="integralLimit" type="number" step="1">
                  <div class="sub">Giới hạn I-term. Tăng = bù lệch mạnh hơn nhưng dễ “quá tay”.</div>
                </div>
                <div>
                  <label>Max Alt Rate Change</label>
                  <input name="maxRateChange" type="number" step="0.01">
                  <div class="sub">Giới hạn thay đổi tốc độ lên/xuống mỗi bước. Nhỏ = mượt, lớn = nhanh.</div>
                </div>
                <div>
                  <label>Alpha (lọc)</label>
                  <input name="alpha" type="number" step="0.01" min="0" max="1">
                  <div class="sub">Alpha cao = lọc mạnh (mượt hơn) nhưng phản hồi chậm hơn.</div>
                </div>
                <div>
                  <label>Beta (lọc)</label>
                  <input name="beta" type="number" step="0.01" min="0" max="1">
                  <div class="sub">Beta dùng trong lọc/ước lượng vận tốc. Tăng = “bám” nhanh hơn nhưng dễ nhiễu.</div>
                </div>
              </div>
            </div>
          </div>

          <!-- Speed -->
          <div class="tab hide" id="tab-speed">
            <h3>Tốc độ</h3>
            <div class="hint">
              Dùng để chỉnh “gắt / nhanh” khi điều khiển. Nếu drone phản ứng quá mạnh → giảm giới hạn hoặc tăng expo.
            </div>

            <div class="section">
              <h4>Giới hạn (Limit)</h4>
              <div class="fields">
                <div>
                  <label>Max Angle (°)</label>
                  <input name="maxAngleDeg" type="number" step="0.1">
                  <div class="sub">Tăng = nghiêng nhiều hơn (bay nhanh/đổi hướng gắt hơn). Giảm = hiền và dễ lái.</div>
                </div>
                <div>
                  <label>Max Yaw Rate (°/s)</label>
                  <input name="maxYawRateDegS" type="number" step="0.1">
                  <div class="sub">Tăng = quay nhanh hơn. Giảm = quay chậm, mượt.</div>
                </div>
                <div>
                  <label>Max Alt Rate (m/s)</label>
                  <input name="maxAltRateMps" type="number" step="0.1">
                  <div class="sub">Tăng = lên/xuống nhanh. Giảm = lên/xuống chậm và ổn định.</div>
                </div>
              </div>
            </div>

            <div class="section">
              <h4>Độ nhạy (Response)</h4>
              <div class="fields">
                <div>
                  <label>Roll/Pitch Response</label>
                  <input name="rpResponse" type="number" step="0.01">
                  <div class="sub">&gt;1 = nhạy hơn, &lt;1 = hiền hơn.</div>
                </div>
                <div>
                  <label>Yaw Response</label>
                  <input name="yawResponse" type="number" step="0.01">
                  <div class="sub">&gt;1 = quay nhạy hơn, &lt;1 = quay hiền hơn.</div>
                </div>
                <div>
                  <label>Alt Response</label>
                  <input name="altResponse" type="number" step="0.01">
                  <div class="sub">&gt;1 = lên/xuống nhạy hơn, &lt;1 = mượt hơn.</div>
                </div>
              </div>
            </div>

            <div class="section">
              <h4>Expo (mượt quanh trung tâm)</h4>
              <div class="fields">
                <div>
                  <label>Roll/Pitch Expo (0..1)</label>
                  <input name="rpExpo" type="number" step="0.01" min="0" max="1">
                  <div class="sub">Tăng expo = mượt khi chạm nhẹ joystick, vẫn giữ tốc độ ở biên.</div>
                </div>
                <div>
                  <label>Yaw Expo (0..1)</label>
                  <input name="yawExpo" type="number" step="0.01" min="0" max="1">
                  <div class="sub">Tăng expo = quay mượt quanh trung tâm.</div>
                </div>
                <div>
                  <label>Alt Expo (0..1)</label>
                  <input name="altExpo" type="number" step="0.01" min="0" max="1">
                  <div class="sub">Tăng expo = lên/xuống mượt khi điều khiển nhỏ.</div>
                </div>
              </div>
            </div>
          </div>

          <!-- Load -->
          <div class="tab hide" id="tab-load">
            <h3>Tải</h3>
            <div class="hint">Chỉnh khi gắn vỏ nặng / thay đổi tải để bay ổn định hơn.</div>
            <div class="section">
              <h4>Tải (Load) / lực nâng</h4>
              <div class="hint">Chỉnh khi gắn vỏ nặng, thay pin, đổi cánh… để giữ độ cao ổn hơn.</div>
              <div class="fields">
                <div>
                  <label><span class="pname">BaseSpeed</span> (ga nền)</label>
                  <input name="baseSpeed" type="number" step="1">
                  <div class="sub">Mức ga nền để treo. Tăng nếu “tụt” khi tải nặng.</div>
                </div>
                <div>
                  <label><span class="pname">range_altitude</span> (biên độ cao độ)</label>
                  <input name="range_altitude" type="number" step="0.1">
                  <div class="sub">Biên độ/độ nhạy vòng cao độ (tuỳ thuật toán). Quá lớn có thể dễ dao động.</div>
                </div>
              </div>
            </div>

            <div class="section">
              <h4>FeedForward (FF)</h4>
              <div class="hint">FF giúp phản ứng “đi trước” thay vì chỉ dựa vào PID. Tăng FF nếu bị trễ khi thay đổi nhanh.</div>
              <div class="fields">
                <div><label><span class="pname">kFF_Roll</span></label><input name="kff_roll" type="number" step="0.01"></div>
                <div><label><span class="pname">kFF_Pitch</span></label><input name="kff_pitch" type="number" step="0.01"></div>
                <div><label><span class="pname">kFF_Yaw</span></label><input name="kff_yaw" type="number" step="0.01"></div>
                <div><label><span class="pname">kFF_Alt</span></label><input name="kff_altitude" type="number" step="0.01"></div>
              </div>
            </div>
          </div>

          <div class="section">
            <div class="row">
              <button class="btn primary" type="submit">Lưu cài đặt</button>
              <a class="btn" href="/telemetry_ui" style="text-decoration:none;display:inline-flex;align-items:center">Telemetry UI</a>
            </div>
          </div>
        </form>
      </div>
      </div>

      <div class="right">
        <div class="rightStack">
          <div class="card" id="telemetryCard">
            <h3>Telemetry (live)</h3>
            <div class="teleGrid">
              <div class="teleItem"><div class="k">Target Roll</div><div class="v"><span id="targetRoll">0</span></div></div>
              <div class="teleItem"><div class="k">Target Pitch</div><div class="v"><span id="targetPitch">0</span></div></div>
              <div class="teleItem"><div class="k">Target Yaw</div><div class="v"><span id="targetYaw">0</span></div></div>
              <div class="teleItem"><div class="k">Alt Rate Target</div><div class="v"><span id="altRateTarget">0</span></div></div>

              <div class="teleItem"><div class="k">Current Roll</div><div class="v"><span id="currentRoll">0</span></div></div>
              <div class="teleItem"><div class="k">Current Pitch</div><div class="v"><span id="currentPitch">0</span></div></div>
              <div class="teleItem"><div class="k">Current Yaw</div><div class="v"><span id="currentYaw">0</span></div></div>
              <div class="teleItem"><div class="k">Altitude</div><div class="v"><span id="currentAltitude">0</span></div></div>
            </div>
            <div class="hint" style="margin-top:10px">
              Nếu drone đang ARMED/FLYING: trang tuning sẽ bị khóa (để tránh save nhầm). Telemetry UI vẫn xem được.
            </div>
          </div>

          <div class="card" id="calibCard">
            <h3>Calibration</h3>
            <div class="hint">Chỉ calib khi để drone trên mặt phẳng, không ARMED. Calib Mag cần xoay cảm biến đủ mọi hướng.</div>
            <div class="row" style="margin-top:10px">
              <button class="btn" type="button" id="btnCalibAG">Calib Accel/Gyro</button>
              <button class="btn" type="button" id="btnCalibMag">Calib Mag</button>
              <button class="btn danger" type="button" id="btnResetCalib">Reset Calib</button>
            </div>
            <div class="hint" style="margin-top:10px">Status: <span class="mono" id="calibStatus">-</span></div>
          </div>
        </div>
      </div>
    </div>
  </div>


<script>
  const msgBox = document.getElementById('msg');
  const DEFAULTS = {{DEFAULTS}};

  function applyDefaults(cfg){
    try{
      for(const [k,v] of Object.entries(cfg||{})){
        const el = document.querySelector('[name="'+k+'"]');
        if(el && (el.value===undefined || el.value===null || el.value==='')) el.value = v;
      }
    }catch(e){}
  }

  function showMsg(t, ok=true){
    if(!t){ msgBox.style.display='none'; return; }
    msgBox.style.display='block';
    msgBox.textContent=t;
    msgBox.style.borderColor = ok ? 'rgba(46,229,157,.55)' : 'rgba(255,77,109,.55)';
  }

  // Tabs
  const tabBtns = [...document.querySelectorAll('.tabbtn')];
  const tabs = {
    pid: document.getElementById('tab-pid'),
    stability: document.getElementById('tab-stability'),
    speed: document.getElementById('tab-speed'),
    load: document.getElementById('tab-load'),
  };
  function setTab(key){
    Object.keys(tabs).forEach(k=>{
      tabs[k].classList.toggle('hide', k!==key);
    });
    tabBtns.forEach(b=>b.classList.toggle('active', b.dataset.tab===key));
  }
  tabBtns.forEach(b=>b.addEventListener('click', ()=>setTab(b.dataset.tab)));

  async function loadConfig(){
    try{
      const r = await fetch('/config', {cache:'no-store'});
      if(!r.ok) throw new Error('HTTP ' + r.status);
      const cfg = await r.json();
      for(const [k,v] of Object.entries(cfg)){
        const el = document.querySelector('[name="'+k+'"]');
        if(el) el.value = v;
      }
    }catch(e){
      showMsg('⚠️ loadConfig error: ' + e, false);
    }
  }

  async function pollTelemetry(){
    try{
      const r = await fetch('/telemetry', {cache:'no-store'});
      if(!r.ok) return;
      const t = await r.json();
      const set=(id,val)=>{ const el=document.getElementById(id); if(el) el.textContent=Number(val).toFixed(2); };
      set('targetRoll', t.targetRoll);
      set('targetPitch', t.targetPitch);
      set('targetYaw', t.targetYaw);
      set('altRateTarget', t.altitude_rate_target);
      set('currentRoll', t.currentRoll);
      set('currentPitch', t.currentPitch);
      set('currentYaw', t.currentYaw);
      set('currentAltitude', t.currentAltitude);
    }catch(e){}
  }

  async function pollCalib(){
    try{
      const r = await fetch('/calibrationStatus', {cache:'no-store'});
      if(r.ok){
        document.getElementById('calibStatus').textContent = await r.text();
      }
    }catch(e){}
  }

  async function post(path){
    try{
      const r = await fetch(path, {method:'POST'});
      const t = await r.text();
      showMsg(t, r.ok);
    }catch(e){
      showMsg('⚠️ ' + path + ' error: ' + e, false);
    }
  }

  const _btnAG = document.getElementById('btnCalibAG');
  if(_btnAG) _btnAG.addEventListener('click', ()=>post('/calibrateAccelGyro'));
  const _btnMag = document.getElementById('btnCalibMag');
  if(_btnMag) _btnMag.addEventListener('click', ()=>post('/calibrateMag'));
  const _btnReset = document.getElementById('btnResetCalib');
  if(_btnReset) _btnReset.addEventListener('click', ()=>post('/resetCalibration'));

  window.addEventListener('load', ()=>{
    const initMsg = msgBox.textContent.trim();
    if(initMsg) showMsg(initMsg, !initMsg.toLowerCase().includes('error'));
    applyDefaults(DEFAULTS);
    loadConfig();
    pollTelemetry();
    pollCalib();
    setInterval(pollTelemetry, 400);
    setInterval(pollCalib, 700);
  });
</script>

</body>
</html>
)HTML";

  html.replace("{{MAC}}", mac);
  html.replace("{{MSG}}", String(message));

  // Embed defaults so UI shows values immediately even if /config fetch is delayed
  String defaults = "{";
  defaults += "\"pRoll_rate\":" + String(pidRoll_rate.P, 6) + ",";
  defaults += "\"iRoll_rate\":" + String(pidRoll_rate.I, 6) + ",";
  defaults += "\"dRoll_rate\":" + String(pidRoll_rate.D, 6) + ",";
  defaults += "\"pPitch_rate\":" + String(pidPitch_rate.P, 6) + ",";
  defaults += "\"iPitch_rate\":" + String(pidPitch_rate.I, 6) + ",";
  defaults += "\"dPitch_rate\":" + String(pidPitch_rate.D, 6) + ",";
  defaults += "\"pYaw_rate\":" + String(pidYaw_rate.P, 6) + ",";
  defaults += "\"iYaw_rate\":" + String(pidYaw_rate.I, 6) + ",";
  defaults += "\"dYaw_rate\":" + String(pidYaw_rate.D, 6) + ",";

  defaults += "\"pRoll_angle\":" + String(pidRoll_angle.P, 6) + ",";
  defaults += "\"iRoll_angle\":" + String(pidRoll_angle.I, 6) + ",";
  defaults += "\"dRoll_angle\":" + String(pidRoll_angle.D, 6) + ",";
  defaults += "\"pPitch_angle\":" + String(pidPitch_angle.P, 6) + ",";
  defaults += "\"iPitch_angle\":" + String(pidPitch_angle.I, 6) + ",";
  defaults += "\"dPitch_angle\":" + String(pidPitch_angle.D, 6) + ",";
  defaults += "\"pYaw_angle\":" + String(pidYaw_angle.P, 6) + ",";
  defaults += "\"iYaw_angle\":" + String(pidYaw_angle.I, 6) + ",";
  defaults += "\"dYaw_angle\":" + String(pidYaw_angle.D, 6) + ",";

  defaults += "\"pAltitude_rate\":" + String(pidAltitude_rate.P, 6) + ",";
  defaults += "\"iAltitude_rate\":" + String(pidAltitude_rate.I, 6) + ",";
  defaults += "\"dAltitude_rate\":" + String(pidAltitude_rate.D, 6) + ",";
  defaults += "\"pAltitude_m\":" + String(pidAltitude_m.P, 6) + ",";
  defaults += "\"iAltitude_m\":" + String(pidAltitude_m.I, 6) + ",";
  defaults += "\"dAltitude_m\":" + String(pidAltitude_m.D, 6) + ",";

  defaults += "\"trimRoll\":" + String(trimRoll, 3) + ",";
  defaults += "\"trimPitch\":" + String(trimPitch, 3) + ",";
  defaults += "\"trimYaw\":" + String(trimYaw, 3) + ",";
  defaults += "\"trimAltitude\":" + String(trimAltitude, 3) + ",";
  defaults += "\"integralLimit\":" + String(integralLimit) + ",";
  defaults += "\"maxRateChange\":" + String(maxRateChange, 3) + ",";
  defaults += "\"alpha\":" + String(alpha, 3) + ",";
  defaults += "\"beta\":" + String(beta, 3) + ",";

  defaults += "\"maxAngleDeg\":" + String(maxAngleDeg, 2) + ",";
  defaults += "\"maxYawRateDegS\":" + String(maxYawRateDegS, 1) + ",";
  defaults += "\"maxAltRateMps\":" + String(maxAltRateMps, 2) + ",";
  defaults += "\"rpResponse\":" + String(rpResponse, 2) + ",";
  defaults += "\"yawResponse\":" + String(yawResponse, 2) + ",";
  defaults += "\"altResponse\":" + String(altResponse, 2) + ",";
  defaults += "\"rpExpo\":" + String(rpExpo, 2) + ",";
  defaults += "\"yawExpo\":" + String(yawExpo, 2) + ",";
  defaults += "\"altExpo\":" + String(altExpo, 2) + ",";

  defaults += "\"range_altitude\":" + String(range_altitude, 2) + ",";
  defaults += "\"kff_roll\":" + String(kff_roll, 2) + ",";
  defaults += "\"kff_pitch\":" + String(kff_pitch, 2) + ",";
  defaults += "\"kff_yaw\":" + String(kff_yaw, 2) + ",";
  defaults += "\"kff_altitude\":" + String(kff_altitude, 2) + ",";
  defaults += "\"baseSpeed\":" + String(baseSpeed);
  defaults += "}";

  html.replace("{{DEFAULTS}}", defaults);
  return html;
}

String telemetryViewHTML() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang="en">
  <head>
    <meta charset="UTF-8">
    <title>MEBlock Drone Telemetry V1</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <style>
      body { margin:0; font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; background:#0e2a52; color:#ffffff; }
      .wrap { max-width: 720px; margin: 0 auto; padding: 14px; }
      .top { display:flex; align-items:center; justify-content:space-between; gap:12px; }
      .title { font-size:18px; font-weight:700; letter-spacing:0.2px; }
      .badge { font-size:12px; padding:6px 10px; border-radius:999px; background:#123a6b; border:1px solid #2e5b99; }
      .grid { margin-top: 12px; display:grid; grid-template-columns: 1fr 1fr; gap: 10px; }
      .card { background:#123a6b; border:1px solid #2e5b99; border-radius: 12px; padding: 12px; }
      .label { font-size:12px; color:#b7b7c6; margin-bottom:6px; }
      .val { font-size:22px; font-weight:800; }
      .unit { font-size:12px; color:#b7b7c6; margin-left: 6px; font-weight: 600; }
      .hint { margin-top: 12px; font-size: 12px; color:#a7a7b6; line-height:1.35; }
      a { color:#6dd6ff; text-decoration:none; }
      a:active, a:hover { text-decoration: underline; }
      .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
    </style>
  </head>
  <body>
    <div class="wrap">
      <div class="top">
        <div class="title">MEBlock Drone Telemetry V1</div>
        <div class="badge mono" id="status">...</div>
      </div>

      <div class="grid">
        <div class="card">
          <div class="label">Target Roll</div>
          <div class="val"><span id="targetRoll_v">0</span><span class="unit">deg</span></div>
        </div>
        <div class="card">
          <div class="label">Current Roll</div>
          <div class="val"><span id="currentRoll_v">0</span><span class="unit">deg</span></div>
        </div>

        <div class="card">
          <div class="label">Target Pitch</div>
          <div class="val"><span id="targetPitch_v">0</span><span class="unit">deg</span></div>
        </div>
        <div class="card">
          <div class="label">Current Pitch</div>
          <div class="val"><span id="currentPitch_v">0</span><span class="unit">deg</span></div>
        </div>

        <div class="card">
          <div class="label">Target Yaw</div>
          <div class="val"><span id="targetYaw_v">0</span><span class="unit">deg</span></div>
        </div>
        <div class="card">
          <div class="label">Current Yaw</div>
          <div class="val"><span id="currentYaw_v">0</span><span class="unit">deg</span></div>
        </div>

        <div class="card">
          <div class="label">Altitude Rate Target</div>
          <div class="val"><span id="altRate_v">0</span><span class="unit">m/s</span></div>
        </div>
        <div class="card">
          <div class="label">Current Altitude</div>
          <div class="val"><span id="alt_v">0</span><span class="unit">m</span></div>
        </div>
      </div>

      <div class="hint">
        • Khi đang bay, UI tuning sẽ tự chuyển sang trang này để giảm tải. <br>
        • Nếu đang <span class="mono">DISARMED</span> bạn có thể mở tuning tại <a href="/tuning">/tuning</a>.
      </div>
    </div>

    <script>
      const $ = (id) => document.getElementById(id);

      function fmt(x, d=2) {
        if (typeof x !== 'number' || isNaN(x)) return '0';
        return x.toFixed(d);
      }

      async function tick() {
        try {
          const r = await fetch('/telemetry', { cache: 'no-store' });
          const d = await r.json();

          const st = (d.armed ? 'ARMED' : 'DISARMED') + (d.onFlying ? ' • FLYING' : '');
          $('status').textContent = st;

          $('targetRoll_v').textContent   = fmt(d.targetRoll, 2);
          $('currentRoll_v').textContent  = fmt(d.currentRoll, 2);
          $('targetPitch_v').textContent  = fmt(d.targetPitch, 2);
          $('currentPitch_v').textContent = fmt(d.currentPitch, 2);
          $('targetYaw_v').textContent    = fmt(d.targetYaw, 2);
          $('currentYaw_v').textContent   = fmt(d.currentYaw, 2);
          $('altRate_v').textContent      = fmt(d.altitude_rate_target, 2);
          $('alt_v').textContent          = fmt(d.currentAltitude, 2);
        } catch (e) {
          // ignore
        }
      }

      tick();
      setInterval(tick, 100); // 10Hz (nhẹ hơn tuning page)
    </script>
  </body>
  </html>
)rawliteral";
  return html;
}

// *************************************************************  Functions  ************************************************************* //

void printConfigToSerial() {
  Serial.println("=== Current Config ===");
  Serial.printf("PID Roll_rate: P=%.3f, I=%.3f, D=%.3f\n", pidRoll_rate.P, pidRoll_rate.I, pidRoll_rate.D);
  Serial.printf("PID Pitch_rate: P=%.3f, I=%.3f, D=%.3f\n", pidPitch_rate.P, pidPitch_rate.I, pidPitch_rate.D);
  Serial.printf("PID Yaw_rate: P=%.3f, I=%.3f, D=%.3f\n", pidYaw_rate.P, pidYaw_rate.I, pidYaw_rate.D);
  Serial.printf("PID Altitude_rate  : P=%.3f, I=%.3f, D=%.3f\n", pidAltitude_rate.P, pidAltitude_rate.I, pidAltitude_rate.D);

  Serial.printf("PID Roll_angle: P=%.3f, I=%.3f, D=%.3f\n", pidRoll_angle.P, pidRoll_angle.I, pidRoll_angle.D);
  Serial.printf("PID Pitch_angle: P=%.3f, I=%.3f, D=%.3f\n", pidPitch_angle.P, pidPitch_angle.I, pidPitch_angle.D);
  Serial.printf("PID Yaw_angle: P=%.3f, I=%.3f, D=%.3f\n", pidYaw_angle.P, pidYaw_angle.I, pidYaw_angle.D);
  Serial.printf("PID Altitude_m  : P=%.3f, I=%.3f, D=%.3f\n", pidAltitude_m.P, pidAltitude_m.I, pidAltitude_m.D);

  Serial.printf("Trim: Roll=%.3f, Pitch=%.3f, Yaw=%.3f, Altitude=%.3f\n", trimRoll, trimPitch, trimYaw, trimAltitude);
  Serial.printf("Base Speed: %d\n", baseSpeed);
  Serial.println("======================");
}

void loadConfig() {
  preferences.begin("drone", true);

  pidRoll_rate.P = preferences.getFloat("pRoll_rate", rollKp_rate);
  pidRoll_rate.I = preferences.getFloat("iRoll_rate", rollKi_rate);
  pidRoll_rate.D = preferences.getFloat("dRoll_rate", rollKd_rate);
  pidPitch_rate.P = preferences.getFloat("pPitch_rate", pitchKp_rate);
  pidPitch_rate.I = preferences.getFloat("iPitch_rate", pitchKi_rate);
  pidPitch_rate.D = preferences.getFloat("dPitch_rate", pitchKd_rate);
  pidYaw_rate.P = preferences.getFloat("pYaw_rate", yawKp_rate);
  pidYaw_rate.I = preferences.getFloat("iYaw_rate", yawKi_rate);
  pidYaw_rate.D = preferences.getFloat("dYaw_rate", yawKd_rate);
  pidAltitude_rate.P = preferences.getFloat("pAltitude_rate", altitudeKp_rate);
  pidAltitude_rate.I = preferences.getFloat("iAltitude_rate", altitudeKi_rate);
  pidAltitude_rate.D = preferences.getFloat("dAltitude_rate", altitudeKd_rate);

  pidRoll_angle.P = preferences.getFloat("pRoll_angle", rollKp_angle);
  pidRoll_angle.I = preferences.getFloat("iRoll_angle", rollKi_angle);
  pidRoll_angle.D = preferences.getFloat("dRoll_angle", rollKd_angle);
  pidPitch_angle.P = preferences.getFloat("pPitch_angle", pitchKp_angle);
  pidPitch_angle.I = preferences.getFloat("iPitch_angle", pitchKi_angle);
  pidPitch_angle.D = preferences.getFloat("dPitch_angle", pitchKd_angle);
  pidYaw_angle.P = preferences.getFloat("pYaw_angle", yawKp_angle);
  pidYaw_angle.I = preferences.getFloat("iYaw_angle", yawKi_angle);
  pidYaw_angle.D = preferences.getFloat("dYaw_angle", yawKd_angle);
  pidAltitude_m.P = preferences.getFloat("pAltitude_m", altitudeKp_m);
  pidAltitude_m.I = preferences.getFloat("iAltitude_m", altitudeKi_m);
  pidAltitude_m.D = preferences.getFloat("dAltitude_m", altitudeKd_m);

  trimRoll = preferences.getFloat("trimRoll", 0.0);
  trimPitch = preferences.getFloat("trimPitch", 0.0);
  trimYaw = preferences.getFloat("trimYaw", 0.0);
  trimAltitude = preferences.getFloat("trimAltitude", 0.0);
  baseSpeed = preferences.getInt("baseSpeed", baseSpeed);
  integralLimit = preferences.getInt("integralLimit", integralLimit);
  maxRateChange = preferences.getFloat("maxRateChange", maxRateChange);
  alpha = preferences.getFloat("alpha", alpha);
  beta  = preferences.getFloat("beta", beta);
  range_altitude = preferences.getFloat("range_altitude", range_altitude);
  kff_roll = preferences.getFloat("kff_roll", kff_roll);
  kff_pitch = preferences.getFloat("kff_pitch", kff_pitch);
  kff_yaw = preferences.getFloat("kff_yaw", kff_yaw);
  kff_altitude = preferences.getFloat("kff_altitude", kff_altitude);
  maxAngleDeg = preferences.getFloat("maxAngleDeg", maxAngleDeg);
  maxYawRateDegS = preferences.getFloat("maxYawRateDegS", maxYawRateDegS);
  maxAltRateMps = preferences.getFloat("maxAltRateMps", maxAltRateMps);
  rpResponse = preferences.getFloat("rpResponse", rpResponse);
  yawResponse = preferences.getFloat("yawResponse", yawResponse);
  altResponse = preferences.getFloat("altResponse", altResponse);
  rpExpo = preferences.getFloat("rpExpo", rpExpo);
  yawExpo = preferences.getFloat("yawExpo", yawExpo);
  altExpo = preferences.getFloat("altExpo", altExpo);
  preferences.end();
}

void saveConfig() {
  preferences.begin("drone", false);

  preferences.putFloat("pRoll_rate", pidRoll_rate.P);
  preferences.putFloat("iRoll_rate", pidRoll_rate.I);
  preferences.putFloat("dRoll_rate", pidRoll_rate.D);
  preferences.putFloat("pPitch_rate", pidPitch_rate.P);
  preferences.putFloat("iPitch_rate", pidPitch_rate.I);
  preferences.putFloat("dPitch_rate", pidPitch_rate.D);
  preferences.putFloat("pYaw_rate", pidYaw_rate.P);
  preferences.putFloat("iYaw_rate", pidYaw_rate.I);
  preferences.putFloat("dYaw_rate", pidYaw_rate.D);
  preferences.putFloat("pAltitude_rate", pidAltitude_rate.P);
  preferences.putFloat("iAltitude_rate", pidAltitude_rate.I);
  preferences.putFloat("dAltitude_rate", pidAltitude_rate.D);

  preferences.putFloat("pRoll_angle", pidRoll_angle.P);
  preferences.putFloat("iRoll_angle", pidRoll_angle.I);
  preferences.putFloat("dRoll_angle", pidRoll_angle.D);
  preferences.putFloat("pPitch_angle", pidPitch_angle.P);
  preferences.putFloat("iPitch_angle", pidPitch_angle.I);
  preferences.putFloat("dPitch_angle", pidPitch_angle.D);
  preferences.putFloat("pYaw_angle", pidYaw_angle.P);
  preferences.putFloat("iYaw_angle", pidYaw_angle.I);
  preferences.putFloat("dYaw_angle", pidYaw_angle.D);
  preferences.putFloat("pAltitude_m", pidAltitude_m.P);
  preferences.putFloat("iAltitude_m", pidAltitude_m.I);
  preferences.putFloat("dAltitude_m", pidAltitude_m.D);

  preferences.putFloat("trimRoll", trimRoll);
  preferences.putFloat("trimPitch", trimPitch);
  preferences.putFloat("trimYaw", trimYaw);
  preferences.putFloat("trimAltitude", trimAltitude);
  preferences.putInt("baseSpeed", baseSpeed);
  preferences.putInt("integralLimit", integralLimit);
  preferences.putFloat("maxRateChange", maxRateChange);
  preferences.putFloat("alpha", alpha);
  preferences.putFloat("beta", beta);
  preferences.putFloat("range_altitude", range_altitude);
  preferences.putFloat("kff_roll", kff_roll);
  preferences.putFloat("kff_pitch", kff_pitch);
  preferences.putFloat("kff_yaw", kff_yaw);
  preferences.putFloat("kff_altitude", kff_altitude);
  preferences.putFloat("maxAngleDeg", maxAngleDeg);
  preferences.putFloat("maxYawRateDegS", maxYawRateDegS);
  preferences.putFloat("maxAltRateMps", maxAltRateMps);
  preferences.putFloat("rpResponse", rpResponse);
  preferences.putFloat("yawResponse", yawResponse);
  preferences.putFloat("altResponse", altResponse);
  preferences.putFloat("rpExpo", rpExpo);
  preferences.putFloat("yawExpo", yawExpo);
  preferences.putFloat("altExpo", altExpo);

  preferences.end();
}

void calibrateAccelGyro() {

    myMPU9250.autoOffsets();

    xyzFloat aOffs = myMPU9250.getAccOffsets();  // get acceleration offsets
    xyzFloat gOffs = myMPU9250.getGyrOffsets();  // get gyroscope offsets 

    ax_bias = aOffs.x;
    ay_bias = aOffs.y;
    az_bias = aOffs.z;
    gx_bias = gOffs.x;
    gy_bias = gOffs.y;
    gz_bias = gOffs.z;
}

void calibrateMag(int samples = 6000) {
    float mx_min = 32767, my_min = 32767, mz_min = 32767;
    float mx_max = -32768, my_max = -32768, mz_max = -32768;

    Serial.println("Rotate the sensor in all directions for magnetometer calibration...");
    delay(200);

    for(int i=0; i<samples; i++){
        xyzFloat mag = myMPU9250.getMagValues();
        if(mag.x < mx_min) mx_min = mag.x;
        if(mag.x > mx_max) mx_max = mag.x;
        if(mag.y < my_min) my_min = mag.y;
        if(mag.y > my_max) my_max = mag.y;
        if(mag.z < mz_min) mz_min = mag.z;
        if(mag.z > mz_max) mz_max = mag.z;
        delay(5);
    }

    // Hard iron offset
    mx_bias = (mx_max + mx_min)/2;
    my_bias = (my_max + my_min)/2;
    mz_bias = (mz_max + mz_min)/2;

    // Soft iron scale
    mx_scale = (mx_max - mx_min)/2;
    my_scale = (my_max - my_min)/2;
    mz_scale = (mz_max - mz_min)/2;

    Serial.println("Magnetometer calibrated.");
}

void saveCalibration() {
    preferences.begin("mpu-calib", false);
    preferences.putFloat("accX", ax_bias);
    preferences.putFloat("accY", ay_bias);
    preferences.putFloat("accZ", az_bias);

    preferences.putFloat("gyroX", gx_bias);
    preferences.putFloat("gyroY", gy_bias);
    preferences.putFloat("gyroZ", gz_bias);

    preferences.putFloat("magX", mx_bias);
    preferences.putFloat("magY", my_bias);
    preferences.putFloat("magZ", mz_bias);

    preferences.putFloat("magScaleX", mx_scale);
    preferences.putFloat("magScaleY", my_scale);
    preferences.putFloat("magScaleZ", mz_scale);

    preferences.putBool("calibrated", true);
    preferences.end();

    Serial.println("Calibration saved.");
}

void loadCalibration() {
    preferences.begin("mpu-calib", true);
    bool calibrated = preferences.getBool("calibrated", false);
    if(calibrated){

        xyzFloat accOffset, gyrOffset;

        accOffset.x = preferences.getFloat("accX", 0.0f);
        accOffset.y = preferences.getFloat("accY", 0.0f);
        accOffset.z = preferences.getFloat("accZ", 0.0f);

        gyrOffset.x = preferences.getFloat("gyroX", 0.0f);
        gyrOffset.y = preferences.getFloat("gyroY", 0.0f);
        gyrOffset.z = preferences.getFloat("gyroZ", 0.0f);

        mx_bias = preferences.getFloat("magX",0);
        my_bias = preferences.getFloat("magY",0);
        mz_bias = preferences.getFloat("magZ",0);

        mx_scale = preferences.getFloat("magScaleX",1);
        my_scale = preferences.getFloat("magScaleY",1);
        mz_scale = preferences.getFloat("magScaleZ",1);

        myMPU9250.setAccOffsets(accOffset);
        myMPU9250.setGyrOffsets(gyrOffset);

        Serial.println("Calibration loaded.");
    } else {
        Serial.println("No calibration found.");
    }
    preferences.end();
}

void resetCalibration() {
    ax_bias = ay_bias = az_bias = 0;
    gx_bias = gy_bias = gz_bias = 0;
    mx_bias = my_bias = mz_bias = 0;
    mx_scale = my_scale = mz_scale = 1;

    preferences.begin("mpu-calib", false);
    preferences.clear();
    preferences.end();

    Serial.println("Calibration reset.");
}

void resetPID() {
  // ===== Inner PID (rate) =====
  rollIntegral_rate = 0.0;
  rollPrevError_rate = 0.0;
  rollOutput_rate = 0.0;

  pitchIntegral_rate = 0.0;
  pitchPrevError_rate = 0.0;
  pitchOutput_rate = 0.0;

  yawIntegral_rate = 0.0;
  yawPrevError_rate = 0.0;
  yawOutput_rate = 0.0;

  altitudeIntegral_rate = 0.0;
  altitudePrevError_rate = 0.0;
  altitudeOutput_rate = 0.0;

  // ===== Outer PID (angle / altitude) =====
  rollIntegral_angle = 0.0;
  rollPrevError_angle = 0.0;
  rollOutput_angle = 0.0;

  pitchIntegral_angle = 0.0;
  pitchPrevError_angle = 0.0;
  pitchOutput_angle = 0.0;

  yawIntegral_angle = 0.0;
  yawPrevError_angle = 0.0;
  yawOutput_angle = 0.0;

  altitudeIntegral_m = 0.0;
  altitudePrevError_m = 0.0;
  altitudeOutput_m = 0.0;
}

void startWebServer() {
  Serial.println("Starting Async Web Server...");
  if (!webServerRunning) {
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(g_apSsid, g_apPass);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    dnsServer.start(DNS_PORT, "*", apIP);

    // Register routes only once (avoid duplicates if start/stop many times)
    if (!routesRegistered) {
      routesRegistered = true;

      // Root: auto switch (FLYING => telemetry, DISARMED => tuning)
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (Armed || OnFlying) {
          request->send(200, "text/html", telemetryViewHTML());
        } else {
          request->send(200, "text/html", htmlForm());
        }
      });

      // Explicit pages
      server.on("/tuning", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (Armed || OnFlying) {
          request->redirect("/telemetry_ui");
          return;
        }
        request->send(200, "text/html", htmlForm());
      });

      server.on("/telemetry_ui", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", telemetryViewHTML());
      });

      // Telemetry JSON (used by both pages)
      server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"armed\":" + String(Armed ? 1 : 0) + ",";
        json += "\"onFlying\":" + String(OnFlying ? 1 : 0) + ",";
        json += "\"targetRoll\":" + String(targetRoll) + ",";
        json += "\"targetPitch\":" + String(targetPitch) + ",";
        json += "\"targetYaw\":" + String(targetYaw) + ",";
        json += "\"altitude_rate_target\":" + String(altitude_rate_target) + ",";
        json += "\"currentRoll\":" + String(currentRoll) + ",";
        json += "\"currentPitch\":" + String(currentPitch) + ",";
        json += "\"currentYaw\":" + String(currentYaw) + ",";
        json += "\"currentAltitude\":" + String(currentAltitude);
        json += "}";
        request->send(200, "application/json", json);
      });
      // Current tuning config (used by /tuning UI)
      server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"pRoll_rate\":" + String(pidRoll_rate.P, 6) + ",";
        json += "\"iRoll_rate\":" + String(pidRoll_rate.I, 6) + ",";
        json += "\"dRoll_rate\":" + String(pidRoll_rate.D, 6) + ",";
        json += "\"pPitch_rate\":" + String(pidPitch_rate.P, 6) + ",";
        json += "\"iPitch_rate\":" + String(pidPitch_rate.I, 6) + ",";
        json += "\"dPitch_rate\":" + String(pidPitch_rate.D, 6) + ",";
        json += "\"pYaw_rate\":" + String(pidYaw_rate.P, 6) + ",";
        json += "\"iYaw_rate\":" + String(pidYaw_rate.I, 6) + ",";
        json += "\"dYaw_rate\":" + String(pidYaw_rate.D, 6) + ",";

        json += "\"pRoll_angle\":" + String(pidRoll_angle.P, 6) + ",";
        json += "\"iRoll_angle\":" + String(pidRoll_angle.I, 6) + ",";
        json += "\"dRoll_angle\":" + String(pidRoll_angle.D, 6) + ",";
        json += "\"pPitch_angle\":" + String(pidPitch_angle.P, 6) + ",";
        json += "\"iPitch_angle\":" + String(pidPitch_angle.I, 6) + ",";
        json += "\"dPitch_angle\":" + String(pidPitch_angle.D, 6) + ",";
        json += "\"pYaw_angle\":" + String(pidYaw_angle.P, 6) + ",";
        json += "\"iYaw_angle\":" + String(pidYaw_angle.I, 6) + ",";
        json += "\"dYaw_angle\":" + String(pidYaw_angle.D, 6) + ",";

        json += "\"pAltitude_rate\":" + String(pidAltitude_rate.P, 6) + ",";
        json += "\"iAltitude_rate\":" + String(pidAltitude_rate.I, 6) + ",";
        json += "\"dAltitude_rate\":" + String(pidAltitude_rate.D, 6) + ",";
        json += "\"pAltitude_m\":" + String(pidAltitude_m.P, 6) + ",";
        json += "\"iAltitude_m\":" + String(pidAltitude_m.I, 6) + ",";
        json += "\"dAltitude_m\":" + String(pidAltitude_m.D, 6) + ",";

        json += "\"trimRoll\":" + String(trimRoll, 3) + ",";
        json += "\"trimPitch\":" + String(trimPitch, 3) + ",";
        json += "\"trimYaw\":" + String(trimYaw, 3) + ",";
        json += "\"trimAltitude\":" + String(trimAltitude, 3) + ",";
        json += "\"integralLimit\":" + String(integralLimit) + ",";
        json += "\"maxRateChange\":" + String(maxRateChange, 3) + ",";
        json += "\"alpha\":" + String(alpha, 3) + ",";
        json += "\"beta\":" + String(beta, 3) + ",";
        json += "\"maxAngleDeg\":" + String(maxAngleDeg, 2) + ",";
        json += "\"maxYawRateDegS\":" + String(maxYawRateDegS, 1) + ",";
        json += "\"maxAltRateMps\":" + String(maxAltRateMps, 2) + ",";
        json += "\"rpResponse\":" + String(rpResponse, 2) + ",";
        json += "\"yawResponse\":" + String(yawResponse, 2) + ",";
        json += "\"altResponse\":" + String(altResponse, 2) + ",";
        json += "\"rpExpo\":" + String(rpExpo, 2) + ",";
        json += "\"yawExpo\":" + String(yawExpo, 2) + ",";
        json += "\"altExpo\":" + String(altExpo, 2) + ",";
        json += "\"range_altitude\":" + String(range_altitude, 2) + ",";
        json += "\"kff_roll\":" + String(kff_roll, 2) + ",";
        json += "\"kff_pitch\":" + String(kff_pitch, 2) + ",";
        json += "\"kff_yaw\":" + String(kff_yaw, 2) + ",";
        json += "\"kff_altitude\":" + String(kff_altitude, 2) + ",";
        json += "\"baseSpeed\":" + String(baseSpeed);
        json += "}";
        request->send(200, "application/json", json);
      });


      // ---------------------- TUNING / CALIBRATION (LOCKED while ARMED/FLYING) ----------------------
      server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (Armed || OnFlying) {
          request->send(423, "text/plain", "Locked while ARMED/FLYING");
          return;
        }

        if (request->hasParam("pRoll_rate", true)) pidRoll_rate.P = request->getParam("pRoll_rate", true)->value().toFloat();
        if (request->hasParam("iRoll_rate", true)) pidRoll_rate.I = request->getParam("iRoll_rate", true)->value().toFloat();
        if (request->hasParam("dRoll_rate", true)) pidRoll_rate.D = request->getParam("dRoll_rate", true)->value().toFloat();
        if (request->hasParam("pPitch_rate", true)) pidPitch_rate.P = request->getParam("pPitch_rate", true)->value().toFloat();
        if (request->hasParam("iPitch_rate", true)) pidPitch_rate.I = request->getParam("iPitch_rate", true)->value().toFloat();
        if (request->hasParam("dPitch_rate", true)) pidPitch_rate.D = request->getParam("dPitch_rate", true)->value().toFloat();
        if (request->hasParam("pYaw_rate", true)) pidYaw_rate.P = request->getParam("pYaw_rate", true)->value().toFloat();
        if (request->hasParam("iYaw_rate", true)) pidYaw_rate.I = request->getParam("iYaw_rate", true)->value().toFloat();
        if (request->hasParam("dYaw_rate", true)) pidYaw_rate.D = request->getParam("dYaw_rate", true)->value().toFloat();
        if (request->hasParam("pAltitude_rate", true)) pidAltitude_rate.P = request->getParam("pAltitude_rate", true)->value().toFloat();
        if (request->hasParam("iAltitude_rate", true)) pidAltitude_rate.I = request->getParam("iAltitude_rate", true)->value().toFloat();
        if (request->hasParam("dAltitude_rate", true)) pidAltitude_rate.D = request->getParam("dAltitude_rate", true)->value().toFloat();

        if (request->hasParam("pRoll_angle", true)) pidRoll_angle.P = request->getParam("pRoll_angle", true)->value().toFloat();
        if (request->hasParam("iRoll_angle", true)) pidRoll_angle.I = request->getParam("iRoll_angle", true)->value().toFloat();
        if (request->hasParam("dRoll_angle", true)) pidRoll_angle.D = request->getParam("dRoll_angle", true)->value().toFloat();
        if (request->hasParam("pPitch_angle", true)) pidPitch_angle.P = request->getParam("pPitch_angle", true)->value().toFloat();
        if (request->hasParam("iPitch_angle", true)) pidPitch_angle.I = request->getParam("iPitch_angle", true)->value().toFloat();
        if (request->hasParam("dPitch_angle", true)) pidPitch_angle.D = request->getParam("dPitch_angle", true)->value().toFloat();
        if (request->hasParam("pYaw_angle", true)) pidYaw_angle.P = request->getParam("pYaw_angle", true)->value().toFloat();
        if (request->hasParam("iYaw_angle", true)) pidYaw_angle.I = request->getParam("iYaw_angle", true)->value().toFloat();
        if (request->hasParam("dYaw_angle", true)) pidYaw_angle.D = request->getParam("dYaw_angle", true)->value().toFloat();

        if (request->hasParam("pAltitude_m", true)) pidAltitude_m.P = request->getParam("pAltitude_m", true)->value().toFloat();
        if (request->hasParam("iAltitude_m", true)) pidAltitude_m.I = request->getParam("iAltitude_m", true)->value().toFloat();
        if (request->hasParam("dAltitude_m", true)) pidAltitude_m.D = request->getParam("dAltitude_m", true)->value().toFloat();

        if (request->hasParam("trimRoll", true)) trimRoll = request->getParam("trimRoll", true)->value().toFloat();
        if (request->hasParam("trimPitch", true)) trimPitch = request->getParam("trimPitch", true)->value().toFloat();
        if (request->hasParam("trimYaw", true)) trimYaw = request->getParam("trimYaw", true)->value().toFloat();
        if (request->hasParam("trimAltitude", true)) trimAltitude = request->getParam("trimAltitude", true)->value().toFloat();

        // Stability / filters
        if (request->hasParam("integralLimit", true)) integralLimit = request->getParam("integralLimit", true)->value().toInt();
        if (request->hasParam("maxRateChange", true)) maxRateChange = request->getParam("maxRateChange", true)->value().toFloat();
        if (request->hasParam("alpha", true)) alpha = request->getParam("alpha", true)->value().toFloat();
        if (request->hasParam("beta", true)) beta = request->getParam("beta", true)->value().toFloat();

        // Speed (response / limits)
        if (request->hasParam("maxAngleDeg", true)) maxAngleDeg = request->getParam("maxAngleDeg", true)->value().toFloat();
        if (request->hasParam("maxYawRateDegS", true)) maxYawRateDegS = request->getParam("maxYawRateDegS", true)->value().toFloat();
        if (request->hasParam("maxAltRateMps", true)) maxAltRateMps = request->getParam("maxAltRateMps", true)->value().toFloat();
        if (request->hasParam("rpResponse", true)) rpResponse = request->getParam("rpResponse", true)->value().toFloat();
        if (request->hasParam("yawResponse", true)) yawResponse = request->getParam("yawResponse", true)->value().toFloat();
        if (request->hasParam("altResponse", true)) altResponse = request->getParam("altResponse", true)->value().toFloat();
        if (request->hasParam("rpExpo", true)) rpExpo = request->getParam("rpExpo", true)->value().toFloat();
        if (request->hasParam("yawExpo", true)) yawExpo = request->getParam("yawExpo", true)->value().toFloat();
        if (request->hasParam("altExpo", true)) altExpo = request->getParam("altExpo", true)->value().toFloat();

        // Load / FF
        if (request->hasParam("range_altitude", true)) range_altitude = request->getParam("range_altitude", true)->value().toFloat();
        if (request->hasParam("kff_roll", true)) kff_roll = request->getParam("kff_roll", true)->value().toFloat();
        if (request->hasParam("kff_pitch", true)) kff_pitch = request->getParam("kff_pitch", true)->value().toFloat();
        if (request->hasParam("kff_yaw", true)) kff_yaw = request->getParam("kff_yaw", true)->value().toFloat();
        if (request->hasParam("kff_altitude", true)) kff_altitude = request->getParam("kff_altitude", true)->value().toFloat();
        if (request->hasParam("baseSpeed", true)) baseSpeed = request->getParam("baseSpeed", true)->value().toInt();

        // Sanity clamps
        maxAngleDeg = constrain(maxAngleDeg, 5.0f, 70.0f);
        maxYawRateDegS = constrain(maxYawRateDegS, 10.0f, 250.0f);
        maxAltRateMps = constrain(maxAltRateMps, 0.5f, 15.0f);
        rpResponse = constrain(rpResponse, 0.2f, 2.5f);
        yawResponse = constrain(yawResponse, 0.2f, 2.5f);
        altResponse = constrain(altResponse, 0.2f, 2.5f);
        rpExpo = constrain(rpExpo, 0.0f, 1.0f);
        yawExpo = constrain(yawExpo, 0.0f, 1.0f);
        altExpo = constrain(altExpo, 0.0f, 1.0f);
        alpha = constrain(alpha, 0.0f, 1.0f);
        beta  = constrain(beta, 0.0f, 1.0f);

        saveConfig();
        printConfigToSerial();
        request->send(200, "text/html", htmlForm("Settings updated successfully!"));
      });

      server.on("/calibrateAccelGyro", HTTP_POST, [](AsyncWebServerRequest *request){
        if (Armed || OnFlying) { request->send(423, "text/plain", "Locked while ARMED/FLYING"); return; }
        calibrateAccelGyroRequested = true;
        request->send(200, "text/html", htmlForm("Accel & Gyro Calibration started. Please keep the drone still."));
      });

      server.on("/calibrateMag", HTTP_POST, [](AsyncWebServerRequest *request){
        if (Armed || OnFlying) { request->send(423, "text/plain", "Locked while ARMED/FLYING"); return; }
        calibrateMagRequested = true;
        request->send(200, "text/html", htmlForm("Magnetometer Calibration started. Please rotate the drone."));
      });

      server.on("/calibrationStatus", HTTP_GET, [](AsyncWebServerRequest *request){
        String status = "";
        if (AccelGyroisCalibrating) {
          status = "⏳ Calibrating Accel & Gyro...";
        }
        else if (AccelGyrocalibrationDone) {
          status = "✅ Accel & Gyro Calibration complete!";
          AccelGyrocalibrationDone = false;
        }
        else if (MagisCalibrating) {
          status = "⏳ Calibrating Magnetometer...";
        }
        else if (MagcalibrationDone) {
          status = "✅ Magnetometer Calibration complete!";
          MagcalibrationDone = false;
        }
        request->send(200, "text/plain", status);
      });
server.on("/resetCalibration", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (Armed || OnFlying) { request->send(423, "text/plain", "Locked while ARMED/FLYING"); return; }
        resetCalibration();
        request->send(200, "text/html", htmlForm("Calibration reset successfully!"));
      });
    }

    server.begin();
    webServerRunning = true;
    Serial.println("Async Web Server Started");
  }
}

void stopWebServer() {
  Serial.println("Stopping Async Web Server...");
  if (webServerRunning) {
    dnsServer.stop();
    server.end();
    WiFi.softAPdisconnect(true);
    webServerRunning = false;
    Serial.println("Async Web Server Stopped");
  }
}

void readJoystickData(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(incomingJoystickData)) {
    memcpy(&incomingJoystickData, data, sizeof(incomingJoystickData));
  } else {
    Serial.printf("Received invalid data size: %d bytes\n", len);
  }
}

void setupESPNow() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_err_t result = esp_now_register_recv_cb(readJoystickData);
  if (result == ESP_OK) {
    Serial.println("Receive callback registered successfully");
  } else {
    Serial.printf("Failed to register receive callback: %d\n", result);
  }
}

float angleError(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// ---------------- Median Filter ----------------
// NOTE: Tránh dùng function template trong .ino vì Arduino build system có thể tự sinh prototype lỗi.
static float medianFilterN(float input, float *buffer, size_t N, size_t &index) {
  if (N == 0) return input;
  buffer[index] = input;
  index = (index + 1) % N;

  // N trong sketch này nhỏ (<=9). Dùng mảng temp cố định để tránh VLA / template.
  const size_t MEDIAN_MAX = 9;
  float temp[MEDIAN_MAX];
  if (N > MEDIAN_MAX) N = MEDIAN_MAX;
  for (size_t i = 0; i < N; i++) temp[i] = buffer[i];

  // sort (bubble-ish) cho N nhỏ
  for (size_t i = 0; i + 1 < N; i++) {
    for (size_t j = i + 1; j < N; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[N / 2];
}

// Giữ nguyên cách gọi: medianFilter(x, gyroX_buf, gyroX_idx);
#define medianFilter(input, buf, idx) \
  medianFilterN((input), (buf), (sizeof(buf) / sizeof((buf)[0])), (idx))

// ---------------- EMA Filter ----------------
float emaFilter(float input, float &emaPrev, float alpha) {
  emaPrev = alpha * input + (1.0f - alpha) * emaPrev;
  return emaPrev;
}

static inline float expoCurve(float x, float expo) {
  // expo: 0..1, keeps sign, compresses near center
  // y = (1-e)*x + e*x^3
  return (1.0f - expo) * x + expo * x * x * x;
}

// ====== Complementary filter =====
void Complementary_Alt_Vz(float accZ, float altitudeBaro, float dt) {
    // integrate acceleration
    velocityZ += accZ * dt;
    altitude += velocityZ * dt;

    // complementary filter สำหรับ altitude
    altitude = alpha * altitude + (1 - alpha) * altitudeBaro;

    // complementary filter สำหรับ velocityZ
    float baroVz = (altitudeBaro - previousBaro) / dt;
    velocityZ = beta * velocityZ + (1 - beta) * baroVz;

    previousBaro = altitudeBaro;
}

void updateSensorsAndMadgwick(float dt) {
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr    = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();

  // Read sensor
  float accX = gValue.x;
  float accY = gValue.y;
  float accZ = gValue.z;
  float gyroX = gyr.x;
  float gyroY = gyr.y;
  float gyroZ = gyr.z;
  float magX = magValue.x - mx_bias;
  float magY = magValue.y - my_bias;
  float magZ = magValue.z - mz_bias; 

  // ----------- Median filter -------------
  float gyroX_med = medianFilter(gyroX, gyroX_buf, gyroX_idx);
  float gyroY_med = medianFilter(gyroY, gyroY_buf, gyroY_idx);
  float gyroZ_med = medianFilter(gyroZ, gyroZ_buf, gyroZ_idx);
  float accX_med  = medianFilter(accX,  accX_buf,  accX_idx);
  float accY_med  = medianFilter(accY,  accY_buf,  accY_idx);
  float accZ_med  = medianFilter(accZ,  accZ_buf,  accZ_idx);
  float magX_med  = medianFilter(magX,  magX_buf,  magX_idx);
  float magY_med  = medianFilter(magY,  magY_buf,  magY_idx);
  float magZ_med  = medianFilter(magZ,  magZ_buf,  magZ_idx);
  float alt_med  = medianFilter(baroAltitude,  alt_buf,  alt_idx);

  // ----------- EMA filter ------------------
  gyroX_filtered = emaFilter(gyroX_med, emaGyroX, alphaGyro); 
  gyroY_filtered = emaFilter(gyroY_med, emaGyroY, alphaGyro);
  gyroZ_filtered = emaFilter(gyroZ_med, emaGyroZ, alphaGyro);
  accX_filtered  = emaFilter(accX_med,  emaAccX,  alphaAcc);
  accY_filtered  = emaFilter(accY_med,  emaAccY,  alphaAcc);
  accZ_filtered  = emaFilter(accZ_med,  emaAccZ,  alphaAcc);
  magX_filtered  = emaFilter(magX_med,  emaMagX,  alphaMag);
  magY_filtered  = emaFilter(magY_med,  emaMagY,  alphaMag);
  magZ_filtered  = emaFilter(magZ_med,  emaMagZ,  alphaMag);
  alt_filtered  = emaFilter(alt_med,  emaAlt,  alphaAlt);

  MadgwickFilter.updateIMU(gyroX_filtered, gyroY_filtered, gyroZ_filtered,
                             accX_filtered, accY_filtered, accZ_filtered);

  currentRoll  = -MadgwickFilter.getRoll();
  currentPitch =  MadgwickFilter.getPitch();
  currentYaw   = 360 - MadgwickFilter.getYaw();

  // g -> m/s^2
  float accX_ms2 = accX_filtered * g; 
  float accY_ms2 = accY_filtered * g; 
  float accZ_ms2 = accZ_filtered * g; 

  // deg -> rad
  float roll  = MadgwickFilter.getRoll()  * DEG_TO_RAD;
  float pitch = MadgwickFilter.getPitch() * DEG_TO_RAD;
  float yaw   = MadgwickFilter.getYaw()   * DEG_TO_RAD;

  // transform body -> world
  float accZ_world = (-accX_ms2 * sin(pitch))
                      + (accY_ms2 * sin(roll) * cos(pitch))
                      + (accZ_ms2 * cos(roll) * cos(pitch));

  // vertical acceleration
  float accZ_true = accZ_world - g;

  Complementary_Alt_Vz(accZ_true, baroAltitude, innerDt);
  
  currentAltitude = altitude;
  velocityZ  = medianFilter(velocityZ,  vz_buf,  vz_idx);
  velocityZ  = emaFilter(velocityZ,  emaVZ,  alphaVZ);
}

void updateParameters(float dt) {
  // Map RC command → desired angles/rates (Speed tab)
  float inRoll  = incomingJoystickData.XR;
  float inPitch = incomingJoystickData.YR;
  float inYaw   = incomingJoystickData.XL;

  // Normalize to -1..1 using current limits, apply expo, then scale back
  float nr = constrain(inRoll  / maxAngleDeg,      -1.0f, 1.0f);
  float np = constrain(inPitch / maxAngleDeg,      -1.0f, 1.0f);
  float ny = constrain(inYaw   / maxYawRateDegS,   -1.0f, 1.0f);

  nr = expoCurve(nr, rpExpo)  * rpResponse;
  np = expoCurve(np, rpExpo)  * rpResponse;
  ny = expoCurve(ny, yawExpo) * yawResponse;

  targetRoll  = constrain(nr * maxAngleDeg + trimRoll,   -maxAngleDeg, maxAngleDeg); // deg
  targetPitch = constrain(np * maxAngleDeg + trimPitch,  -maxAngleDeg, maxAngleDeg); // deg
  targetYaw   = constrain(ny * maxYawRateDegS + trimYaw, -maxYawRateDegS, maxYawRateDegS); // deg/s

  // --- Headless Mode transform ---
  if (headlessMode == true) {
    // convention: pitch < 0 = forward 
    float dx = targetRoll;          // +right
    float dy = -targetPitch;        // +forward = away from pilot

    // yaw_error = currentYaw - yaw_ref
    float yaw_error = currentYaw - yaw_ref;
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    float yaw_rad = yaw_error * DEG_TO_RAD;
    float cosYaw = cosf(yaw_rad);
    float sinYaw = sinf(yaw_rad);

    // หมุน vector จาก frame ผู้บิน → frame โดรน
    float dx_rot = cosYaw * dx - sinYaw * dy;
    float dy_rot = sinYaw * dx + cosYaw * dy;

    targetRoll  = dx_rot;
    targetPitch = -dy_rot;
  }

  // --- Yaw control ---
  yaw_rate_target = targetYaw;
  yaw_setpoint += yaw_rate_target * dt;

  float rawAltitudeRate = constrain((incomingJoystickData.YL * 0.01f), -maxAltRateMps, maxAltRateMps); // m/s
  // Apply expo + response
  float na = constrain(rawAltitudeRate / maxAltRateMps, -1.0f, 1.0f);
  na = expoCurve(na, altExpo) * altResponse;
  rawAltitudeRate = constrain(na * maxAltRateMps, -maxAltRateMps, maxAltRateMps);


  // Slew-rate limit: 
  static float limitedAltitudeRate = 0;  
  float delta = rawAltitudeRate - limitedAltitudeRate;
  if (delta > maxRateChange) {
    delta = maxRateChange;
  }
  else if (delta < -maxRateChange) {
    delta = -maxRateChange;
  }
  limitedAltitudeRate += delta;

  // Update altitude setpoint 
  altitude_setpoint += limitedAltitudeRate * dt;

  // Clamp setpoint
  altitude_setpoint = constrain(altitude_setpoint, min_altitude, max_altitude);
}

// =================== Inner PID (rate) ===================
void innerPID(float dt) {
  float gyroRoll  = -gyroX_filtered;
  float gyroPitch =  gyroY_filtered;
  float gyroYaw   = -gyroZ_filtered;

  // Roll rate
  float rollError = rollOutput_angle - gyroRoll;
  rollIntegral_rate += rollError * dt;
  rollIntegral_rate = constrain(rollIntegral_rate, -integralLimit, integralLimit);
  rollOutput_rate = pidRoll_rate.P * rollError +
                    pidRoll_rate.I * rollIntegral_rate +
                    pidRoll_rate.D * ((rollError - rollPrevError_rate)/dt);
  rollOutput_rate = rollOutput_rate + (kff_roll * rollOutput_angle);  
  rollPrevError_rate = rollError;

  // Pitch rate
  float pitchError = pitchOutput_angle - gyroPitch;
  pitchIntegral_rate += pitchError * dt;
  pitchIntegral_rate = constrain(pitchIntegral_rate, -integralLimit, integralLimit);
  pitchOutput_rate = pidPitch_rate.P * pitchError +
                     pidPitch_rate.I * pitchIntegral_rate +
                     pidPitch_rate.D * ((pitchError - pitchPrevError_rate)/dt);
  pitchOutput_rate = pitchOutput_rate + (kff_pitch * pitchOutput_angle);
  pitchPrevError_rate = pitchError;

  // Yaw rate
  float yawError = yawOutput_angle - gyroYaw;
  yawIntegral_rate += yawError * dt;
  yawIntegral_rate = constrain(yawIntegral_rate, -integralLimit, integralLimit);
  yawOutput_rate = pidYaw_rate.P * yawError +
                   pidYaw_rate.I * yawIntegral_rate +
                   pidYaw_rate.D * ((yawError - yawPrevError_rate)/dt);
  yawOutput_rate = yawOutput_rate + (kff_yaw * yawOutput_angle);
  yawPrevError_rate = yawError;

  // Altitude rate
  float altitudeError = altitudeOutput_m - velocityZ;
  altitudeIntegral_rate += altitudeError * dt;
  altitudeIntegral_rate = constrain(altitudeIntegral_rate, -integralLimit, integralLimit);
  altitudeOutput_rate = pidAltitude_rate.P * altitudeError +
                        pidAltitude_rate.I * altitudeIntegral_rate +
                        pidAltitude_rate.D * ((altitudeError - altitudePrevError_rate)/dt);      
  altitudeOutput_rate = altitudeOutput_rate + (kff_altitude * altitudeOutput_m);                                  
  altitudePrevError_rate = altitudeError;
}

// =================== Outer PID (angle / altitude) ===================
void outerPID(float dt) {
  // Roll angle
  float rollError_angle = targetRoll - currentRoll;
  rollIntegral_angle += rollError_angle * dt;
  rollIntegral_angle = constrain(rollIntegral_angle, -integralLimit, integralLimit);
  rollOutput_angle = pidRoll_angle.P * rollError_angle +
                     pidRoll_angle.I * rollIntegral_angle +
                     pidRoll_angle.D * ((rollError_angle - rollPrevError_angle)/dt);
  rollPrevError_angle = rollError_angle;

  // Pitch angle
  float pitchError_angle = targetPitch - currentPitch;
  pitchIntegral_angle += pitchError_angle * dt;
  pitchIntegral_angle = constrain(pitchIntegral_angle, -integralLimit, integralLimit);
  pitchOutput_angle = pidPitch_angle.P * pitchError_angle +
                      pidPitch_angle.I * pitchIntegral_angle +
                      pidPitch_angle.D * ((pitchError_angle - pitchPrevError_angle)/dt);
  pitchPrevError_angle = pitchError_angle;

  // Yaw angle
  float yawError_angle = angleError(yaw_setpoint, currentYaw);
  yawIntegral_angle += yawError_angle * dt;
  yawIntegral_angle = constrain(yawIntegral_angle, -integralLimit, integralLimit);
  yawOutput_angle = pidYaw_angle.P * yawError_angle +
                    pidYaw_angle.I * yawIntegral_angle +
                    pidYaw_angle.D * ((yawError_angle - yawPrevError_angle)/dt);
  yawPrevError_angle = yawError_angle;

  // Altitude
  float altitudeError_m = altitude_setpoint - currentAltitude;
  altitudeIntegral_m += altitudeError_m * dt;
  altitudeIntegral_m = constrain(altitudeIntegral_m, -integralLimit, integralLimit);
  altitudeOutput_m = pidAltitude_m.P * altitudeError_m +
                          pidAltitude_m.I * altitudeIntegral_m +
                          pidAltitude_m.D * ((altitudeError_m - altitudePrevError_m)/dt);                                           
  altitudePrevError_m = altitudeError_m;
}

void Arm() {
  ledcWrite(g_pinMA, 100);
  ledcWrite(g_pinMB, 100);
  ledcWrite(g_pinMC, 100);
  ledcWrite(g_pinMD, 100);
}

void Disarm() {
  ledcWrite(g_pinMA, 0);
  ledcWrite(g_pinMB, 0);
  ledcWrite(g_pinMC, 0);
  ledcWrite(g_pinMD, 0);
}

// =================== Motor output ===================
void driveMotors() {
  float m1 = baseSpeed  + pitchOutput_rate + rollOutput_rate - yawOutput_rate + altitudeOutput_rate;
  float m2 = baseSpeed  + pitchOutput_rate - rollOutput_rate + yawOutput_rate + altitudeOutput_rate;
  float m3 = baseSpeed  - pitchOutput_rate - rollOutput_rate - yawOutput_rate + altitudeOutput_rate;
  float m4 = baseSpeed  - pitchOutput_rate + rollOutput_rate + yawOutput_rate + altitudeOutput_rate;

  m1 = constrain(m1, 0, 1023);
  m2 = constrain(m2, 0, 1023);
  m3 = constrain(m3, 0, 1023);
  m4 = constrain(m4, 0, 1023);

  // Safety 
  if (fabs(currentRoll) > 70 || fabs(currentPitch) > 70) {
    Disarm();
  }
  else {
    ledcWrite(g_pinMA, (int)m1);
    ledcWrite(g_pinMB, (int)m2);
    ledcWrite(g_pinMC, (int)m3);
    ledcWrite(g_pinMD, (int)m4);
  }
}

void FlightController() {
  unsigned long now = micros();

  // -------- Inner loop: 400 Hz --------
  if (now - lastInnerTime >= 1000000UL / innerHz) {
    lastInnerTime += 1000000UL / innerHz;   // ใช้ period คงที่
    updateSensorsAndMadgwick(innerDt);      // innerDt คงที่ = 1/400
    innerPID(innerDt);
  }

  // -------- Outer loop: 100 Hz --------
  if (now - lastOuterTime >= 1000000UL / outerHz) {
    lastOuterTime += 1000000UL / outerHz;
    outerPID(outerDt);
  }

  // -------- Barometer: 50 Hz --------
  if (now - lastBaroTime >= 1000000UL / baroHz) {
    lastBaroTime += 1000000UL / baroHz;
    baroAltitude = bmp.readAltitude(1013.25);
  }
}

void setupSensors() {
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_2);
  myMPU9250.setSampleRateDivider(0);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_500);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_2);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                Adafruit_BMP280::SAMPLING_X4,   // temp oversampling
                Adafruit_BMP280::SAMPLING_X4,  // pressure oversampling
                Adafruit_BMP280::FILTER_X8,    // IIR filter
                Adafruit_BMP280::STANDBY_MS_1); // standby (ignore ใน FORCED)

  MadgwickFilter.begin(innerHz);
}



// [MOVED TO LIB] setup() body moved into drone_setup()





// [MOVED TO LIB] loop() body moved into drone_loop()



// ===================== Wrapped sketch entrypoints =====================
static void drone_setup(){
  Serial.begin(115200);
  Wire.begin(g_i2cSDA, g_i2cSCL);
  Wire.setClock(400000); // 400 kHz Fast Mode

  ledcAttach(g_pinMA, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(g_pinMB, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(g_pinMC, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcAttach(g_pinMD, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);

  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  if (!bmp.begin(0x76)) {
    while (1) {
      Serial.println("BMP280 connection failed. Please check your connection");
      delay(10);
    }
  }
  
  setupSensors();
  setupESPNow();
  loadConfig();
  loadCalibration();

  incomingJoystickData.webCommand = 1;
  webServerRunning = false;
  Serial.println("Setup done, waiting for joystick data...");
  Serial.println("DragonFly Spirit Version 2.0.0");
  Serial.println("Control : Angle Mode & Headless Mode");
  Serial.println("You must close the web page every time before flight");
  Serial.println(WiFi.macAddress());
}

static void drone_loop(){
  // ---------------------- WEB SERVER (TUNING vs TELEMETRY) ----------------------
  // webCommand (từ Controller): 1 = bật UI tuning (an toàn: DISARM), 0 = chế độ bay bình thường
  // Khi đang bay: tự bật Web UI telemetry để xem dữ liệu (UI tuning sẽ bị khóa).
  bool wantTelemetryUI = (Armed || OnFlying);   // ưu tiên khi ARMED/đang bay
  bool wantTuningUI    = (!wantTelemetryUI) && (incomingJoystickData.webCommand != 0);
  bool wantAnyWeb      = wantTelemetryUI || wantTuningUI;

  if (wantAnyWeb && !webServerRunning) {
    startWebServer();
  } else if (!wantAnyWeb && webServerRunning) {
    stopWebServer();
  }

  if (webServerRunning) {
    dnsServer.processNextRequest();
  }

// ---------------------- SENSOR CALIBRATION ----------------------
  if (calibrateAccelGyroRequested) { 
    AccelGyroisCalibrating = true;
    AccelGyrocalibrationDone = false;
    Serial.println("[Calibrate] Starting Accel & Gyro...");
    calibrateAccelGyro();
    saveCalibration();
    setupSensors();
    loadCalibration();
    resetPID();
    AccelGyroisCalibrating = false;
    AccelGyrocalibrationDone = true;
    calibrateAccelGyroRequested = false;
    Serial.println("[Calibrate] Accel & Gyro complete.");
  }

  if (calibrateMagRequested) {
    MagisCalibrating = true;
    MagcalibrationDone = false;
    Serial.println("[Calibrate] Starting Magnetometer...");
    calibrateMag();
    saveCalibration();
    setupSensors();
    loadCalibration();
    resetPID();
    MagisCalibrating = false;
    MagcalibrationDone = true;
    calibrateMagRequested = false;
    Serial.println("[Calibrate] Magnetometer complete.");
  }

  // ---------------------- JOYSTICK INPUT ----------------------
  swlPressed = incomingJoystickData.SWL;
  swrPressed = incomingJoystickData.SWR;
  int YL = incomingJoystickData.YL;

  // ---------------------- HEADLESS MODE (กดค้าง 3 วิ) ----------------------
  static unsigned long swlPressStart = 0;
  static unsigned long swrPressStart = 0;
  static bool swlWasPressed = false;
  static bool swrWasPressed = false;
  static bool swlActionDone = false;
  static bool swrActionDone = false;

  // ปุ่มซ้ายค้าง => เปิด headlessMode
  if (swlPressed && !swlWasPressed) {
    swlPressStart = millis();
    swlActionDone = false;
  }
  if (swlPressed && !swlActionDone && millis() - swlPressStart >= 3000) {
    headlessMode = true;
    swlActionDone = true;
  }
  if (!swlPressed) {
    swlPressStart = 0;
    swlActionDone = false;
  }

  // ปุ่มขวาค้าง => ปิด headlessMode
  if (swrPressed && !swrWasPressed) {
    swrPressStart = millis();
    swrActionDone = false;
  }
  if (swrPressed && !swrActionDone && millis() - swrPressStart >= 3000) {
    headlessMode = false;
    swrActionDone = true;
  }
  if (!swrPressed) {
    swrPressStart = 0;
    swrActionDone = false;
  }

  // อัพเดตสถานะปุ่มก่อนจบรอบ
  swlWasPressed = swlPressed;
  swrWasPressed = swrPressed;

  // ---------------------- ARM / DISARM ----------------------
  bool armButton = (swlPressed && swrPressed); // กดพร้อมกัน
  if (armButton && !lastArmButton) {
    Armed = !Armed;
    OnFlying = false; 
    initial_altitude = false;
    initial_yaw = false;
  }
  lastArmButton = armButton;

  if (Armed && YL != 0 && !OnFlying) {
    OnFlying = true;
  }

  // ---------------------- FLIGHT CONTROL ----------------------
  updateParameters(innerDt);
  FlightController();
  
  bool tuningLocked = (incomingJoystickData.webCommand != 0); // tuning mode: safety lock motors
  if (!tuningLocked) {
    if (Armed && !OnFlying) {
      if (!initial_altitude && !initial_yaw) {
        altitude_baseline = bmp.readAltitude(1013.25);
        altitude_setpoint = altitude_baseline + 1;
        max_altitude = altitude_baseline + range_altitude;
        min_altitude = altitude_baseline - range_altitude;
        initial_altitude = true;

        yaw_ref = currentYaw;
        yaw_setpoint = currentYaw;
        initial_yaw = true;
      }
      Arm(); 
    }
    else if (Armed && OnFlying) {
      driveMotors();
    }
    else {
      Disarm();
    }
  }
  else {
    Armed = false;
    OnFlying = false;
    initial_altitude = false;
    initial_yaw = false;
    Disarm();
  }
}


#include "MeblockDrone.h"

void MeblockDrone::begin(const MeblockDroneConfig& cfg){
  // ---------------- WiFi / IP ----------------
  g_apSsid = cfg.apSsid ? cfg.apSsid : g_apSsid;
  g_apPass = cfg.apPass ? cfg.apPass : g_apPass;
  apIP     = cfg.apIP;

  // ---------------- Default tuning overrides ----------------
  // (Giữ logic bay, chỉ override giá trị khởi tạo nếu user muốn)
  if (cfg.baseSpeed >= 0) baseSpeed = cfg.baseSpeed;
  if (cfg.integralLimit >= 0) integralLimit = cfg.integralLimit;

  if (!isnan(cfg.range_altitude)) range_altitude = cfg.range_altitude;
  if (!isnan(cfg.maxRateChange))  maxRateChange  = cfg.maxRateChange;

  if (!isnan(cfg.maxAngleDeg))    maxAngleDeg    = cfg.maxAngleDeg;
  if (!isnan(cfg.maxYawRateDegS)) maxYawRateDegS = cfg.maxYawRateDegS;
  if (!isnan(cfg.maxAltRateMps))  maxAltRateMps  = cfg.maxAltRateMps;

  if (!isnan(cfg.rpResponse))     rpResponse     = cfg.rpResponse;
  if (!isnan(cfg.yawResponse))    yawResponse    = cfg.yawResponse;
  if (!isnan(cfg.altResponse))    altResponse    = cfg.altResponse;

  if (!isnan(cfg.rpExpo))         rpExpo         = cfg.rpExpo;
  if (!isnan(cfg.yawExpo))        yawExpo        = cfg.yawExpo;
  if (!isnan(cfg.altExpo))        altExpo        = cfg.altExpo;

  if (!isnan(cfg.kff_roll))       kff_roll       = cfg.kff_roll;
  if (!isnan(cfg.kff_pitch))      kff_pitch      = cfg.kff_pitch;
  if (!isnan(cfg.kff_yaw))        kff_yaw        = cfg.kff_yaw;
  if (!isnan(cfg.kff_altitude))   kff_altitude   = cfg.kff_altitude;

  // ---------------- Run original setup ----------------
  drone_setup();
}

void MeblockDrone::update_and_fly(){
  drone_loop();
}
