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
uint8_t lastWebCommand = 255;


// *************************************************************  HTML  ************************************************************* //

String htmlForm(const char* message = "") {
  String mac = getMacAddress();
  mac.toUpperCase();

  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>MEBlock Drone V1</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    :root{
      --bg:#0e2a52;
      --panel:#123a6b;
      --text:#ffffff;
      --muted:rgba(255,255,255,.72);
      --accent:#2f6bff;
      --good:#2ee59d;
      --bad:#ff4d6d;
      --btn:#1a4a8d;
      --btn2:#1e5aa8;
    }
    *{box-sizing:border-box}
    body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,sans-serif}
    .wrap{max-width:1020px;margin:0 auto;padding:16px}
    .top{display:flex;gap:12px;flex-wrap:wrap;align-items:center;justify-content:space-between}
    .brand{font-weight:800;font-size:18px;letter-spacing:.3px}
    .meta{font-size:12px;color:var(--muted)}
    .right{display:flex;gap:8px;align-items:center;flex-wrap:wrap;justify-content:flex-end}
    .pill{display:inline-block;padding:6px 10px;border-radius:999px;background:rgba(255,255,255,.12);border:1px solid rgba(255,255,255,.18);font-size:12px}
    .msg{margin-top:10px;padding:10px 12px;border-radius:10px;background:rgba(46,91,153,.25);border:1px solid rgba(46,91,153,.45);font-size:13px}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px}
    @media(max-width:900px){.grid{grid-template-columns:1fr}}

    .stack{display:flex;flex-direction:column;gap:12px}
    .card{background:var(--panel);border:1px solid rgba(255,255,255,.12);border-radius:14px;padding:14px}
    h3{margin:0 0 10px 0;font-size:14px}

    .cardhead{display:flex;align-items:center;justify-content:space-between;gap:10px}
    .cardhead h3{margin:0}

    label{display:block;font-size:12px;color:var(--muted);margin:8px 0 6px}
    input{width:100%;padding:10px 10px;border-radius:10px;border:1px solid rgba(255,255,255,.18);background:rgba(0,0,0,.18);color:#fff;outline:none}
    input:focus{border-color:rgba(47,107,255,.9)}
    input[readonly]{opacity:.9}

    .row3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
    @media(max-width:720px){.row3{grid-template-columns:1fr}}

    .btn{display:inline-block;padding:10px 12px;border-radius:12px;border:1px solid rgba(255,255,255,.18);background:var(--btn);color:#fff;text-decoration:none;cursor:pointer;font-size:13px}
    .btn:hover{background:var(--btn2)}
    .btnline{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}

    .btn.btn-save{background:var(--good);color:#04251a;font-weight:900;border-color:rgba(255,255,255,.10);box-shadow:0 8px 18px rgba(46,229,157,.18)}
    .btn.btn-save:hover{filter:brightness(1.03)}
    .btn.btn-danger{background:var(--bad);font-weight:900}
    .btn.btn-danger:hover{filter:brightness(1.03)}

    .btn.btn-lang{padding:6px 10px;border-radius:999px;font-size:12px;background:rgba(0,0,0,.16);border:1px solid rgba(255,255,255,.18)}
    .btn.btn-lang:hover{background:rgba(0,0,0,.26)}

    .tabs{display:flex;gap:8px;flex-wrap:wrap}
    .tab{padding:8px 10px;border-radius:999px;border:1px solid rgba(255,255,255,.18);background:rgba(0,0,0,.16);cursor:pointer;font-size:12px;color:var(--muted)}
    .tab.active{background:rgba(47,107,255,.25);border-color:rgba(47,107,255,.6);color:#fff}

    .telemetry-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
    @media(max-width:980px){.telemetry-grid{grid-template-columns:repeat(2,minmax(0,1fr));}}
    @media(max-width:520px){.telemetry-grid{grid-template-columns:1fr;}}
    .tbox{background:rgba(0,0,0,.12);padding:10px 10px;border-radius:12px;border:1px solid rgba(255,255,255,.12)}
    .tbox .lbl{font-size:11px;color:var(--muted)}
    .tbox .val{margin-top:4px;font-size:18px;font-weight:800;color:#00ffee;font-family:ui-monospace,Menlo,Consolas,monospace}

    .hint{font-size:12px;color:var(--muted);line-height:1.35}
    .mono{font-family:ui-monospace,Menlo,Consolas,monospace}
    .dim{opacity:.65}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div>
        <div class="brand">MEBlock Drone V1</div>
        <div class="meta">MAC: <span class="mono">)rawliteral" + mac + R"rawliteral(</span></div>
      </div>
      <div class="right">
        <div class="pill">AP: <span class="mono">192.168.4.1</span></div>
        <button class="btn btn-lang" id="langBtn" type="button" title="EN/VI">EN/VI</button>
      </div>
    </div>

    )rawliteral" + (String(message).length() ? String("<div class='msg' id='serverMsg' style='display:none'>") + message + "</div>" : "") + R"rawliteral(

    <div class="grid">
      <div class="stack">
        <div class="card">
          <h3 data-i18n="telemetry_live">Telemetry (Live)</h3>
          <div class="telemetry-grid">
            <div class="tbox"><div class="lbl" data-i18n="t_target_roll">Target Roll</div><div class="val" id="tRoll">0</div></div>
            <div class="tbox"><div class="lbl" data-i18n="t_current_roll">Current Roll</div><div class="val" id="cRoll">0</div></div>
            <div class="tbox"><div class="lbl" data-i18n="t_target_pitch">Target Pitch</div><div class="val" id="tPitch">0</div></div>
            <div class="tbox"><div class="lbl" data-i18n="t_current_pitch">Current Pitch</div><div class="val" id="cPitch">0</div></div>
            <div class="tbox"><div class="lbl" data-i18n="t_target_yaw">Target Yaw</div><div class="val" id="tYaw">0</div></div>
            <div class="tbox"><div class="lbl" data-i18n="t_current_yaw">Current Yaw</div><div class="val" id="cYaw">0</div></div>
            <div class="tbox"><div class="lbl" data-i18n="t_alt_rate">Alt Rate</div><div class="val" id="tAltRate">0</div></div>
            <div class="tbox"><div class="lbl" data-i18n="t_altitude">Altitude</div><div class="val" id="cAlt">0</div></div>
          </div>
          <div class="hint" style="margin-top:10px" data-i18n-html="telemetry_hint">
            * Giống file .ino mẫu: khi mở Web Tuning, hệ thống sẽ <b>DISARM</b> để an toàn.
            Hãy đóng web trước khi bay.
          </div>
        </div>

        <div class="card">
          <div class="cardhead">
            <h3 data-i18n="save_title">Save</h3>
            <button class="btn btn-save" type="submit" form="mainForm" id="saveBtn" data-i18n="save_btn">Save</button>
          </div>
          <div id="msgHost"></div>
        </div>

        <div class="card">
          <h3 data-i18n="cal_title">Calibration</h3>
          <div class="btnline" style="margin-top:8px">
            <form method="POST" action="/calibrateAccelGyro">
              <button class="btn" type="submit" data-i18n="cal_ag">Calibrate Accel & Gyro</button>
            </form>
            <form method="POST" action="/calibrateMag">
              <button class="btn" type="submit" data-i18n="cal_mag">Calibrate Magnetometer</button>
            </form>
            <form method="POST" action="/resetCalibration">
              <button class="btn btn-danger" type="submit" data-i18n="cal_reset">Reset Calibration</button>
            </form>
          </div>
          <div class="msg" id="calStatus" style="display:none"></div>
        </div>
      </div>

      <div class="card">
        <h3 data-i18n="tuning_title">Web Tuning</h3>
        <div class="tabs" role="tablist">
          <div class="tab active" data-tab="pid" data-i18n="tab_pid">PID</div>
          <div class="tab" data-tab="load" data-i18n="tab_load">Mức tải</div>
          <div class="tab" data-tab="speed" data-i18n="tab_speed">Tốc độ</div>
          <div class="tab" data-tab="stability" data-i18n="tab_stability">Ổn định</div>
        </div>

        <form method="POST" action="/save" id="mainForm">

          <div id="tab-pid" style="margin-top:12px">
            <h3 style="margin:0 0 10px 0" data-i18n="pid_rate_title">PID (Rate)</h3>

            <div class="row3">
              <div>
                <label>Roll Rate P</label>
                <input name="pRoll_rate" type="number" step="0.001" value=")rawliteral" + String(pidRoll_rate.P) + R"rawliteral(">
              </div>
              <div>
                <label>Roll Rate I</label>
                <input name="iRoll_rate" type="number" step="0.001" value=")rawliteral" + String(pidRoll_rate.I) + R"rawliteral(">
              </div>
              <div>
                <label>Roll Rate D</label>
                <input name="dRoll_rate" type="number" step="0.001" value=")rawliteral" + String(pidRoll_rate.D) + R"rawliteral(">
              </div>
            </div>

            <div class="row3">
              <div>
                <label>Pitch Rate P</label>
                <input name="pPitch_rate" type="number" step="0.001" value=")rawliteral" + String(pidPitch_rate.P) + R"rawliteral(">
              </div>
              <div>
                <label>Pitch Rate I</label>
                <input name="iPitch_rate" type="number" step="0.001" value=")rawliteral" + String(pidPitch_rate.I) + R"rawliteral(">
              </div>
              <div>
                <label>Pitch Rate D</label>
                <input name="dPitch_rate" type="number" step="0.001" value=")rawliteral" + String(pidPitch_rate.D) + R"rawliteral(">
              </div>
            </div>

            <div class="row3">
              <div>
                <label>Yaw Rate P</label>
                <input name="pYaw_rate" type="number" step="0.001" value=")rawliteral" + String(pidYaw_rate.P) + R"rawliteral(">
              </div>
              <div>
                <label>Yaw Rate I</label>
                <input name="iYaw_rate" type="number" step="0.001" value=")rawliteral" + String(pidYaw_rate.I) + R"rawliteral(">
              </div>
              <div>
                <label>Yaw Rate D</label>
                <input name="dYaw_rate" type="number" step="0.001" value=")rawliteral" + String(pidYaw_rate.D) + R"rawliteral(">
              </div>
            </div>

            <div class="row3">
              <div>
                <label>Altitude Rate P</label>
                <input name="pAltitude_rate" type="number" step="0.001" value=")rawliteral" + String(pidAltitude_rate.P) + R"rawliteral(">
              </div>
              <div>
                <label>Altitude Rate I</label>
                <input name="iAltitude_rate" type="number" step="0.001" value=")rawliteral" + String(pidAltitude_rate.I) + R"rawliteral(">
              </div>
              <div>
                <label>Altitude Rate D</label>
                <input name="dAltitude_rate" type="number" step="0.001" value=")rawliteral" + String(pidAltitude_rate.D) + R"rawliteral(">
              </div>
            </div>

            <h3 style="margin:14px 0 10px 0" data-i18n="pid_angle_title">PID (Angle / Altitude)</h3>

            <div class="row3">
              <div>
                <label>Roll Angle P</label>
                <input name="pRoll_angle" type="number" step="0.001" value=")rawliteral" + String(pidRoll_angle.P) + R"rawliteral(">
              </div>
              <div>
                <label>Roll Angle I</label>
                <input name="iRoll_angle" type="number" step="0.001" value=")rawliteral" + String(pidRoll_angle.I) + R"rawliteral(">
              </div>
              <div>
                <label>Roll Angle D</label>
                <input name="dRoll_angle" type="number" step="0.001" value=")rawliteral" + String(pidRoll_angle.D) + R"rawliteral(">
              </div>
            </div>

            <div class="row3">
              <div>
                <label>Pitch Angle P</label>
                <input name="pPitch_angle" type="number" step="0.001" value=")rawliteral" + String(pidPitch_angle.P) + R"rawliteral(">
              </div>
              <div>
                <label>Pitch Angle I</label>
                <input name="iPitch_angle" type="number" step="0.001" value=")rawliteral" + String(pidPitch_angle.I) + R"rawliteral(">
              </div>
              <div>
                <label>Pitch Angle D</label>
                <input name="dPitch_angle" type="number" step="0.001" value=")rawliteral" + String(pidPitch_angle.D) + R"rawliteral(">
              </div>
            </div>

            <div class="row3">
              <div>
                <label>Yaw Angle P</label>
                <input name="pYaw_angle" type="number" step="0.001" value=")rawliteral" + String(pidYaw_angle.P) + R"rawliteral(">
              </div>
              <div>
                <label>Yaw Angle I</label>
                <input name="iYaw_angle" type="number" step="0.001" value=")rawliteral" + String(pidYaw_angle.I) + R"rawliteral(">
              </div>
              <div>
                <label>Yaw Angle D</label>
                <input name="dYaw_angle" type="number" step="0.001" value=")rawliteral" + String(pidYaw_angle.D) + R"rawliteral(">
              </div>
            </div>

            <div class="row3">
              <div>
                <label>Altitude (m) P</label>
                <input name="pAltitude_m" type="number" step="0.001" value=")rawliteral" + String(pidAltitude_m.P) + R"rawliteral(">
              </div>
              <div>
                <label>Altitude (m) I</label>
                <input name="iAltitude_m" type="number" step="0.001" value=")rawliteral" + String(pidAltitude_m.I) + R"rawliteral(">
              </div>
              <div>
                <label>Altitude (m) D</label>
                <input name="dAltitude_m" type="number" step="0.001" value=")rawliteral" + String(pidAltitude_m.D) + R"rawliteral(">
              </div>
            </div>
          </div>

          <div id="tab-load" style="margin-top:12px;display:none">
            <h3 style="margin:0 0 10px 0" data-i18n="load_title">Mức tải</h3>
            <label>Base Speed</label>
            <input name="baseSpeed" type="number" value=")rawliteral" + String(baseSpeed) + R"rawliteral(">
            <div class="hint" style="margin-top:10px" data-i18n="base_speed_hint">
              * Base Speed là mức throttle nền (giống file .ino).
            </div>
          </div>

          <div id="tab-speed" style="margin-top:12px;display:none">
            <h3 style="margin:0 0 10px 0" data-i18n="speed_title">Tốc độ</h3>
            <div class="row3">
              <div>
                <label data-i18n="max_rate_change">Max Rate Change (m/s)</label>
                <input name="maxRateChange" type="number" step="0.01" value=")rawliteral" + String(maxRateChange) + R"rawliteral(">
              </div>
              <div>
                <label data-i18n="roll_pitch_limit">Roll/Pitch Limit</label>
                <input type="text" value="±30°" readonly>
              </div>
              <div>
                <label data-i18n="yaw_rate_limit">Yaw Rate Limit</label>
                <input type="text" value="±90°/s" readonly>
              </div>
            </div>
            <label style="margin-top:10px" data-i18n="alt_rate_limit">Alt Rate Limit</label>
            <input type="text" value="±5 m/s" readonly>
            <div class="hint" style="margin-top:10px" data-i18n-html="speed_hint">
              * Slew-rate altitude dùng <span class="mono">maxRateChange</span> (giống logic file .ino).
            </div>
          </div>

          <div id="tab-stability" style="margin-top:12px;display:none">
            <h3 style="margin:0 0 10px 0" data-i18n="stability_title">Ổn định</h3>

            <div class="row3">
              <div>
                <label>Trim Roll</label>
                <input name="trimRoll" type="number" step="0.001" value=")rawliteral" + String(trimRoll) + R"rawliteral(">
              </div>
              <div>
                <label>Trim Pitch</label>
                <input name="trimPitch" type="number" step="0.001" value=")rawliteral" + String(trimPitch) + R"rawliteral(">
              </div>
              <div>
                <label>Trim Yaw</label>
                <input name="trimYaw" type="number" step="0.001" value=")rawliteral" + String(trimYaw) + R"rawliteral(">
              </div>
            </div>

            <label style="margin-top:10px">Trim Altitude</label>
            <input name="trimAltitude" type="number" step="0.001" value=")rawliteral" + String(trimAltitude) + R"rawliteral(">
          </div>

        </form>
      </div>
    </div>
  </div>

<script>
  // Move server message into Save card
  window.addEventListener('DOMContentLoaded', () => {
    const msg = document.getElementById('serverMsg');
    const host = document.getElementById('msgHost');
    if(msg && host){ host.appendChild(msg); msg.style.display = ''; }
  });

  // Language toggle (EN/VI)
  const I18N = {
    vi: {
      telemetry_live: 'Telemetry (Live)',
      t_target_roll: 'Roll mục tiêu',
      t_current_roll: 'Roll hiện tại',
      t_target_pitch: 'Pitch mục tiêu',
      t_current_pitch: 'Pitch hiện tại',
      t_target_yaw: 'Yaw mục tiêu',
      t_current_yaw: 'Yaw hiện tại',
      t_alt_rate: 'Tốc độ cao',
      t_altitude: 'Độ cao',
      telemetry_hint: '* Giống file .ino mẫu: khi mở Web Tuning, hệ thống sẽ <b>DISARM</b> để an toàn. Hãy đóng web trước khi bay.',
      save_title: 'Lưu',
      save_btn: 'LƯU',
      cal_title: 'Hiệu chuẩn',
      cal_ag: 'Calib Gia tốc & Gyro',
      cal_mag: 'Calib Từ kế',
      cal_reset: 'Reset Calib',
      tuning_title: 'Web Tuning',
      tab_pid: 'PID',
      tab_load: 'Mức tải',
      tab_speed: 'Tốc độ',
      tab_stability: 'Ổn định',
      pid_rate_title: 'PID (Rate)',
      pid_angle_title: 'PID (Angle / Altitude)',
      load_title: 'Mức tải',
      base_speed_hint: '* Base Speed là mức throttle nền (giống file .ino).',
      speed_title: 'Tốc độ',
      max_rate_change: 'Max Rate Change (m/s)',
      roll_pitch_limit: 'Giới hạn Roll/Pitch',
      yaw_rate_limit: 'Giới hạn Yaw rate',
      alt_rate_limit: 'Giới hạn Alt rate',
      speed_hint: '* Slew-rate altitude dùng <span class="mono">maxRateChange</span> (giống logic file .ino).',
      stability_title: 'Ổn định'
    },
    en: {
      telemetry_live: 'Telemetry (Live)',
      t_target_roll: 'Target Roll',
      t_current_roll: 'Current Roll',
      t_target_pitch: 'Target Pitch',
      t_current_pitch: 'Current Pitch',
      t_target_yaw: 'Target Yaw',
      t_current_yaw: 'Current Yaw',
      t_alt_rate: 'Alt Rate',
      t_altitude: 'Altitude',
      telemetry_hint: '* Same as the .ino: opening Web Tuning will <b>DISARM</b> for safety. Close the web before flight.',
      save_title: 'Save',
      save_btn: 'SAVE',
      cal_title: 'Calibration',
      cal_ag: 'Calibrate Accel & Gyro',
      cal_mag: 'Calibrate Magnetometer',
      cal_reset: 'Reset Calibration',
      tuning_title: 'Web Tuning',
      tab_pid: 'PID',
      tab_load: 'Load',
      tab_speed: 'Speed',
      tab_stability: 'Stability',
      pid_rate_title: 'PID (Rate)',
      pid_angle_title: 'PID (Angle / Altitude)',
      load_title: 'Load',
      base_speed_hint: '* Base Speed is the base throttle (as in the .ino).',
      speed_title: 'Speed',
      max_rate_change: 'Max Rate Change (m/s)',
      roll_pitch_limit: 'Roll/Pitch Limit',
      yaw_rate_limit: 'Yaw Rate Limit',
      alt_rate_limit: 'Alt Rate Limit',
      speed_hint: '* Altitude slew-rate uses <span class="mono">maxRateChange</span> (same as the .ino).',
      stability_title: 'Stability'
    }
  };

  let lang = (localStorage.getItem('meb_lang') || 'vi');

  function tr(key){
    return (I18N[lang] && I18N[lang][key]) || (I18N.en[key] || key);
  }

  function applyLang(){
    document.documentElement.lang = (lang === 'vi' ? 'vi' : 'en');
    const btn = document.getElementById('langBtn');
    if(btn) btn.innerHTML = (lang === 'vi' ? '<b>VI</b><span class="dim">/EN</span>' : '<span class="dim">VI/</span><b>EN</b>');

    document.querySelectorAll('[data-i18n]').forEach(el => {
      const k = el.getAttribute('data-i18n');
      el.textContent = tr(k);
    });
    document.querySelectorAll('[data-i18n-html]').forEach(el => {
      const k = el.getAttribute('data-i18n-html');
      el.innerHTML = tr(k);
    });
  }

  document.getElementById('langBtn').addEventListener('click', () => {
    lang = (lang === 'vi' ? 'en' : 'vi');
    localStorage.setItem('meb_lang', lang);
    applyLang();
  });

  applyLang();

  // Tabs
  const tabs = document.querySelectorAll('.tab');
  const panels = {
    pid: document.getElementById('tab-pid'),
    load: document.getElementById('tab-load'),
    speed: document.getElementById('tab-speed'),
    stability: document.getElementById('tab-stability'),
  };
  tabs.forEach(t => t.addEventListener('click', () => {
    tabs.forEach(x => x.classList.remove('active'));
    t.classList.add('active');
    const key = t.getAttribute('data-tab');
    Object.keys(panels).forEach(k => panels[k].style.display = (k === key ? '' : 'none'));
  }));

  // Telemetry polling (giống ino: fetch /telemetry)
  async function updateTelemetry(){
    try{
      const r = await fetch('/telemetry', {cache:'no-store'});
      const d = await r.json();
      document.getElementById('tRoll').textContent = (d.targetRoll ?? 0).toFixed(2);
      document.getElementById('tPitch').textContent = (d.targetPitch ?? 0).toFixed(2);
      document.getElementById('tYaw').textContent = (d.targetYaw ?? 0).toFixed(2);
      document.getElementById('tAltRate').textContent = (d.altitude_rate_target ?? 0).toFixed(2);
      document.getElementById('cRoll').textContent = (d.currentRoll ?? 0).toFixed(2);
      document.getElementById('cPitch').textContent = (d.currentPitch ?? 0).toFixed(2);
      document.getElementById('cYaw').textContent = (d.currentYaw ?? 0).toFixed(2);
      document.getElementById('cAlt').textContent = (d.currentAltitude ?? 0).toFixed(2);
    }catch(e){}
  }
  updateTelemetry();
  setInterval(updateTelemetry, 200);

  // Calibration status
  async function pollCal(){
    try{
      const r = await fetch('/calibrationStatus', {cache:'no-store'});
      const t = await r.text();
      const el = document.getElementById('calStatus');
      if(t && t.trim().length){
        el.style.display = '';
        el.textContent = t;
      } else {
        el.style.display = 'none';
      }
    }catch(e){}
  }
  setInterval(pollCal, 500);
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
  maxRateChange = preferences.getFloat("maxRateChange", maxRateChange);
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
  preferences.putFloat("maxRateChange", maxRateChange);
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

    if (!routesRegistered) {
      server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", htmlForm());
      });

      server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
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

      server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
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
        if (request->hasParam("baseSpeed", true)) baseSpeed = request->getParam("baseSpeed", true)->value().toInt();
        if (request->hasParam("maxRateChange", true)) {
          float v = request->getParam("maxRateChange", true)->value().toFloat();
          if (v < 0.01f) v = 0.01f;
          if (v > 10.0f) v = 10.0f;
          maxRateChange = v;
        }

        saveConfig();
        printConfigToSerial();
        request->send(200, "text/html", htmlForm("Settings updated successfully!"));
      });

      server.on("/calibrateAccelGyro", HTTP_POST, [](AsyncWebServerRequest *request){
        calibrateAccelGyroRequested = true;
        request->send(200, "text/html", htmlForm("Accel & Gyro Calibration started. Please keep the drone still"));
      });

      server.on("/calibrateMag", HTTP_POST, [](AsyncWebServerRequest *request){
        calibrateMagRequested = true;
        request->send(200, "text/html", htmlForm("Magnetometer Calibration started. Please move the drone"));
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

      server.on("/resetCalibration", HTTP_POST, [](AsyncWebServerRequest *request){
        resetCalibration();
        request->send(200, "text/html", htmlForm("Calibration reset to default."));
      });

      server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(200, "text/html", htmlForm());
      });

      routesRegistered = true;
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
  // === Mapping input giống file .ino mẫu ===
  targetRoll  = constrain(incomingJoystickData.XR + trimRoll, -30, 30); // deg
  targetPitch = constrain(incomingJoystickData.YR + trimPitch, -30, 30); // deg
  targetYaw   = constrain(incomingJoystickData.XL + trimYaw, -90, 90); // deg/s

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

  float rawAltitudeRate = constrain(incomingJoystickData.YL * 0.01f, -5.0f, 5.0f); // m/s

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
  // ---------------------- WEB SERVER (giống .ino mẫu) ----------------------
  if (incomingJoystickData.webCommand != lastWebCommand) {
    if (incomingJoystickData.webCommand && !webServerRunning) {
      startWebServer();
    } 
    else if (!incomingJoystickData.webCommand && webServerRunning) {
      stopWebServer();
    }
    lastWebCommand = incomingJoystickData.webCommand;
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

  if (!incomingJoystickData.webCommand && !webServerRunning) {
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
  // SoftAP config
  g_apSsid = cfg.apSsid ? cfg.apSsid : g_apSsid;
  g_apPass = cfg.apPass ? cfg.apPass : g_apPass;
  apIP     = cfg.apIP;

  // Run original setup
  drone_setup();
}

void MeblockDrone::update_and_fly(){
  drone_loop();
}
