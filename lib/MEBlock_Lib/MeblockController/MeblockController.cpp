#include <WiFi.h>
#include "MeblockController.h"
#include <esp_now.h>
#include "esp_wifi.h"
#include <DNSServer.h>
#include <WebServer.h>
#include <Preferences.h>

// ==== PIN Definitions ====
#define XL_PIN 3 //3 34
#define YL_PIN 4  //4 35
#define SWL_PIN 6 //6 25
#define XR_PIN 1 //1 32
#define YR_PIN 2 //2 33
#define SWR_PIN 5 //5 26

// ==== WiFi Config ====
char ssid[33] = "Controller Calibration";
char password[65] = "12345678";
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;
WebServer server(80);
bool wifiActive = false;
bool inCalibrationMode = false;  // <--- เพิ่ม flag โหมด Calibration

Preferences prefs;

// ==== Joystick Data ====
int XL, YL, XR, YR;
bool SWL, SWR;
uint8_t webCommand = 0;

int XLMinMap, XLMaxMap, YLMinMap, YLMaxMap;
int XRMinMap, XRMaxMap, YRMinMap, YRMaxMap;

int XL_min = 4095, YL_min = 4095, XR_min = 4095, YR_min = 4095;
int XL_max = 0,    YL_max = 0,    XR_max = 0,    YR_max = 0;
int XL_center = 0, YL_center = 0, XR_center = 0, YR_center = 0;
int deadzone = 60;
int loopDelayMs = 50;

// ==== Double Click Detect ====
unsigned long lastClickSWL = 0, lastClickSWR = 0;
int clickCountSWL = 0, clickCountSWR = 0;
const unsigned long clickInterval = 500;

// ==== ESP-NOW ====
uint8_t ReceiverMAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //08:A6:F7:21:BA:6C
struct JoyData {
  int XL, YL;
  bool SWL;
  int XR, YR;
  bool SWR;
  uint8_t webCommand;
};

// ==== Sci-Fi Style HTML Page ====
const char htmlPage[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>Controller Calibration</title>
<style>
  :root{
    --bg0:#0b2a4b;
    --bg1:#0a2240;
    --card:#10365f;
    --stroke:rgba(255,255,255,.10);
    --stroke2:rgba(255,255,255,.16);
    --text:#ffffff;
    --muted:rgba(255,255,255,.72);
    --muted2:rgba(255,255,255,.55);
    --primary:#2f7dff;
    --primary2:#1b5fd6;
    --danger:#b33a3a;
    --danger2:#8f2b2b;
    --pill:rgba(0,0,0,.20);
    --radius:14px;
  }
  *{box-sizing:border-box}
  body{
    margin:0;
    font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
    color:var(--text);
    background: radial-gradient(1200px 600px at 15% 10%, rgba(47,125,255,.28), transparent 60%),
                linear-gradient(180deg,var(--bg0),var(--bg1));
  }
  .wrap{max-width:1120px;margin:0 auto;padding:12px}
  .card{
    background: linear-gradient(180deg, rgba(255,255,255,.06), rgba(255,255,255,.03));
    border:1px solid var(--stroke);
    border-radius: var(--radius);
    padding:12px;
    box-shadow: 0 10px 30px rgba(0,0,0,.20);
  }
  .topbar{
    display:flex;align-items:center;justify-content:space-between;gap:10px;
    padding:12px 14px;
  }
  .brand{font-size:18px;font-weight:900;letter-spacing:.2px;line-height:1.1}
  .sub{margin-top:4px;color:var(--muted);font-size:12px}
  .pill{
    background: var(--pill);
    border:1px solid var(--stroke2);
    padding:6px 10px;border-radius:999px;
    color:var(--text);font-size:12px;font-weight:800;
    white-space:nowrap;
  }
  .pillbtn{cursor:pointer; user-select:none}
  .pillbtn:active{transform: translateY(1px)}
  .topActions{display:flex;gap:8px;align-items:center}

  .tabs{display:flex;gap:8px;margin:10px 0; flex-wrap:wrap}
  .tabbtn{
    border:1px solid var(--stroke2);
    background: rgba(0,0,0,.18);
    color:var(--text);
    padding:8px 12px;border-radius:10px;
    cursor:pointer;font-weight:900;font-size:13px;
  }
  .tabbtn.active{
    background: rgba(47,125,255,.22);
    border-color: rgba(47,125,255,.45);
  }

  .grid{
    display:grid;gap:10px;align-items:start;
    grid-template-columns: 420px 1fr;
  }
  @media (max-width: 980px){
    .grid{grid-template-columns: 1fr}
  }

  h3{margin:0 0 8px 0;font-size:15px}
  .muted{color:var(--muted);font-size:12px;line-height:1.45}
  .note{
    margin-top:10px;
    padding:10px;
    border:1px dashed rgba(255,255,255,.18);
    border-radius:12px;
    background: rgba(0,0,0,.14);
    color:var(--muted);
    font-size:12px;
  }
  .note b{color:#fff}

  .formgrid{
    display:grid;gap:10px;
    grid-template-columns: 1fr 1fr;
  }
  .field label{display:block;margin-bottom:6px;color:var(--muted);font-size:12px}
  input{
    width:100%;
    padding:10px 10px;
    border-radius:10px;
    border:1px solid var(--stroke2);
    background: rgba(0,0,0,.18);
    color:var(--text);
    outline:none;
    font-weight:800;
  }
  input::placeholder{color: rgba(255,255,255,.45); font-weight:700}
  .btnrow{display:flex;gap:10px;margin-top:12px;flex-wrap:wrap}
  .btn{
    border:1px solid transparent;
    border-radius:12px;
    padding:10px 12px;
    font-weight:900;
    cursor:pointer;
    background: rgba(0,0,0,.20);
    color:var(--text);
  }
  .btn.primary{background: linear-gradient(180deg, var(--primary), var(--primary2))}
  .btn.danger{background: linear-gradient(180deg, var(--danger), var(--danger2))}
  .btn:active{transform: translateY(1px)}
  .status{
    margin-top:12px;
    padding:10px 12px;
    border-radius:12px;
    border:1px solid var(--stroke);
    background: rgba(0,0,0,.16);
    color:var(--muted);
    font-size:12px;
    font-weight:800;
    min-height: 40px;
    display:flex;align-items:center;
  }

  /* Right side */
  .rightStack{display:grid;gap:10px}
  .teleGrid{
    display:grid;grid-template-columns: repeat(4, 1fr);
    gap:8px;margin-top:10px;
  }
  @media (max-width: 560px){ .teleGrid{grid-template-columns: repeat(2, 1fr)} }
  .kv{
    border:1px solid var(--stroke);
    border-radius:12px;
    padding:10px 10px;
    background: rgba(0,0,0,.16);
  }
  .k{color:var(--muted2);font-size:11px;font-weight:900}
  .v{font-size:18px;font-weight:1000;letter-spacing:.3px}

  .mini{display:grid;grid-template-columns: 1fr 1fr; gap:10px; margin-top:10px}
  @media (max-width: 560px){ .mini{grid-template-columns: 1fr} }

  .joyBox{
    border:1px solid var(--stroke);
    border-radius:12px;
    padding:10px;
    background: rgba(0,0,0,.14);
  }
  .joyTitle{font-weight:1000; font-size:13px; margin-bottom:8px}
  .pad{
    position:relative;
    width:160px; height:160px;
    border-radius:14px;
    background: radial-gradient(circle at 50% 50%, rgba(255,255,255,.06), rgba(0,0,0,.18));
    border:1px solid rgba(255,255,255,.12);
    margin: 0 auto 8px auto;
    overflow:hidden;
  }
  .centerDot{
    position:absolute;left:50%;top:50%;
    width:8px;height:8px;border-radius:999px;
    background: rgba(255,255,255,.55);
    transform: translate(-50%,-50%);
  }
  .handle{
    position:absolute;left:50%;top:50%;
    width:18px;height:18px;border-radius:999px;
    background: rgba(47,125,255,.9);
    border:1px solid rgba(255,255,255,.25);
    transform: translate(0px,0px);
  }
  .miniNums{
    display:grid;grid-template-columns: repeat(3,1fr);
    gap:6px;
    font-size:11px;
    color:var(--muted);
  }
  .miniNums b{color:#fff}

  .tabpanel{display:none}
  .tabpanel.active{display:block}
</style>
</head>

<body>
  <div class="wrap">
    <div class="card topbar">
      <div>
        <div class="brand" data-i18n="brand">Controller Calibration</div>
        <div class="sub">
          <span data-i18n="receiverMac">Receiver MAC</span>: <b id="rxMac">--:--:--:--:--:--</b> &nbsp;·&nbsp; AP: <b>192.168.4.1</b>
        </div>
      </div>
      <div class="topActions">
        <button class="pill pillbtn" id="langBtn" title="Language">VI</button>
        <div class="pill">Controller UI</div>
      </div>
    </div>

    <div class="tabs">
      <button class="tabbtn active" data-tab="map" data-i18n="tabMapping">Mapping</button>
      <button class="tabbtn" data-tab="mac" data-i18n="tabMac">MAC</button>
      <button class="tabbtn" data-tab="help" data-i18n="tabHelp">Hướng dẫn</button>
    </div>

    <div class="grid">
      <!-- LEFT: Settings -->
      <div class="card">
        <div id="tab-map" class="tabpanel active">
          <h3 data-i18n="mapTitle">Mapping Parameters</h3>
          <div class="muted" data-i18n="mapDesc">
            Dùng để map raw ADC của joystick sang giá trị điều khiển (roll/pitch/yaw/throttle). Âm/dương quyết định chiều.
          </div>

          <div class="note" data-i18n="mapNote">
            <b>Mặc định theo firmware:</b> XL [-10..10], YL [-20..20], XR [-15..15], YR [15..-15] (đảo chiều).
            <br/>Muốn đảo trục: đổi dấu (ví dụ YR Max = +15 và YR Min = -15).
          </div>

          <div class="formgrid" style="margin-top:12px">
            <div class="field"><label><b>XL Min</b> map</label><input type="number" id="XLMinMap" /></div>
            <div class="field"><label><b>XL Max</b> map</label><input type="number" id="XLMaxMap" /></div>
            <div class="field"><label><b>YL Min</b> map</label><input type="number" id="YLMinMap" /></div>
            <div class="field"><label><b>YL Max</b> map</label><input type="number" id="YLMaxMap" /></div>
            <div class="field"><label><b>XR Min</b> map</label><input type="number" id="XRMinMap" /></div>
            <div class="field"><label><b>XR Max</b> map</label><input type="number" id="XRMaxMap" /></div>
            <div class="field"><label><b>YR Min</b> map</label><input type="number" id="YRMinMap" /></div>
            <div class="field"><label><b>YR Max</b> map</label><input type="number" id="YRMaxMap" /></div>
          </div>
          <div class="btnrow">
            <button class="btn primary" id="btnSaveMap" data-i18n="btnSaveMap">Lưu mapping</button>
          </div>
        </div>

        <div id="tab-mac" class="tabpanel">
          <h3 data-i18n="macTitle">Receiver MAC Address</h3>
          <div class="muted" data-i18n="macDesc">
            Dùng để controller gửi ESP-NOW tới drone. Ví dụ: <b>08:A6:F7:21:BA:6C</b>
          </div>
          <div class="formgrid" style="grid-template-columns: 1fr; margin-top:12px">
            <div class="field">
              <label><b data-i18n="macLabel">Receiver MAC</b></label>
              <input type="text" id="macAddress" placeholder="08:A6:F7:21:BA:6C" />
            </div>
          </div>
          <div class="btnrow">
            <button class="btn primary" id="btnSaveMac" data-i18n="btnSaveMac">Lưu MAC</button>
          </div>
        </div>

        <div id="tab-help" class="tabpanel">
          <h3 data-i18n="helpTitle">Hướng dẫn nhanh</h3>
          <div class="muted" data-i18n="helpDesc">
            <b>Calib Center</b>: để joystick về giữa rồi bấm.<br/>
            <b>Save Min/Max</b>: đảo joystick hết biên 4 góc vài lần rồi bấm.<br/>
            <b>Reset</b>: xoá toàn bộ giá trị đã lưu.
          </div>
        </div>

        <div id="status" class="status">Status: --</div>
      </div>

      <!-- RIGHT: Telemetry + Calib -->
      <div class="rightStack">
        <div class="card" id="telemetryCard">
          <h3 data-i18n="teleTitle">Telemetry (live)</h3>

          <div class="teleGrid">
            <div class="kv"><div class="k">XL</div><div class="v" id="xl">0</div></div>
            <div class="kv"><div class="k">YL</div><div class="v" id="yl">0</div></div>
            <div class="kv"><div class="k">XR</div><div class="v" id="xr">0</div></div>
            <div class="kv"><div class="k">YR</div><div class="v" id="yr">0</div></div>
          </div>

          <div class="mini">
            <div class="joyBox">
              <div class="joyTitle" data-i18n="leftJoy">Left Joystick</div>
              <div class="pad">
                <div class="centerDot"></div>
                <div class="handle" id="leftHandle"></div>
              </div>
              <div class="miniNums">
                <div><span data-i18n="min">Min</span> <b id="xl_min">0</b></div>
                <div><span data-i18n="center">C</span> <b id="xl_center">0</b></div>
                <div><span data-i18n="max">Max</span> <b id="xl_max">0</b></div>
                <div><span data-i18n="min">Min</span> <b id="yl_min">0</b></div>
                <div><span data-i18n="center">C</span> <b id="yl_center">0</b></div>
                <div><span data-i18n="max">Max</span> <b id="yl_max">0</b></div>
                <div>SWL <b id="swl">0</b></div>
              </div>
            </div>

            <div class="joyBox">
              <div class="joyTitle" data-i18n="rightJoy">Right Joystick</div>
              <div class="pad">
                <div class="centerDot"></div>
                <div class="handle" id="rightHandle"></div>
              </div>
              <div class="miniNums">
                <div><span data-i18n="min">Min</span> <b id="xr_min">0</b></div>
                <div><span data-i18n="center">C</span> <b id="xr_center">0</b></div>
                <div><span data-i18n="max">Max</span> <b id="xr_max">0</b></div>
                <div><span data-i18n="min">Min</span> <b id="yr_min">0</b></div>
                <div><span data-i18n="center">C</span> <b id="yr_center">0</b></div>
                <div><span data-i18n="max">Max</span> <b id="yr_max">0</b></div>
                <div>SWR <b id="swr">0</b></div>
              </div>
            </div>
          </div>
        </div>

        <div class="card" id="calibCard">
          <h3 data-i18n="calibTitle">Calibration</h3>
          <div class="muted" data-i18n="calibDesc">
            Chỉ calib khi drone không bay. Calib Center: thả tay tự nhiên ở giữa. Save Min/Max: đảo biên 4 góc.
          </div>
          <div class="btnrow">
            <button class="btn primary" id="btnSaveMinMax" data-i18n="btnSaveMinMax">Save Min/Max</button>
            <button class="btn primary" id="btnSaveCenter" data-i18n="btnSaveCenter">Calibrate Center</button>
            <button class="btn danger" id="btnResetAll" data-i18n="btnResetAll">Reset All</button>
          </div>
        </div>
      </div>
    </div>
  </div>

<script>
  const i18n = {
    vi: {
      brand: 'Hiệu chỉnh Controller',
      receiverMac: 'Receiver MAC',
      tabMapping: 'Mapping',
      tabMac: 'MAC',
      tabHelp: 'Hướng dẫn',
      mapTitle: 'Thông số Mapping',
      mapDesc: 'Dùng để map raw ADC của joystick sang giá trị điều khiển (roll/pitch/yaw/throttle). Âm/dương quyết định chiều.',
      mapNote: '<b>Mặc định theo firmware:</b> XL [-10..10], YL [-20..20], XR [-15..15], YR [15..-15] (đảo chiều).<br/>Muốn đảo trục: đổi dấu (ví dụ YR Max = +15 và YR Min = -15).',
      btnSaveMap: 'Lưu mapping',
      macTitle: 'Địa chỉ MAC của Drone',
      macDesc: 'Dùng để controller gửi ESP-NOW tới drone. Ví dụ: <b>08:A6:F7:21:BA:6C</b>',
      macLabel: 'Receiver MAC',
      btnSaveMac: 'Lưu MAC',
      helpTitle: 'Hướng dẫn nhanh',
      helpDesc: '<b>Calib Center</b>: để joystick về giữa rồi bấm.<br/><b>Save Min/Max</b>: đảo joystick hết biên 4 góc vài lần rồi bấm.<br/><b>Reset</b>: xoá toàn bộ giá trị đã lưu.',
      teleTitle: 'Telemetry (live)',
      leftJoy: 'Joystick Trái',
      rightJoy: 'Joystick Phải',
      calibTitle: 'Hiệu chỉnh',
      calibDesc: 'Chỉ calib khi drone không bay. Calib Center: thả tay tự nhiên ở giữa. Save Min/Max: đảo biên 4 góc.',
      btnSaveMinMax: 'Lưu Min/Max',
      btnSaveCenter: 'Calib Center',
      btnResetAll: 'Reset',
      status: 'Trạng thái',
      min: 'Min',
      max: 'Max',
      center: 'C',
      ready: 'Sẵn sàng',
      saved: 'Đã lưu',
      savedMap: 'Đã lưu mapping',
      savedMac: 'Đã lưu MAC',
      savedMinMax: 'Đã lưu Min/Max',
      savedCenter: 'Đã calib Center',
      resetDone: 'Đã reset'
    },
    en: {
      brand: 'Controller Calibration',
      receiverMac: 'Receiver MAC',
      tabMapping: 'Mapping',
      tabMac: 'MAC',
      tabHelp: 'Help',
      mapTitle: 'Mapping Parameters',
      mapDesc: 'Map raw joystick ADC values into control output (roll/pitch/yaw/throttle). Sign (+/-) sets direction.',
      mapNote: '<b>Firmware defaults:</b> XL [-10..10], YL [-20..20], XR [-15..15], YR [15..-15] (inverted).<br/>To invert an axis: flip the sign (ex: YR Max = +15 and YR Min = -15).',
      btnSaveMap: 'Save mapping',
      macTitle: 'Drone Receiver MAC',
      macDesc: 'Used for ESP-NOW target (drone). Example: <b>08:A6:F7:21:BA:6C</b>',
      macLabel: 'Receiver MAC',
      btnSaveMac: 'Save MAC',
      helpTitle: 'Quick guide',
      helpDesc: '<b>Calib Center</b>: release sticks to center, then click.<br/><b>Save Min/Max</b>: move sticks to all corners, then click.<br/><b>Reset</b>: clear all stored values.',
      teleTitle: 'Telemetry (live)',
      leftJoy: 'Left Joystick',
      rightJoy: 'Right Joystick',
      calibTitle: 'Calibration',
      calibDesc: 'Calibrate only when the drone is not flying. Center: release to neutral. Min/Max: move to all corners.',
      btnSaveMinMax: 'Save Min/Max',
      btnSaveCenter: 'Calibrate Center',
      btnResetAll: 'Reset All',
      status: 'Status',
      min: 'Min',
      max: 'Max',
      center: 'C',
      ready: 'Ready',
      saved: 'Saved',
      savedMap: 'Mapping saved',
      savedMac: 'MAC saved',
      savedMinMax: 'Min/Max saved',
      savedCenter: 'Center calibrated',
      resetDone: 'All values reset'
    }
  };

  let LANG = localStorage.getItem('lang') || 'vi';
  let lastStatus = '--';

  function t(key){
    return (i18n[LANG] && i18n[LANG][key]) ? i18n[LANG][key] : key;
  }

  function applyLang(){
    document.documentElement.lang = LANG;
    document.querySelectorAll('[data-i18n]').forEach(el=>{
      const k = el.getAttribute('data-i18n');
      const val = t(k);
      el.innerHTML = val;
    });
    const btn = document.getElementById('langBtn');
    if(btn) btn.textContent = (LANG === 'vi') ? 'VI' : 'EN';
    // refresh status label
    setStatus(lastStatus, true);
  }

  const statusEl = document.getElementById('status');

  function setStatus(msg, keepRaw){
    lastStatus = msg;
    const label = t('status');
    statusEl.textContent = label + ': ' + msg;
  }

  // Language toggle
  document.getElementById('langBtn')?.addEventListener('click', ()=>{
    LANG = (LANG === 'vi') ? 'en' : 'vi';
    localStorage.setItem('lang', LANG);
    applyLang();
  });

  // Tabs
  document.querySelectorAll('.tabbtn').forEach(btn=>{
    btn.addEventListener('click', ()=>{
      document.querySelectorAll('.tabbtn').forEach(b=>b.classList.remove('active'));
      btn.classList.add('active');
      const tab = btn.dataset.tab;
      document.querySelectorAll('.tabpanel').forEach(p=>p.classList.remove('active'));
      const el = document.getElementById('tab-' + tab);
      if(el) el.classList.add('active');
    });
  });

  function updateHandle(elId, x, y, cx, cy, minX, maxX, minY, maxY){
    let rangeX = (maxX - minX); if(rangeX<=0) rangeX = 1;
    let rangeY = (maxY - minY); if(rangeY<=0) rangeY = 1;
    let nx = -1 * (x - cx) / (rangeX/2);
    let ny = -1 * (y - cy) / (rangeY/2);
    nx = Math.max(-1, Math.min(1, nx));
    ny = Math.max(-1, Math.min(1, ny));
    const radius = 62;
    const tx = nx * radius;
    const ty = ny * radius;
    const h = document.getElementById(elId);
    if(h) h.style.transform = `translate(${tx}px, ${ty}px)`;
  }

  function loadMap(){
    fetch('/get_map').then(r=>r.json()).then(d=>{
      ['XLMinMap','XLMaxMap','YLMinMap','YLMaxMap','XRMinMap','XRMaxMap','YRMinMap','YRMaxMap'].forEach(k=>{
        const el = document.getElementById(k);
        if(el) el.value = d[k];
      });
    }).catch(()=>{});
  }

  function loadMac(){
    fetch('/get_mac').then(r=>r.text()).then(mac=>{
      document.getElementById('rxMac').textContent = mac || '--';
      const inEl = document.getElementById('macAddress');
      if(inEl) inEl.value = mac || '';
    }).catch(()=>{});
  }

  function saveMap(){
    const params = new URLSearchParams();
    ['XLMinMap','XLMaxMap','YLMinMap','YLMaxMap','XRMinMap','XRMaxMap','YRMinMap','YRMaxMap'].forEach(k=>{
      params.append(k, (document.getElementById(k)?.value || ''));
    });
    fetch('/save_map?' + params.toString())
      .then(()=>setStatus(t('savedMap')));
  }

  function saveMac(){
    const mac = (document.getElementById('macAddress')?.value || '').trim();
    fetch('/save_mac?mac=' + encodeURIComponent(mac))
      .then(()=>{ setStatus(t('savedMac')); loadMac(); });
  }

  function saveMinMax(){
    fetch('/save_minmax').then(()=>setStatus(t('savedMinMax')));
  }
  function saveCenter(){
    fetch('/save_center').then(()=>setStatus(t('savedCenter')));
  }
  function resetAll(){
    fetch('/reset_values').then(()=>setStatus(t('resetDone')));
  }

  document.getElementById('btnSaveMap')?.addEventListener('click', saveMap);
  document.getElementById('btnSaveMac')?.addEventListener('click', saveMac);
  document.getElementById('btnSaveMinMax')?.addEventListener('click', saveMinMax);
  document.getElementById('btnSaveCenter')?.addEventListener('click', saveCenter);
  document.getElementById('btnResetAll')?.addEventListener('click', resetAll);

  // Init
  window.addEventListener('load', ()=>{
    applyLang();
    loadMap();
    loadMac();
    setStatus(t('ready'));
  });

  // Telemetry refresh
  setInterval(()=>{
    fetch('/data').then(r=>r.json()).then(d=>{
      document.getElementById('xl').textContent = d.XL;
      document.getElementById('yl').textContent = d.YL;
      document.getElementById('swl').textContent = d.SWL;

      document.getElementById('xr').textContent = d.XR;
      document.getElementById('yr').textContent = d.YR;
      document.getElementById('swr').textContent = d.SWR;

      document.getElementById('xl_min').textContent = d.XL_min;
      document.getElementById('xl_max').textContent = d.XL_max;
      document.getElementById('yl_min').textContent = d.YL_min;
      document.getElementById('yl_max').textContent = d.YL_max;

      document.getElementById('xr_min').textContent = d.XR_min;
      document.getElementById('xr_max').textContent = d.XR_max;
      document.getElementById('yr_min').textContent = d.YR_min;
      document.getElementById('yr_max').textContent = d.YR_max;

      document.getElementById('xl_center').textContent = d.XL_center;
      document.getElementById('yl_center').textContent = d.YL_center;
      document.getElementById('xr_center').textContent = d.XR_center;
      document.getElementById('yr_center').textContent = d.YR_center;

      updateHandle('leftHandle', d.XL, d.YL, d.XL_center, d.YL_center, d.XL_min, d.XL_max, d.YL_min, d.YL_max);
      updateHandle('rightHandle', d.XR, d.YR, d.XR_center, d.YR_center, d.XR_min, d.XR_max, d.YR_min, d.YR_max);
    }).catch(()=>{});
  }, 120);
</script>
</body>
</html>
)rawliteral";

int readAverage(int pin, int samples = 5) {
  int sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return sum / samples;
}

void readJoystick() {
  XL = readAverage(XL_PIN);
  YL = readAverage(YL_PIN);
  XR = readAverage(XR_PIN);
  YR = readAverage(YR_PIN);
  SWL = !digitalRead(SWL_PIN);
  SWR = !digitalRead(SWR_PIN);

  // Update min/max แบบ realtime ตามค่าจริง
  if (XL < XL_min) XL_min = XL;
  if (XL > XL_max) XL_max = XL;

  if (YL < YL_min) YL_min = YL;
  if (YL > YL_max) YL_max = YL;

  if (XR < XR_min) XR_min = XR;
  if (XR > XR_max) XR_max = XR;

  if (YR < YR_min) YR_min = YR;
  if (YR > YR_max) YR_max = YR;
}

void loadReceiverMAC() {
  prefs.begin("mac", true);
  String macStr = prefs.getString("mac", "00:00:00:00:00:00");
  prefs.end();

  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; i++) {
      ReceiverMAC[i] = (uint8_t)values[i];
    }
  }
}

void sendESPNow() {
  JoyData data = {XL, YL, SWL, XR, YR, SWR, webCommand};
  esp_now_send(ReceiverMAC, (uint8_t*)&data, sizeof(data));
  
  Serial.print("  Sending -> ");
  Serial.print("XL: "); Serial.print(data.XL); Serial.print("  ");
  Serial.print("YL: "); Serial.print(data.YL); Serial.print("  ");
  Serial.print("SWL: "); Serial.print(data.SWL); Serial.print("  ");
  Serial.print("XR: "); Serial.print(data.XR); Serial.print("  ");
  Serial.print("YR: "); Serial.print(data.YR); Serial.print("  ");
  Serial.print("SWR: "); Serial.print(data.SWR); Serial.print("  ");
  Serial.print("webCommand: "); Serial.print(data.webCommand);
}

void handleWebData() {
  readJoystick();
  String json = "{";
  json += "\"XL\":" + String(XL) + ",";
  json += "\"YL\":" + String(YL) + ",";
  json += "\"SWL\":" + String(SWL) + ",";
  json += "\"XR\":" + String(XR) + ",";
  json += "\"YR\":" + String(YR) + ",";
  json += "\"SWR\":" + String(SWR) + ",";
  json += "\"XL_min\":" + String(XL_min) + ",\"XL_max\":" + String(XL_max) + ",";
  json += "\"YL_min\":" + String(YL_min) + ",\"YL_max\":" + String(YL_max) + ",";
  json += "\"XR_min\":" + String(XR_min) + ",\"XR_max\":" + String(XR_max) + ",";
  json += "\"YR_min\":" + String(YR_min) + ",\"YR_max\":" + String(YR_max) + ",";
  json += "\"XL_center\":" + String(XL_center) + ",";
  json += "\"YL_center\":" + String(YL_center) + ",";
  json += "\"XR_center\":" + String(XR_center) + ",";
  json += "\"YR_center\":" + String(YR_center);
  json += "}";
  server.send(200, "application/json", json);
}

void saveMinMax() {
  prefs.begin("joy", false);
  prefs.putInt("XL_min", XL_min); prefs.putInt("XL_max", XL_max);
  prefs.putInt("YL_min", YL_min); prefs.putInt("YL_max", YL_max);
  prefs.putInt("XR_min", XR_min); prefs.putInt("XR_max", XR_max);
  prefs.putInt("YR_min", YR_min); prefs.putInt("YR_max", YR_max);
  prefs.end();
  server.send(200, "text/plain", "Min/Max saved");
}

void saveCenter() {
  readJoystick();  // อ่านค่าปัจจุบัน
  prefs.begin("joy", false);
  prefs.putInt("XL_center", XL);
  prefs.putInt("YL_center", YL);
  prefs.putInt("XR_center", XR);
  prefs.putInt("YR_center", YR);
  prefs.end();

  XL_center = XL;
  YL_center = YL;
  XR_center = XR;
  YR_center = YR;

  server.send(200, "text/plain", "Center saved");
}

void loadStoredValues() {
  prefs.begin("joy", true);
  XL_min = prefs.getInt("XL_min", 4095); XL_max = prefs.getInt("XL_max", 0);
  YL_min = prefs.getInt("YL_min", 4095); YL_max = prefs.getInt("YL_max", 0);
  XR_min = prefs.getInt("XR_min", 4095); XR_max = prefs.getInt("XR_max", 0);
  YR_min = prefs.getInt("YR_min", 4095); YR_max = prefs.getInt("YR_max", 0);
  XL_center = prefs.getInt("XL_center", 0);
  YL_center = prefs.getInt("YL_center", 0);
  XR_center = prefs.getInt("XR_center", 0);
  YR_center = prefs.getInt("YR_center", 0);
  prefs.end();
}

void loadMapValues() {
  // ===== Mapping output values (DEFAULTS MATCH ControllerDragonFlySpirit_fixed.ino) =====
  // These values define the OUTPUT range after deadzone removal and scaling, then sent via ESP-NOW.
  // Notes:
  // - XL / XR are inverted in code (a '-' sign is applied), so sign here affects final direction.
  // - To invert an axis, simply swap the sign (example: default YR is inverted by using Max=-15, Min=+15).
  // Recommended defaults:
  //   XL: -10..+10, YL: -20..+20, XR: -15..+15, YR: +15..-15 (inverted)
  prefs.begin("map", true);
  XLMinMap = prefs.getInt("XLMinMap", -10);
  XLMaxMap = prefs.getInt("XLMaxMap", 10);
  YLMinMap = prefs.getInt("YLMinMap", -20);
  YLMaxMap = prefs.getInt("YLMaxMap", 20);
  XRMinMap = prefs.getInt("XRMinMap", -15);
  XRMaxMap = prefs.getInt("XRMaxMap", 15);
  YRMinMap = prefs.getInt("YRMinMap", 15);
  YRMaxMap = prefs.getInt("YRMaxMap", -15);
  prefs.end();
}

void resetStoredValues() {
  prefs.begin("joy", false);
  prefs.clear();  // 🔄 ลบทุกค่าที่เคยบันทึกไว้
  prefs.end();

  // ตั้งค่าดีฟอลต์กลับใหม่
  XL_min = YL_min = XR_min = YR_min = 4095;
  XL_max = YL_max = XR_max = YR_max = 0;
  XL_center = YL_center = XR_center = YR_center = 0;

  server.send(200, "text/plain", "Values reset");
}

void mapJoystick() {
  
  if (XL > XL_center + deadzone) {
    XL = map(XL, XL_center + deadzone, XL_max, 0, -XLMaxMap);
  }
  else if (XL < XL_center - deadzone) {
    XL = map(XL, XL_center - deadzone, XL_min, 0, -XLMinMap);
  }
  else {
    XL = 0 ;
  }
  if (XR > XR_center + deadzone) {
    XR = map(XR, XR_center + deadzone, XR_max, 0, -XRMaxMap);
  }
  else if (XR < XR_center - deadzone) {
    XR = map(XR, XR_center - deadzone, XR_min, 0, -XRMinMap);
  }
  else {
    XR = 0 ;
  }
  if (YL > YL_center + deadzone) {
    YL = map(YL, YL_center + deadzone, YL_max, 0, YLMaxMap);
  }
  else if (YL < YL_center - deadzone) {
    YL = map(YL, YL_center - deadzone, YL_min, 0, YLMinMap);
  }
  else {
    YL = 0 ;
  }
  if (YR > YR_center + deadzone) {
    YR = map(YR, YR_center + deadzone, YR_max, 0, YRMaxMap);
  }
  else if (YR < YR_center - deadzone) {
    YR = map(YR, YR_center - deadzone, YR_min, 0, YRMinMap);
  }
  else {
    YR = 0 ;
  }
}

void startWeb() {
  if (wifiActive) return;
  loadStoredValues();  // ✅ โหลดค่าที่บันทึกไว้
  loadMapValues();
  
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  delay(100);
  dnsServer.start(53, "*", apIP);

  server.on("/", []() { server.send_P(200, "text/html", htmlPage); });
  server.on("/data", handleWebData);
  server.on("/save_minmax", saveMinMax);
  server.on("/save_center", saveCenter);
  server.on("/reset_values", resetStoredValues);  // ✅ เพิ่มเส้นทาง reset

  // สำหรับ redirect บนอุปกรณ์
  server.on("/generate_204", []() { server.send(204, "text/plain", ""); });
  server.on("/hotspot-detect.html", []() { server.send(200, "text/html", htmlPage); });
  server.on("/gen_204", []() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });
  server.on("/ncsi.txt", []() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });
  server.on("/connecttest.txt", []() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });
  server.onNotFound([]() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });

  server.on("/save_map", []() {
    prefs.begin("map", false);
    XLMinMap = server.arg("XLMinMap").toInt(); prefs.putInt("XLMinMap", XLMinMap);
    XLMaxMap = server.arg("XLMaxMap").toInt(); prefs.putInt("XLMaxMap", XLMaxMap);
    YLMinMap = server.arg("YLMinMap").toInt(); prefs.putInt("YLMinMap", YLMinMap);
    YLMaxMap = server.arg("YLMaxMap").toInt(); prefs.putInt("YLMaxMap", YLMaxMap);
    XRMinMap = server.arg("XRMinMap").toInt(); prefs.putInt("XRMinMap", XRMinMap);
    XRMaxMap = server.arg("XRMaxMap").toInt(); prefs.putInt("XRMaxMap", XRMaxMap);
    YRMinMap = server.arg("YRMinMap").toInt(); prefs.putInt("YRMinMap", YRMinMap);
    YRMaxMap = server.arg("YRMaxMap").toInt(); prefs.putInt("YRMaxMap", YRMaxMap);
    prefs.end();
    server.send(200, "text/plain", "✅ Saved Mapping ✅");
  });

  server.on("/get_map", []() {
    String json = "{";
    json += "\"XLMinMap\":" + String(XLMinMap) + ",";
    json += "\"XLMaxMap\":" + String(XLMaxMap) + ",";
    json += "\"YLMinMap\":" + String(YLMinMap) + ",";
    json += "\"YLMaxMap\":" + String(YLMaxMap) + ",";
    json += "\"XRMinMap\":" + String(XRMinMap) + ",";
    json += "\"XRMaxMap\":" + String(XRMaxMap) + ",";
    json += "\"YRMinMap\":" + String(YRMinMap) + ",";
    json += "\"YRMaxMap\":" + String(YRMaxMap);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/save_mac", []() {
    String mac = server.arg("mac");
    prefs.begin("config", false);
    prefs.putString("ReceiverMAC", mac);
    prefs.end();
    server.send(200, "text/plain", "✅ MAC saved: " + mac);
  });

  server.on("/get_mac", []() {
    prefs.begin("config", true);
    String mac = prefs.getString("ReceiverMAC", "00:00:00:00:00:00");
    prefs.end();
    server.send(200, "text/plain", mac);
  });

  server.begin();
  wifiActive = true;
  inCalibrationMode = true;
  Serial.println("   📶 WebServer Started");
}


void stopWeb() {
  if (!wifiActive) return;
  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  wifiActive = false;
  inCalibrationMode = false;
  Serial.println("   🛑 WebServer Stopped");
}

void detectDoubleClick() {
  static bool lastSWL = false, lastSWR = false;
  bool currSWL = SWL;
  bool currSWR = SWR;
  unsigned long now = millis();

  if (!lastSWL && currSWL) {
    if (now - lastClickSWL < clickInterval) {
      clickCountSWL++;
      if (clickCountSWL == 2) {
        startWeb();
        webCommand = 1;
        clickCountSWL = 0;
      }
    } else clickCountSWL = 1;
    lastClickSWL = now;
  }

  if (!lastSWR && currSWR) {
    if (now - lastClickSWR < clickInterval) {
      clickCountSWR++;
      if (clickCountSWR == 2) {
        stopWeb();
        webCommand = 0;
        clickCountSWR = 0;
      }
    } else clickCountSWR = 1;
    lastClickSWR = now;
  }

  lastSWL = currSWL;
  lastSWR = currSWR;
}

// Arduino-ESP32 core 3.0.7 (ESP-IDF v5.1) expects send callback:
//   void (*)(const uint8_t *mac_addr, esp_now_send_status_t status)
// (this core DOES NOT provide wifi_tx_info_t).
static void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send to ");
  if (mac_addr) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
  } else {
    Serial.print("(null)");
  }
  Serial.print(" : ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(68); // 17 dBm
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  prefs.begin("config", true);
  String macStr = prefs.getString("ReceiverMAC", "00:00:00:00:00:00");
  prefs.end();

  sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
         &ReceiverMAC[0], &ReceiverMAC[1], &ReceiverMAC[2],
         &ReceiverMAC[3], &ReceiverMAC[4], &ReceiverMAC[5]);

  esp_now_register_send_cb(OnSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ReceiverMAC, 6);
  // Keep defaults explicit for stability on ESP32-C3.
  peerInfo.channel = 0;            // 0 = use current channel
  peerInfo.ifidx   = WIFI_IF_STA;  // send from STA interface
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.print("📡 Receiver MAC Set: ");
  Serial.println(macStr);
}

void controller_setup() {
  Serial.begin(115200);
  pinMode(SWL_PIN, INPUT_PULLUP);
  pinMode(SWR_PIN, INPUT_PULLUP);
  loadReceiverMAC();
  loadStoredValues();
  loadMapValues();
  setupESPNow();
}

void controller_loop() {
  XL = readAverage(XL_PIN);
  YL = readAverage(YL_PIN);
  XR = readAverage(XR_PIN);
  YR = readAverage(YR_PIN);
  SWL = !digitalRead(SWL_PIN);
  SWR = !digitalRead(SWR_PIN);
  detectDoubleClick();
  mapJoystick();
  sendESPNow();
  if (wifiActive) {
    dnsServer.processNextRequest();
    server.handleClient();
    Serial.println("   📶 WebServer On");
  }
  else {
    Serial.println("   🛑 WebServer Off");
  }
  delay(loopDelayMs);
} 

// ---- Library wrapper ----
static bool _mbc_started = false;

void MeblockController::begin(const MeblockControllerConfig& cfg) {
  // Copy SSID/PASS into mutable buffers
  if (cfg.apSsid && cfg.apSsid[0]) {
    strlcpy(ssid, cfg.apSsid, sizeof(ssid));
  }
  if (cfg.apPass) {
    strlcpy(password, cfg.apPass, sizeof(password));
  }
  apIP = cfg.apIP;

  if (cfg.deadzone >= 0) deadzone = cfg.deadzone;
  if (cfg.loopDelayMs >= 0) loopDelayMs = cfg.loopDelayMs;

  if (!_mbc_started) {
    controller_setup();
    _mbc_started = true;
  }
}

void MeblockController::update() {
  if (!_mbc_started) {
    // auto start with defaults
    MeblockControllerConfig cfg;
    begin(cfg);
  }
  controller_loop();
}
