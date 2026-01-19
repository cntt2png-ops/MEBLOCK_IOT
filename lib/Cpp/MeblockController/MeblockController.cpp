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
char ssid[33] = "MEDrone Controller V1";
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
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>MEDrone Controller Web V1</title>
<style>
  :root{
    --bg0:#0b2a4b;
    --bg1:#0a2240;
    --card:#10365f;
    --card2:#0f2f54;
    --stroke:rgba(255,255,255,.10);
    --stroke2:rgba(255,255,255,.16);
    --text:#ffffff;
    --muted:rgba(255,255,255,.70);
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
  .wrap{max-width:1200px;margin:0 auto;padding:14px}
  .card{
    background: linear-gradient(180deg, rgba(255,255,255,.06), rgba(255,255,255,.03));
    border:1px solid var(--stroke);
    border-radius: var(--radius);
    padding:14px;
    box-shadow: 0 10px 30px rgba(0,0,0,.20);
  }
  .topbar{
    display:flex;align-items:center;justify-content:space-between;gap:12px;
    padding:14px 16px;
  }
  .brand{font-size:20px;font-weight:800;letter-spacing:.2px}
  .sub{margin-top:4px;color:var(--muted);font-size:12px}
  .pill{
    background: var(--pill);
    border:1px solid var(--stroke2);
    padding:6px 10px;border-radius:999px;
    color:var(--text);font-size:12px;font-weight:700;
  }
  .tabs{display:flex;gap:8px;margin:12px 0}
  .tabbtn{
    border:1px solid var(--stroke2);
    background: rgba(0,0,0,.18);
    color:var(--text);
    padding:8px 12px;border-radius:10px;
    cursor:pointer;font-weight:800;font-size:13px;
  }
  .tabbtn.active{
    background: rgba(47,125,255,.22);
    border-color: rgba(47,125,255,.45);
  }
  .grid{
    display:grid;gap:12px;align-items:start;
    grid-template-columns: 1fr;
  }
  @media(min-width:860px){
    .grid{grid-template-columns: minmax(0,1fr) 420px;}
  }
  h3{margin:0 0 10px 0;font-size:15px;font-weight:900;letter-spacing:.2px}
  .muted{color:var(--muted);font-size:12px;line-height:1.35}
  .status{
    margin-top:10px;
    color:var(--muted);
    font-size:12px;
    min-height: 18px;
  }

  .formgrid{
    display:grid;
    grid-template-columns: repeat(2, minmax(0,1fr));
    gap:10px;
    margin-top:10px;
  }
  .field label{
    display:flex;gap:8px;align-items:baseline;
    color:var(--muted);font-size:12px;margin:0 0 6px 2px;
  }
  .field label b{color:var(--text);font-weight:900}
  input{
    width:100%;
    background: rgba(0,0,0,.18);
    border:1px solid var(--stroke2);
    border-radius:12px;
    padding:10px 12px;
    color:var(--text);
    outline:none;
    font-size:14px;
  }
  input:focus{border-color: rgba(47,125,255,.55); box-shadow:0 0 0 3px rgba(47,125,255,.16)}
  .btnrow{display:flex;flex-wrap:wrap;gap:10px;margin-top:12px}
  .btn{
    border:1px solid var(--stroke2);
    background: rgba(0,0,0,.20);
    color:var(--text);
    padding:10px 12px;
    border-radius:12px;
    cursor:pointer;
    font-weight:900;
    font-size:13px;
  }
  .btn.primary{
    background: rgba(47,125,255,.25);
    border-color: rgba(47,125,255,.55);
  }
  .btn.primary:hover{background: rgba(47,125,255,.33)}
  .btn.danger{
    background: rgba(179,58,58,.28);
    border-color: rgba(179,58,58,.55);
  }
  .btn.danger:hover{background: rgba(179,58,58,.36)}

  .tabpanel{display:none}
  .tabpanel.active{display:block}

  .rightStack{display:flex;flex-direction:column;gap:12px}

  /* Telemetry */
  .teleGrid{
    display:grid;
    grid-template-columns: repeat(2, minmax(0,1fr));
    gap:10px;
  }
  .kv{
    background: rgba(0,0,0,.18);
    border:1px solid var(--stroke2);
    border-radius:12px;
    padding:10px 12px;
  }
  .kv .k{font-size:12px;color:var(--muted)}
  .kv .v{font-size:20px;font-weight:900;margin-top:2px}
  .mini{
    margin-top:10px;
    display:grid;
    grid-template-columns: 1fr 1fr;
    gap:10px;
  }
  .joyBox{
    background: rgba(0,0,0,.16);
    border:1px solid var(--stroke2);
    border-radius:12px;
    padding:10px;
  }
  .joyTitle{font-weight:900;font-size:12px;color:var(--muted);margin:0 0 8px 0}
  .pad{
    position:relative;
    width:170px;height:170px;margin:0 auto;
    border-radius:999px;
    border:1px solid rgba(255,255,255,.18);
    background: radial-gradient(circle at 50% 50%, rgba(47,125,255,.22), rgba(0,0,0,.05) 60%, rgba(0,0,0,0) 72%);
  }
  .centerDot{
    position:absolute;left:50%;top:50%;
    width:10px;height:10px;margin-left:-5px;margin-top:-5px;
    border-radius:999px;background: rgba(255,255,255,.55);
  }
  .handle{
    position:absolute;left:50%;top:50%;
    width:26px;height:26px;margin-left:-13px;margin-top:-13px;
    border-radius:999px;
    background: rgba(47,125,255,.95);
    box-shadow:0 0 0 3px rgba(47,125,255,.20);
    transition: transform .08s linear;
  }
  .miniNums{
    margin-top:8px;
    display:grid;grid-template-columns: repeat(3, 1fr);gap:8px;
    font-size:12px;color:var(--muted);
  }
  .miniNums b{color:var(--text);font-weight:900}
</style>
</head>

<body>
  <div class="wrap">
    <div class="card topbar">
      <div>
        <div class="brand">MEDrone Controller Web V1</div>
        <div class="sub">
          Receiver MAC: <b id="rxMac">--:--:--:--:--:--</b> &nbsp;·&nbsp; AP: <b>192.168.4.1</b>
        </div>
      </div>
      <div class="pill">Controller UI</div>
    </div>

    <div class="tabs">
      <button class="tabbtn active" data-tab="map">Mapping</button>
      <button class="tabbtn" data-tab="mac">MAC</button>
      <button class="tabbtn" data-tab="help">Hướng dẫn</button>
    </div>

    <div class="grid">
      <!-- LEFT: Settings -->
      <div class="card">
        <div id="tab-map" class="tabpanel active">
          <h3>Mapping Parameters</h3>
          <div class="muted">
            Dùng để map raw ADC của joystick sang giá trị điều khiển (ví dụ: roll/pitch/yaw/throttle). Giá trị âm/dương quyết định chiều.
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
            <button class="btn primary" id="btnSaveMap">Lưu mapping</button>
          </div>
        </div>

        <div id="tab-mac" class="tabpanel">
          <h3>Receiver MAC Address</h3>
          <div class="muted">
            Dùng để controller gửi ESP-NOW tới drone. Ví dụ: <b>08:A6:F7:21:BA:6C</b>
          </div>
          <div class="formgrid" style="grid-template-columns: 1fr; margin-top:12px">
            <div class="field">
              <label><b>Receiver MAC</b></label>
              <input type="text" id="macAddress" placeholder="08:A6:F7:21:BA:6C" />
            </div>
          </div>
          <div class="btnrow">
            <button class="btn primary" id="btnSaveMac">Lưu MAC</button>
          </div>
        </div>

        <div id="tab-help" class="tabpanel">
          <h3>Hướng dẫn nhanh</h3>
          <div class="muted">
            <b>Calib Center</b>: để joystick về giữa rồi bấm.<br/>
            <b>Save Min/Max</b>: đảo joystick hết biên 4 góc vài lần rồi bấm.<br/>
            <b>Reset</b>: xoá toàn bộ giá trị đã lưu.<br/><br/>
            UI này đã được đồng bộ style với Web Tuning trên drone.
          </div>
        </div>

        <div id="status" class="status">Status: --</div>
      </div>

      <!-- RIGHT: Telemetry + Calib -->
      <div class="rightStack">
        <div class="card" id="telemetryCard">
          <h3>Telemetry (live)</h3>

          <div class="teleGrid">
            <div class="kv"><div class="k">XL</div><div class="v" id="xl">0</div></div>
            <div class="kv"><div class="k">YL</div><div class="v" id="yl">0</div></div>
            <div class="kv"><div class="k">XR</div><div class="v" id="xr">0</div></div>
            <div class="kv"><div class="k">YR</div><div class="v" id="yr">0</div></div>
          </div>

          <div class="mini">
            <div class="joyBox">
              <div class="joyTitle">Left Joystick</div>
              <div class="pad">
                <div class="centerDot"></div>
                <div class="handle" id="leftHandle"></div>
              </div>
              <div class="miniNums">
                <div>Min <b id="xl_min">0</b></div>
                <div>C <b id="xl_center">0</b></div>
                <div>Max <b id="xl_max">0</b></div>
                <div>Min <b id="yl_min">0</b></div>
                <div>C <b id="yl_center">0</b></div>
                <div>Max <b id="yl_max">0</b></div>
                <div>SWL <b id="swl">0</b></div>
              </div>
            </div>

            <div class="joyBox">
              <div class="joyTitle">Right Joystick</div>
              <div class="pad">
                <div class="centerDot"></div>
                <div class="handle" id="rightHandle"></div>
              </div>
              <div class="miniNums">
                <div>Min <b id="xr_min">0</b></div>
                <div>C <b id="xr_center">0</b></div>
                <div>Max <b id="xr_max">0</b></div>
                <div>Min <b id="yr_min">0</b></div>
                <div>C <b id="yr_center">0</b></div>
                <div>Max <b id="yr_max">0</b></div>
                <div>SWR <b id="swr">0</b></div>
              </div>
            </div>
          </div>
        </div>

        <div class="card" id="calibCard">
          <h3>Calibration</h3>
          <div class="muted">
            Chỉ calib khi drone không bay. Calib Center: để tay thả tự nhiên ở giữa. Save Min/Max: đảo biên 4 góc.
          </div>
          <div class="btnrow">
            <button class="btn primary" id="btnSaveMinMax">Save Min/Max</button>
            <button class="btn primary" id="btnSaveCenter">Calibrate Center</button>
            <button class="btn danger" id="btnResetAll">Reset All</button>
          </div>
        </div>
      </div>
    </div>
  </div>

<script>
  const statusEl = document.getElementById('status');

  function setStatus(msg){
    statusEl.textContent = 'Status: ' + msg;
  }

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
      .then(r=>r.text()).then(t=>setStatus(t || 'Saved'));
  }

  function saveMac(){
    const mac = (document.getElementById('macAddress')?.value || '').trim();
    fetch('/save_mac?mac=' + encodeURIComponent(mac))
      .then(r=>r.text()).then(t=>{
        setStatus(t || 'MAC saved');
        loadMac();
      });
  }

  function saveMinMax(){
    fetch('/save_minmax').then(()=>setStatus('Min/Max saved'));
  }
  function saveCenter(){
    fetch('/save_center').then(()=>setStatus('Center calibrated'));
  }
  function resetAll(){
    fetch('/reset_values').then(()=>setStatus('All values reset'));
  }

  document.getElementById('btnSaveMap')?.addEventListener('click', saveMap);
  document.getElementById('btnSaveMac')?.addEventListener('click', saveMac);
  document.getElementById('btnSaveMinMax')?.addEventListener('click', saveMinMax);
  document.getElementById('btnSaveCenter')?.addEventListener('click', saveCenter);
  document.getElementById('btnResetAll')?.addEventListener('click', resetAll);

  // Init
  window.addEventListener('load', ()=>{
    loadMap();
    loadMac();
    setStatus('Ready');
  });

  // Telemetry refresh
  setInterval(()=>{
    fetch('/data').then(r=>r.json()).then(d=>{
      // Main
      document.getElementById('xl').textContent = d.XL;
      document.getElementById('yl').textContent = d.YL;
      document.getElementById('swl').textContent = d.SWL;

      document.getElementById('xr').textContent = d.XR;
      document.getElementById('yr').textContent = d.YR;
      document.getElementById('swr').textContent = d.SWR;

      // min/max/center
      ['xl_min','xl_max','yl_min','yl_max','xr_min','xr_max','yr_min','yr_max',
       'xl_center','yl_center','xr_center','yr_center'].forEach(id=>{
        if(document.getElementById(id)){
          const key = id.replace('xl','XL').replace('yl','YL').replace('xr','XR').replace('yr','YR')
                        .replace('_min','_min').replace('_max','_max').replace('_center','_center');
          // d has exact keys like XL_min etc
        }
      });

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

static void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  (void)mac_addr;
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
