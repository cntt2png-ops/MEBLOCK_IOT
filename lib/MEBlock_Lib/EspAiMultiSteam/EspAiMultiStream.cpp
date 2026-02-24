#include "EspAiMultiStream.h"

EspAiMultiStream* _multiInstance = nullptr;

const char CAMERA_INDEX_HTML[] PROGMEM = R"raw(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'> <title>AI CAMERA MONITOR</title>
  <script src='https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@latest/dist/tf.min.js'></script>
  <script src='https://cdn.jsdelivr.net/npm/@teachablemachine/image@latest/dist/teachablemachine-image.min.js'></script>
  <style>
    body{background:#111;color:#eee;font-family:sans-serif;display:flex;flex-direction:column;align-items:center;margin:0}
    #container{position:relative;width:95%;max-width:640px;border:2px solid #555;margin-top:10px}
    img{width:100%;height:auto;display:block}
    #fps{position:absolute;top:10px;left:10px;color:#0f0;font-weight:bold;background:rgba(0,0,0,0.5);padding:5px}
    #label-container{width:95%;max-width:640px;background:#222;padding:10px;margin-top:5px}
    .bar-row{display:flex;align-items:center;margin-bottom:5px}
    .bar-bg{flex-grow:1;background:#444;height:12px;margin:0 10px;border-radius:6px;overflow:hidden}
    .bar-fill{height:100%;background:#0f0;width:0%;transition:width 0.1s}
  </style>
</head>
<body>
  <h2>AI CAMERA MEBLOCK</h2>
  <div id='container'><div id="fps">FPS: 0</div><img id='stream' src=''></div>
  <div id='label-container'>Loading AI Model...</div>
  <script>
    const MODEL_URL = 'REPLACE_THIS_WITH_REAL_LINK';
    let model, ws, img = document.getElementById('stream');
    let frameCount=0, lastTime=Date.now(), isProcessing = false;
    async function init(){
      try {
        model = await tmImage.load(MODEL_URL+'model.json', MODEL_URL+'metadata.json');
        const lc = document.getElementById('label-container'); lc.innerHTML = '';
        for(let i=0; i<model.getTotalClasses(); i++) 
          lc.innerHTML += `<div class='bar-row'><div style='width:90px'>${model.getClassLabels()[i]}</div><div class='bar-bg'><div class='bar-fill' id='b-${i}'></div></div><div id='v-${i}'>0%</div></div>`;
        connect(); setInterval(predict, 60);
      } catch(e){ alert("Model Load Error: "+e); }
    }
    async function predict() {
      if (model && img.complete && img.naturalHeight !== 0 && !isProcessing) {
        isProcessing = true;
        const pred = await model.predict(img);
        let maxP = 0, bestL = "";
        for(let i=0; i<pred.length; i++) {
          document.getElementById('b-'+i).style.width = Math.round(pred[i].probability*100) + '%';
          document.getElementById('v-'+i).innerText = Math.round(pred[i].probability*100) + '%';
          if(pred[i].probability > maxP) { maxP = pred[i].probability; bestL = pred[i].className; }
        }
        if (ws && ws.readyState === WebSocket.OPEN && maxP > 0.8) {
           ws.send(bestL + ":" + Math.round(maxP*100));
        }
        isProcessing = false;
      }
    }
    function connect(){
      ws = new WebSocket('ws://' + window.location.hostname + '/ws/cam');
      ws.binaryType = 'blob';
      ws.onmessage = function(e) {
        if (img.src) URL.revokeObjectURL(img.src);
        img.src = URL.createObjectURL(e.data);
        frameCount++;
        if(Date.now()-lastTime>=1000){ document.getElementById('fps').innerText="FPS: "+frameCount; frameCount=0; lastTime=Date.now(); }
      };
      ws.onclose = () => setTimeout(connect, 1000);
    }
    init();
  </script>
</body>
</html>
)raw";

const char AUDIO_INDEX_HTML[] PROGMEM = R"raw(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'> <title>AI AUDIO DEBUG</title>
  <script src="https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@latest/dist/tf.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/@tensorflow-models/speech-commands@latest/dist/speech-commands.min.js"></script>
  <style>
    body{background:#111;color:#eee;font-family:monospace;text-align:center;padding:20px;display:flex;flex-direction:column;align-items:center} 
    #status{border:2px solid #0f0;padding:15px;margin:10px;cursor:pointer;border-radius:5px;width:300px;text-transform:uppercase} 
    #meter{width:300px;height:20px;background:#333;margin:10px 0;border-radius:4px;overflow:hidden} 
    #fill{height:100%;background:#0f0;width:0%}
    #debug-board{background:#222;padding:15px;border-radius:8px;width:320px;text-align:left;border:1px solid #444}
    .label-row{display:flex;justify-content:space-between;margin-bottom:5px;font-size:14px}
    .high{color:#0f0;font-weight:bold}
  </style>
</head>
<body>
  <h2>AI AUDIO MEBLOCK</h2>
  <div id="status">CLICK TO START</div>
  <div id="meter"><div id="fill"></div></div>
  <div id="debug-board">Loading model...</div>
  <script>
    const MODEL_URL = "REPLACE_THIS_WITH_REAL_LINK";
    let recognizer, ws, audioCtx, samples=[];
    const GAIN = 4.0;

    async function init() {
      try {
        recognizer = speechCommands.create("BROWSER_FFT", undefined, MODEL_URL+"model.json", MODEL_URL+"metadata.json");
        await recognizer.ensureModelLoaded();
        document.getElementById('debug-board').innerHTML = "Ready. Click START to begin.";
        document.getElementById('status').onclick = start;
      } catch(e) { document.getElementById('debug-board').innerText = "Error: " + e.message; }
    }

    async function start() {
      if(audioCtx) return;
      audioCtx = new AudioContext({ sampleRate: 16000 });
      const dest = audioCtx.createMediaStreamDestination();
      const node = audioCtx.createScriptProcessor(2048, 1, 1);
      
      node.onaudioprocess = (e) => {
        const out = e.outputBuffer.getChannelData(0);
        if (samples.length > 6000) samples.splice(0, samples.length - 2048);
        
        let sum = 0;
        for (let i=0; i<out.length; i++) {
          let v = (samples.shift() || 0) * GAIN;
          out[i] = v; sum += v*v;
        }
        document.getElementById('fill').style.width = Math.min(Math.sqrt(sum/out.length)*200, 100) + "%";
      };
      node.connect(dest);
      //node.connect(audioCtx.destination); Mở cmt nếu muốn nghe âm thanh từ mic i2s phát ra pc  

      recognizer.listen(result => {
        const labels = recognizer.wordLabels();
        let html = "";
        let maxP = 0, bestL = "";
        
        for (let i = 0; i < labels.length; i++) {
          const p = Math.round(result.scores[i] * 100);
          const isHigh = p > 70 ? "class='high'" : "";
          html += `<div class='label-row' ${isHigh}><span>${labels[i]}</span><span>${p}%</span></div>`;
          if(result.scores[i] > maxP) { maxP = result.scores[i]; bestL = labels[i]; }
        }
        document.getElementById('debug-board').innerHTML = html;
        if(ws && ws.readyState === 1 && maxP > 0.6) ws.send(bestL + ":" + Math.round(maxP*100));
      }, { probabilityThreshold: 0.5, overlapFactor: 0.5, stream: dest.stream });

      connectWS();
      document.getElementById('status').innerText = "LISTENING TO ESP32...";
    }

    function connectWS() {
      ws = new WebSocket('ws://' + window.location.hostname + '/ws/audio');
      ws.binaryType = 'arraybuffer';
      ws.onmessage = (e) => {
        const in16 = new Int16Array(e.data);
        for(let i=0; i<in16.length; i++) samples.push(in16[i]/32768.0);
      };
      ws.onclose = () => setTimeout(connectWS, 1000);
    }
    init();
  </script>
</body>
</html>
)raw";

EspAiMultiStream::EspAiMultiStream() : server(80), wsCamera("/ws/cam"), wsAudio("/ws/audio") {
    _latestCamResult = ""; _latestCamScore = 0;
    _latestAudioResult = ""; _latestAudioScore = 0;
    _cameraEnabled = false; _audioEnabled = false; _multiInstance = this; _lastCamTime = 0;
}

void EspAiMultiStream::beginWiFi(const char* ssid, const char* pass) {
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWIFI CONNECTED: " + WiFi.localIP().toString());
}

bool EspAiMultiStream::beginCamera(String tmCamModelLink, bool flipV, bool flipH) {
    _tmCamModelLink = tmCamModelLink;
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 11; config.pin_d1 = 9; config.pin_d2 = 8; config.pin_d3 = 10;
    config.pin_d4 = 12; config.pin_d5 = 18; config.pin_d6 = 17; config.pin_d7 = 16;
    config.pin_xclk = 15; config.pin_pclk = 13; config.pin_vsync = 6;
    config.pin_href = 7; config.pin_sccb_sda = 4; config.pin_sccb_scl = 5;
    config.pin_pwdn = -1; config.pin_reset = -1;
    config.xclk_freq_hz = 20000000; config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA; config.jpeg_quality = 20; config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    if (esp_camera_init(&config) != ESP_OK) return false;
    sensor_t * s = esp_camera_sensor_get();
    if (s) { s->set_vflip(s, flipV); s->set_hmirror(s, flipH); }
    _cameraEnabled = true; return true;
}

bool EspAiMultiStream::beginAudio(String tmAudioModelLink) {
    _tmAudioModelLink = tmAudioModelLink;
    i2s_config_t config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = 8, .dma_buf_len = 128, .use_apll = false
    };
    i2s_pin_config_t pin_config = { .bck_io_num = 14, .ws_io_num = 21, .data_out_num = -1, .data_in_num = 2 };
    if (i2s_driver_install(I2S_NUM_0, &config, 0, NULL) != ESP_OK) return false;
    i2s_set_pin(I2S_NUM_0, &pin_config);
    _audioEnabled = true; return true;
}

void EspAiMultiStream::beginServer() {
    // Logic tách chuỗi Nhan:Score
    auto parseMsg = [](void* arg, uint8_t* d, size_t l, String& res, int& score) {
        String msg = ""; for(size_t i=0; i<l; i++) msg += (char)d[i];
        int colonIdx = msg.indexOf(':');
        if(colonIdx != -1) {
            res = msg.substring(0, colonIdx);
            score = msg.substring(colonIdx + 1).toInt();
        }
    };

    wsCamera.onEvent([parseMsg](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* a, uint8_t* d, size_t l){
       if(t == WS_EVT_DATA && _multiInstance) 
           parseMsg(nullptr, d, l, _multiInstance->_latestCamResult, _multiInstance->_latestCamScore);
    });
    wsAudio.onEvent([parseMsg](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* a, uint8_t* d, size_t l){
       if(t == WS_EVT_DATA && _multiInstance)
           parseMsg(nullptr, d, l, _multiInstance->_latestAudioResult, _multiInstance->_latestAudioScore);
    });

    server.addHandler(&wsCamera); server.addHandler(&wsAudio);
    server.on("/camera", HTTP_GET, [this](AsyncWebServerRequest * req) {
        String h = String(CAMERA_INDEX_HTML); h.replace("REPLACE_THIS_WITH_REAL_LINK", _tmCamModelLink);
        req->send(200, "text/html", h);
    });
    server.on("/audio", HTTP_GET, [this](AsyncWebServerRequest * req) {
        String h = String(AUDIO_INDEX_HTML); h.replace("REPLACE_THIS_WITH_REAL_LINK", _tmAudioModelLink);
        req->send(200, "text/html", h);
    });
    server.begin();
}

void EspAiMultiStream::loop() {
    wsCamera.cleanupClients(); wsAudio.cleanupClients();
    unsigned long now = millis();
    if (_cameraEnabled && wsCamera.count() > 0 && wsCamera.availableForWriteAll()) {
        if (now - _lastCamTime >= 40) {
            camera_fb_t * fb = esp_camera_fb_get();
            if (fb) { wsCamera.binaryAll(fb->buf, fb->len); esp_camera_fb_return(fb); _lastCamTime = now; }
        }
    }
    if (_audioEnabled && wsAudio.count() > 0 && wsAudio.availableForWriteAll()) {
        static int16_t audioBuf[1024]; 
        size_t bytesRead = 0;
        esp_err_t res = i2s_read(I2S_NUM_0, &audioBuf, sizeof(audioBuf), &bytesRead, 0);
        if (res == ESP_OK && bytesRead > 0) {
            wsAudio.binaryAll((uint8_t*)audioBuf, bytesRead);
        }
    }
}

String EspAiMultiStream::getCamAiResult() { String t = _latestCamResult; _latestCamResult = ""; return t; }
int EspAiMultiStream::getCamAiScore() { return _latestCamScore; }
String EspAiMultiStream::getAudioAiResult() { String t = _latestAudioResult; _latestAudioResult = ""; return t; }
int EspAiMultiStream::getAudioAiScore() { return _latestAudioScore; }