#include "EspAiMultiStream.h"

EspAiMultiStream* _multiInstance = nullptr;

const char CAMERA_INDEX_HTML[] PROGMEM = R"raw(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <title>AI CAMERA</title>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <script src='https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@latest/dist/tf.min.js'></script>
  <script src='https://cdn.jsdelivr.net/npm/@teachablemachine/image@latest/dist/teachablemachine-image.min.js'></script>
  <style>
    body{background:#111;color:#eee;font-family:sans-serif;display:flex;flex-direction:column;align-items:center;margin:0;height:100vh} 
    #container{position:relative;border:2px solid #555;border-radius:4px;overflow:hidden;max-width:640px;width:95%} 
    img{width:100%;display:block} 
    #fps{position:absolute;top:10px;left:10px;background:rgba(0,0,0,0.6);color:#0f0;font-size:16px;padding:5px;border-radius:4px;font-weight:bold} 
    #label-container{background:#222;padding:10px;width:95%;max-width:640px;margin-top:5px;border-radius:4px} 
    .bar-row{display:flex;align-items:center;margin-bottom:4px;font-size:14px} 
    .bar-bg{flex-grow:1;background:#444;height:15px;margin:0 10px;border-radius:4px} 
    .bar-fill{height:100%;background:#0f0;width:0%;border-radius:4px;transition:width 0.1s}
  </style>
</head>
<body>
  <h2>AI CAMERA</h2>
  <div id='container'><div id="fps">FPS: 0</div><img id='stream' src=''></div>
  <div id='label-container'>Loading AI...</div>
  <script>
    const MODEL_URL = 'REPLACE_THIS_WITH_REAL_LINK';
    let model, maxPredictions, ws;
    const img = document.getElementById('stream');
    let frameCount=0, lastTime=Date.now();

    async function initAI(){
      try {
        model = await tmImage.load(MODEL_URL+'model.json', MODEL_URL+'metadata.json');
        maxPredictions = model.getTotalClasses();
        const lc = document.getElementById('label-container'); lc.innerHTML = '';
        for(let i=0; i<maxPredictions; i++) lc.innerHTML += `<div class='bar-row'><div style='width:100px'>Class ${i}</div><div class='bar-bg'><div class='bar-fill' id='b-${i}'></div></div><div id='v-${i}'>0%</div></div>`;
        connect(); loopAI();
      } catch(e){ document.getElementById('label-container').innerHTML = 'Lỗi Model: '+e; }
    }

    async function loopAI() {
      if (model && img.complete && img.naturalHeight !== 0) {
        const prediction = await model.predict(img);
        let maxProb = 0, bestClass = -1;
        for(let i=0; i<maxPredictions; i++){
          if(prediction[i].probability > maxProb) { maxProb = prediction[i].probability; bestClass = i; }
          const p = Math.round(prediction[i].probability * 100);
          document.getElementById('b-'+i).style.width = p + '%';
          document.getElementById('v-'+i).innerText = p + '%';
        }
        if (ws && ws.readyState === WebSocket.OPEN && maxProb > 0.7) ws.send(bestClass.toString());
      }
      requestAnimationFrame(loopAI);
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
      ws.onclose = function() { setTimeout(connect, 1000); };
    }
    initAI();
  </script>
</body>
</html>
)raw";


const char AUDIO_INDEX_HTML[] PROGMEM = R"raw(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'> <title>AI AUDIO</title>
  <script src="https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@latest/dist/tf.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/@tensorflow-models/speech-commands@latest/dist/speech-commands.min.js"></script>
  <style>
    body{background:#111;color:#eee;font-family:monospace;text-align:center;padding:20px;display:flex;flex-direction:column;align-items:center} 
    #status{border:2px solid #0f0;padding:15px;margin:20px;cursor:pointer;border-radius:5px;width:300px;font-weight:bold} 
    #meter{width:300px;height:20px;background:#333;margin:10px 0;border-radius:4px;overflow:hidden} 
    #fill{height:100%;background:#0f0;width:0%}
    #debug{color:#888;font-size:12px;margin-top:20px}
  </style>
</head>
<body>
  <h2>AI AUDIO </h2>
  <div id="status">START</div>
  <div id="meter"><div id="fill"></div></div>
  <div id='label-container'>Loading...</div>
  <div id="debug">Waiting...</div>
  <script>
    const MODEL_URL = "REPLACE_THIS_WITH_REAL_LINK";
    let recognizer, ws, audioCtx, dest, samples=[];

    async function init() {
      const s = document.getElementById('status');
      try {
        if (typeof navigator.mediaDevices === 'undefined') navigator.mediaDevices = {};
        if (typeof navigator.mediaDevices.getUserMedia === 'undefined') {
            navigator.mediaDevices.getUserMedia = function(c) {
                const ac = new (window.AudioContext || window.webkitAudioContext)();
                return Promise.resolve(ac.createMediaStreamDestination().stream);
            };
        }

        recognizer = speechCommands.create("BROWSER_FFT", undefined, MODEL_URL+"model.json", MODEL_URL+"metadata.json");
        await recognizer.ensureModelLoaded();
        document.getElementById('label-container').innerHTML = recognizer.wordLabels().join(', ');
        s.onclick = start;
      } catch(e) { s.innerText = "Lỗi Init: " + e.message; }
    }

    async function start() {
      if(audioCtx) return;
      const s = document.getElementById('status');
      try {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)({ sampleRate: 16000 });
        dest = audioCtx.createMediaStreamDestination();
        
        await navigator.mediaDevices.getUserMedia({ audio: true }); 
        
        const node = audioCtx.createScriptProcessor(2048, 1, 1);
        node.onaudioprocess = (e) => {
          const out = e.outputBuffer.getChannelData(0);
          let sum = 0;
          for (let i=0; i<out.length; i++) {
            out[i] = samples.shift() || 0; // Lấy dữ liệu từ ESP32
            sum += out[i]*out[i];
          }
          document.getElementById('fill').style.width = Math.min(Math.sqrt(sum/out.length)*500, 100) + "%";
        };
        node.connect(dest);

        recognizer.listen(result => {
          let maxIdx = result.scores.indexOf(Math.max(...result.scores));
          if(ws && ws.readyState === WebSocket.OPEN && result.scores[maxIdx] > 0.85) {
             ws.send(maxIdx.toString());
             document.getElementById('debug').innerText = "Detected: " + maxIdx + " (" + Math.round(result.scores[maxIdx]*100) + "%)";
          }
        }, { probabilityThreshold: 0.85, overlapFactor: 0.5 });
        
        connectWS();
        s.innerText = "MIC ESP32 RUN";
        s.style.borderColor = "#fff";
        s.style.color = "#0f0";
      } catch (err) { s.innerText = "Lỗi Start: " + err.message; }
    }

    function connectWS() {
      ws = new WebSocket('ws://' + window.location.hostname + '/ws/audio');
      ws.binaryType = 'arraybuffer';
      ws.onmessage = (e) => {
        const in16 = new Int16Array(e.data);
        for(let i=0; i<in16.length; i++) samples.push((in16[i]/32768.0));
        if(samples.length > 8000) samples.splice(0, 4000); 
      };
      ws.onclose = () => { console.log("WS Closed"); setTimeout(connectWS, 1000); };
    }
    init();
  </script>
</body>
</html>
)raw";


EspAiMultiStream::EspAiMultiStream() 
    : server(80), 
      wsCamera("/ws/cam"), 
      wsAudio("/ws/audio") 
{
    _latestCamResult = -1; 
    _latestAudioResult = -1;
    _cameraEnabled = false; 
    _audioEnabled = false;
    _multiInstance = this;
    _lastCamTime = 0;
}

void EspAiMultiStream::beginWiFi(const char* ssid, const char* pass) {
    WiFi.begin(ssid, pass);
    WiFi.setTxPower(WIFI_POWER_11dBm); 
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) { 
        delay(500); Serial.print("."); 
        retry++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWIFI CONNECTED");
    } else {
        Serial.println("\nWIFI ERROR: Check Pass/Power");
    }
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
    
    config.xclk_freq_hz = 20000000; 
    config.pixel_format = PIXFORMAT_JPEG;
    
    if(psramFound()){
      config.frame_size = FRAMESIZE_VGA; 
      config.jpeg_quality = 10;          
      config.fb_count = 2;               
    } else {
      config.frame_size = FRAMESIZE_QVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera Init Failed!");
        return false;
    }
    sensor_t * s = esp_camera_sensor_get();
    if (s) { s->set_vflip(s, flipV); s->set_hmirror(s, flipH); }
    
    _cameraEnabled = true; 
    return true;
}

bool EspAiMultiStream::beginAudio(String tmAudioModelLink) {
    _tmAudioModelLink = tmAudioModelLink;
    i2s_config_t config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64, 
        .use_apll = false
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = 14, 
        .ws_io_num = 21, 
        .data_out_num = -1, 
        .data_in_num = 2
    };
    
    if (i2s_driver_install(I2S_NUM_0, &config, 0, NULL) != ESP_OK) return false;
    i2s_set_pin(I2S_NUM_0, &pin_config);
    _audioEnabled = true; 
    return true;
}

void EspAiMultiStream::beginServer() {
    wsCamera.onEvent([](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* a, uint8_t* d, size_t l){
       if(t == WS_EVT_DATA && _multiInstance){
           String msg = ""; for(size_t i=0; i<l; i++) msg += (char)d[i];
           _multiInstance->_latestCamResult = msg.toInt();
       }
    });
    server.addHandler(&wsCamera);

    wsAudio.onEvent([](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* a, uint8_t* d, size_t l){
       if(t == WS_EVT_DATA && _multiInstance){
           String msg = ""; for(size_t i=0; i<l; i++) msg += (char)d[i];
           _multiInstance->_latestAudioResult = msg.toInt();
       }
    });
    server.addHandler(&wsAudio);

    server.on("/camera", HTTP_GET, [this](AsyncWebServerRequest * req) {
        String h = String(CAMERA_INDEX_HTML);
        h.replace("REPLACE_THIS_WITH_REAL_LINK", _tmCamModelLink);
        req->send(200, "text/html", h);
    });

    server.on("/audio", HTTP_GET, [this](AsyncWebServerRequest * req) {
        String h = String(AUDIO_INDEX_HTML);
        h.replace("REPLACE_THIS_WITH_REAL_LINK", _tmAudioModelLink);
        req->send(200, "text/html", h);
    });

    server.begin();
}

void EspAiMultiStream::loop() {
    wsCamera.cleanupClients();
    wsAudio.cleanupClients();

    unsigned long now = millis();

    if (_cameraEnabled && wsCamera.count() > 0) {
        if (now - _lastCamTime >= 33) { 
            camera_fb_t * fb = esp_camera_fb_get();
            if (fb) {
                wsCamera.binaryAll(fb->buf, fb->len);
                esp_camera_fb_return(fb);
                _lastCamTime = now; 
            }
        }
    }

    if (_audioEnabled && wsAudio.count() > 0) {
        static int16_t audioBuf[512]; 
        size_t bytesRead = 0;
        esp_err_t result = i2s_read(I2S_NUM_0, &audioBuf, sizeof(audioBuf), &bytesRead, 0);
        
        if (result == ESP_OK && bytesRead > 0) {
            if (wsAudio.availableForWriteAll()) {
                wsAudio.binaryAll((uint8_t*)audioBuf, bytesRead);
            }
        }
    }
}

int EspAiMultiStream::getCamAiResult() { 
    int t = _latestCamResult; _latestCamResult = -1; return t; 
}

int EspAiMultiStream::getAudioAiResult() { 
    int t = _latestAudioResult; _latestAudioResult = -1; return t; 
}