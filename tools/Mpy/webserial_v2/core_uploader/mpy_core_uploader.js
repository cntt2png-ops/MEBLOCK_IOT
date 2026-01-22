// mpy_core_uploader.js

class SerialMicroPythonREPL {
  constructor({ onLog = (t)=>console.log(t), onStatus = ()=>{}, paceMs = 2 } = {}) {
    this.onLog = onLog;
    this.onStatus = onStatus;
    this.paceMs = paceMs;

    this.port = null;
    this.reader = null;        // live read loop reader
    this.writer = null;
    this.enc = new TextEncoder();
    this.connected = false;

    this._readAbort = null;
    this._writeQueue = Promise.resolve();
    this._flashAbort = false;
  }

  _requireConnected(){ if(!this.connected || !this.port) throw new Error("Not connected"); }
  async _sleep(ms){ if(!ms||ms<=0) return; return new Promise(r=>setTimeout(r,ms)); }
  _log(t){ if(t) this.onLog(t); }

  async _startReadLoop(){
    if(!this.port || this.reader) return;
    this._readAbort = new AbortController();
    const signal = this._readAbort.signal;
    const reader = this.port.readable.getReader();
    this.reader = reader;
    const td = new TextDecoder();
    this._log("[ReadLoop] started.\n");
    (async ()=>{
      try{
        while(true){
          if(signal.aborted) break;
          const {value, done} = await reader.read();
          if(done) break;
          if(value) this._log(td.decode(value));
        }
      }catch(err){
        if(!signal.aborted) this._log(`[ReadLoop] error: ${err}\n`);
      }finally{
        try{ reader.releaseLock(); }catch{}
        if(this.reader===reader) this.reader=null;
        this._log("[ReadLoop] stopped.\n");
      }
    })();
  }

  async _stopReadLoop(){
    if(!this.reader) return;
    try{ this._readAbort?.abort(); }catch{}
    this._readAbort = null;
    try{ await this.reader.cancel(); }catch{}
    try{ this.reader.releaseLock(); }catch{}
    this.reader = null;
  }

  async connect({ baudRate = 115200 } = {}){
    if(!("serial" in navigator)) throw new Error("Web Serial API not available (use Chrome/Edge)");
    const port = await navigator.serial.requestPort({});
    await port.open({ baudRate });
    this.port = port;
    this.connected = true;
    this.writer = port.writable.getWriter();
    this.onStatus(true);
    this._log(`[Serial] Connected @${baudRate}.\n`);
    await this._startReadLoop();
  }

  async disconnect(){
    await this._stopReadLoop().catch(()=>{});
    try{ if(this.writer) await this.writer.close(); }catch{}
    try{ this.writer?.releaseLock(); }catch{}
    this.writer=null;
    try{ if(this.port) await this.port.close(); }catch{}
    this.port=null;
    this.connected=false;
    this.onStatus(false);
    this._log("[Serial] Disconnected.\n");
  }

  async _sendBytes(bytes, { chunkSize=128 } = {}){
    this._requireConnected();
    const w = this.writer;
    this._writeQueue = this._writeQueue.then(async ()=>{
      for(let i=0;i<bytes.length;i+=chunkSize){
        await w.write(bytes.slice(i, i+chunkSize));
        if(this.paceMs>0) await this._sleep(this.paceMs);
      }
    });
    return this._writeQueue;
  }
  _sendText(t){ return this._sendBytes(this.enc.encode(t)); }

  ctrlA(){ return this._sendBytes(Uint8Array.of(0x01)); }
  ctrlB(){ return this._sendBytes(Uint8Array.of(0x02)); }
  ctrlC(){ return this._sendBytes(Uint8Array.of(0x03)); }
  ctrlD(){ return this._sendBytes(Uint8Array.of(0x04)); }

  abortFlash(){ this._flashAbort = true; }

  async stop(){ this._requireConnected(); await this.ctrlC(); this._log("[CTRL-C]\n"); }

  async reset(){
    this._requireConnected();
    this._log("[machine.reset()]\n");
    await this._exclusiveRaw(async (reader)=>{
      await this._enterRawWait(reader);
      await this._rawExecWait(reader, "import machine\nmachine.reset()", { allowPromptMissing: true, timeoutMs: 1500 });
    });
  }

  static base64FromBytes(bytes){
    let binary = "";
    const step = 0x8000;
    for(let i=0;i<bytes.length;i+=step){
      const sub = bytes.subarray(i, i+step);
      binary += String.fromCharCode.apply(null, sub);
    }
    return btoa(binary);
  }

  static pyQuote(s){
    return "'" + String(s).replace(/\\/g,"\\\\").replace(/'/g,"\\'") + "'";
  }

  async _exclusiveRaw(fn){
    this._requireConnected();
    const wasLooping = !!this.reader;
    await this._stopReadLoop().catch(()=>{});

    const reader = this.port.readable.getReader();
    try{
      return await fn(reader);
    } finally {
      try{ reader.releaseLock(); }catch{}
      if(wasLooping){
        await this._sleep(150);
        await this._startReadLoop().catch(()=>{});
      }
    }
  }

  async _enterRawWait(reader, timeoutMs=2000){
    await this.ctrlC(); await this._sleep(30);
    await this.ctrlA();

    const td = new TextDecoder();
    let buf = "";
    const start = Date.now();
    while(Date.now() - start < timeoutMs){
      const {value, done} = await reader.read();
      if(done) throw new Error("Port closed");
      if(value){
        buf += td.decode(value);
        if(buf.includes("raw REPL") && buf.includes(">")) return;
        if(buf.trimEnd().endsWith(">")) return;
      }
    }
  }

  async _rawExecWait(reader, code, { timeoutMs=6000, allowPromptMissing=false } = {}){
    if(!code.endsWith("\n")) code += "\n";
    await this._sendText(code);
    await this.ctrlD();

    const out = [];
    const err = [];
    let phase = 0; // 0 stdout, 1 stderr, 2 wait prompt
    let gotPrompt = false;
    const start = Date.now();

    while(true){
      if(Date.now() - start > timeoutMs){
        if(allowPromptMissing) return { outText:"", errText:"" };
        throw new Error("Timeout waiting raw REPL response");
      }
      const {value, done} = await reader.read();
      if(done){
        if(allowPromptMissing) return { outText:"", errText:"" };
        throw new Error("Port closed while waiting response");
      }
      if(!value) continue;

      for(let i=0;i<value.length;i++){
        const b = value[i];
        if(phase === 0){
          if(b === 0x04){ phase = 1; continue; }
          out.push(b);
        } else if(phase === 1){
          if(b === 0x04){ phase = 2; continue; }
          err.push(b);
        } else {
          if(b === 0x3E){ gotPrompt = true; break; } // '>'
        }
      }
      if(gotPrompt) break;
    }

    const td = new TextDecoder();
    const outText = td.decode(new Uint8Array(out)).replace(/\r/g,"");
    const errText = td.decode(new Uint8Array(err)).replace(/\r/g,"");

    if(errText.trim()){
      this._log(errText.trimEnd() + "\n");
      const first = errText.trim().split("\n")[0] || "RemoteError";
      throw new Error(first);
    }

    const cleaned = outText.replace(/^OK\s*/g, "").trim();
    if(cleaned) this._log(cleaned + "\n");

    return { outText, errText };
  }

  async flashFiles(files, {
    dstResolver = (file)=>file?.name || "main.py",
    chunkSize = 384,
    autoReset = true,
    onOverallProgress,
  } = {}){
    this._requireConnected();
    const arr = Array.from(files||[]).filter(f => f instanceof File);
    if(!arr.length) throw new Error("No files");
    this._flashAbort = false;

    const metas = arr.map(f => ({ file: f, size: f.size, dst: String(dstResolver(f) || f.name) }));
    const totalAll = metas.reduce((s,m)=>s+(m.size||0), 0);
    let sentAll = 0;

    this._log(`--- FLASH (${metas.length} files, ${totalAll} bytes) ---\n`);

    await this._exclusiveRaw(async (reader)=>{
      await this._enterRawWait(reader);

      const init =
        "import ubinascii, os\n" +
        "def __fw(p,b64,m):\n" +
        "    f=open(p,m)\n" +
        "    f.write(ubinascii.a2b_base64(b64))\n" +
        "    f.close()\n" +
        "def __check(p):\n" +
        "    s=open(p,'r').read()\n" +
        "    compile(s, p, 'exec')\n";
      await this._rawExecWait(reader, init);

      for(let i=0;i<metas.length;i++){
        if(this._flashAbort) throw new Error("FLASH_ABORTED");
        const {file, dst} = metas[i];
        this._log(`--- FILE ${i+1}/${metas.length}: ${file.name} -> ${dst} ---\n`);

        const bytes = new Uint8Array(await file.arrayBuffer());
        let written = 0;

        while(written < bytes.length){
          if(this._flashAbort) throw new Error("FLASH_ABORTED");
          const part = bytes.subarray(written, written + chunkSize);
          const b64 = SerialMicroPythonREPL.base64FromBytes(part);
          const mode = written === 0 ? "wb" : "ab";
          const stmt = `__fw(${SerialMicroPythonREPL.pyQuote(dst)}, ${SerialMicroPythonREPL.pyQuote(b64)}, ${SerialMicroPythonREPL.pyQuote(mode)})`;
          await this._rawExecWait(reader, stmt);

          written += part.length;
          sentAll += part.length;
          onOverallProgress?.({ written: sentAll, total: totalAll });
        }

        await this._rawExecWait(reader, `__check(${SerialMicroPythonREPL.pyQuote(dst)})`);
        this._log(`[OK] ${file.name} -> ${dst}\n`);
      }

      await this._rawExecWait(reader, "import os\ntry:\n    os.sync()\nexcept Exception:\n    pass\nprint('[FLASH_DONE]')\n");

      if(autoReset){
        this._log("[AUTO RESET]\n");
        await this._rawExecWait(reader, "import machine\nmachine.reset()", { allowPromptMissing: true, timeoutMs: 1500 });
      } else {
        await this.ctrlB();
      }
    });

    this._log("[DONE] Flash finished.\n");
  }
}

// ===== UI wiring (giữ nguyên selectors) =====
const $ = (sel)=>document.querySelector(sel);
const logEl = $("#log");

function log(t){
  const d = document.createElement("div");
  d.textContent = t;
  logEl.appendChild(d);
  logEl.scrollTop = logEl.scrollHeight;
}

function setConn(connected){
  $("#connDot").classList.toggle("ok", connected);
  $("#connDot").classList.toggle("bad", !connected);
  $("#connLabel").textContent = connected ? "Connected" : "Disconnected";
  $("#btnConnect").disabled = connected;
  $("#btnDisconnect").disabled = !connected;
  $("#btnStop").disabled = !connected;
  $("#btnReset").disabled = !connected;
  $("#btnFlash").disabled = !connected;
  $("#btnAbort").disabled = !connected;
}

if(!window.isSecureContext && location.hostname !== "localhost"){
  $("#insecureBanner").classList.remove("hide");
}

const repl = new SerialMicroPythonREPL({
  onLog: (t)=>log(t),
  onStatus: (ok)=>setConn(ok),
  paceMs: 3,
});

const state = { files: [], totalBytes: 0 };

function normalizePath(p){
  p = (p || "").replace(/\\/g,"/");
  p = p.replace(/\/+/g,"/");
  p = p.replace(/^\.\//,"");
  return p;
}
function baseNameFromPath(p){
  const s = normalizePath(p);
  const parts = s.split("/");
  return parts[parts.length - 1] || "main.py";
}
function fmtBytes(n){
  if(!Number.isFinite(n)) return "";
  if(n < 1024) return `${n} B`;
  if(n < 1024*1024) return `${(n/1024).toFixed(1)} KB`;
  return `${(n/1024/1024).toFixed(2)} MB`;
}
function rebuildTotal(){
  const pyFiles = state.files.filter(f => f.name.toLowerCase().endsWith(".py"));
  state.totalBytes = pyFiles.reduce((s,f)=>s+f.size,0);
  $("#progOverall").value = 0;
  $("#overallText").textContent = `0 / ${fmtBytes(state.totalBytes)}`;
}

$("#btnClear").addEventListener("click", ()=>{ logEl.innerHTML = ""; });

$("#pickFolder").addEventListener("change", (e)=>{
  const picked = Array.from(e.target.files || []);
  state.files = picked; // folder-only
  rebuildTotal();
  log(`[FOLDER] Selected: ${picked.length} item(s)\n`);
});

$("#btnConnect").addEventListener("click", async ()=>{
  try{
    const baudRate = parseInt($("#baud").value, 10) || 115200;
    await repl.connect({ baudRate });
  }catch(err){
    log(`[ERR] ${err?.message || err}\n`);
  }
});

$("#btnDisconnect").addEventListener("click", async ()=>{
  try{ await repl.disconnect(); }catch(err){ log(`[ERR] ${err?.message || err}\n`); }
});

$("#btnStop").addEventListener("click", async ()=>{
  try{ await repl.stop(); }catch(err){ log(`[ERR] ${err?.message || err}\n`); }
});

$("#btnReset").addEventListener("click", async ()=>{
  try{ await repl.reset(); }catch(err){ log(`[ERR] ${err?.message || err}\n`); }
});

$("#btnAbort").addEventListener("click", ()=>{
  repl.abortFlash();
  log("[ABORT] requested.\n");
});

$("#btnFlash").addEventListener("click", async ()=>{
  if(!repl.connected){ log("[WARN] Chưa connect.\n"); return; }
  const pyFiles = state.files.filter(f => f.name.toLowerCase().endsWith(".py"));
  if(!pyFiles.length){ log("[WARN] Thư mục không có file .py.\n"); return; }

  const autoReset = $("#autoReset").checked;
  const chunkSize = Math.max(128, parseInt($("#chunkSize").value, 10) || 1024);

  $("#progOverall").value = 0;
  $("#overallText").textContent = `0 / ${fmtBytes(state.totalBytes)}`;

  try{
    await repl.flashFiles(pyFiles, {
      dstResolver: (file)=>{
        const src = file.webkitRelativePath ? file.webkitRelativePath : file.name;
        return baseNameFromPath(src); // ROOT (/filename.py) — overwrite
      },
      chunkSize,
      autoReset,
      onOverallProgress: ({written, total})=>{
        $("#progOverall").value = total ? (written/total*100) : 0;
        $("#overallText").textContent = `${fmtBytes(written)} / ${fmtBytes(total)}`;
      }
    });
  }catch(err){
    log(`\n[ERR] ${err?.message || err}\n`);
  }
});

setConn(false);
