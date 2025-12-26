/* ble_mpy_multi_uploader.js
 * Multi-file .py uploader for MicroPython over Web Bluetooth (Nordic UART Service - NUS)
 */
export class BleMicroPythonNUS {
  constructor({ onLog=(t)=>console.log(t), onStatus=(ok,name)=>console.log('status',ok,name), paceMs=3, bleWriteChunk=100 } = {}) {
    this.onLog = onLog; this.onStatus = onStatus; this.paceMs = paceMs; this.bleWriteChunk = bleWriteChunk;
    this.device=null; this.server=null; this.service=null; this.chrRX=null; this.chrTX=null; this.connected=false;
    this._writeQueue=Promise.resolve(); this._boundOnTx=this._onTxNotify.bind(this);
    this.enc=new TextEncoder(); this.dec=new TextDecoder(); this._flashAbort=false;
  }
  static get NUS_SERVICE(){ return '6e400001-b5a3-f393-e0a9-e50e24dcca9e'; }
  static get NUS_RX(){ return '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; }
  static get NUS_TX(){ return '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; }
  static pyQuote(s){ return "'" + String(s).replace(/\\/g,'\\\\').replace(/'/g,"\\'") + "'"; }
  static base64FromBytes(bytes){
    let binary=''; const step=0x8000;
    for(let i=0;i<bytes.length;i+=step){
      const sub=bytes.subarray(i,i+step);
      binary += String.fromCharCode.apply(null, sub);
    }
    return btoa(binary);
  }
  _log(t){ if(t) this.onLog(t); }
  _requireConnected(){ if(!this.connected||!this.device||!this.chrRX) throw new Error('BLE not connected'); }
  async _sleep(ms){ if(!ms||ms<=0) return; return new Promise(r=>setTimeout(r,ms)); }

  async connect({ namePrefix='MEBLOCK-' } = {}){
    if(!navigator.bluetooth) throw new Error('Web Bluetooth API is not available');
    const prefix=String(namePrefix||'MEBLOCK-');
    const opts={ filters:[{ namePrefix: prefix }], optionalServices:[BleMicroPythonNUS.NUS_SERVICE] };
    this._log(`Requesting device with prefix '${prefix}'...\n`);
    this.device = await navigator.bluetooth.requestDevice(opts);
    this.device.addEventListener('gattserverdisconnected', ()=>this._onDisconnected());
    this._log('Connecting to GATT...\n');
    this.server = await this.device.gatt.connect();
    this.service = await this.server.getPrimaryService(BleMicroPythonNUS.NUS_SERVICE);
    this.chrRX = await this.service.getCharacteristic(BleMicroPythonNUS.NUS_RX);
    this.chrTX = await this.service.getCharacteristic(BleMicroPythonNUS.NUS_TX);
    await this.chrTX.startNotifications();
    this.chrTX.addEventListener('characteristicvaluechanged', this._boundOnTx);
    this.connected=true; this.onStatus(true, this.device.name||''); this._log('[BLE] Connected.\n');
  }

  async disconnect(){
    if(!this.device) return;
    try{
      if(this.chrTX){
        this.chrTX.removeEventListener('characteristicvaluechanged', this._boundOnTx);
        try{ await this.chrTX.stopNotifications(); }catch{}
      }
    }catch{}
    try{ if(this.server && this.server.connected) await this.server.disconnect(); }catch{}
    this.device=null; this.server=null; this.service=null; this.chrRX=null; this.chrTX=null; this.connected=false;
    this.onStatus(false,''); this._log('[BLE] Disconnected.\n');
  }

  _onDisconnected(){ this.connected=false; this.onStatus(false,''); this._log('[BLE] Disconnected (remote).\n'); }
  _onTxNotify(ev){ this._log(this.dec.decode(ev.target.value)); }

  async _sendBytes(bytes, { chunkSize } = {}){
    this._requireConnected();
    const chr=this.chrRX; const sliceSize=chunkSize||this.bleWriteChunk;
    this._writeQueue = this._writeQueue.then(async ()=>{
      for(let i=0;i<bytes.length;i+=sliceSize){
        const slice = bytes.slice(i,i+sliceSize);
        await chr.writeValue(slice);
        if(this.paceMs>0) await this._sleep(this.paceMs);
      }
    });
    return this._writeQueue;
  }
  async _sendText(text){ return this._sendBytes(this.enc.encode(text)); }

  async ctrlC(){ await this._sendBytes(new Uint8Array([0x03])); await this._sleep(30); }
  async ctrlA(){ await this._sendBytes(new Uint8Array([0x01])); await this._sleep(30); }
  async ctrlB(){ await this._sendBytes(new Uint8Array([0x02])); await this._sleep(30); }
  async ctrlD(){ await this._sendBytes(new Uint8Array([0x04])); await this._sleep(30); }

  async enterRaw(){ this._requireConnected(); await this.ctrlC(); await this.ctrlA(); await this._sleep(60); }
  async rawRun(code){
    this._requireConnected();
    await this._sendText(code.endsWith('\n')?code:(code+'\n'));
    await this.ctrlD();
    await this._sleep(60);
  }
  async stop(){ await this.ctrlC(); }
  async reset(){ await this.enterRaw(); await this.rawRun('import machine\nmachine.reset()\n'); }
  abortFlash(){ this._flashAbort=true; }

  static normalizePath(p){
    let s=String(p||'').replace(/\\/g,'/'); s=s.replace(/\/+/g,'/'); s=s.replace(/^\.\//,'');
    while(s.startsWith('../')) s=s.slice(3);
    return s.replace(/\/+/g,'/');
  }
  static joinUnix(a,b){
    const A=BleMicroPythonNUS.normalizePath(a); const B=BleMicroPythonNUS.normalizePath(b);
    if(!A) return B; if(!B) return A;
    return (A.endsWith('/')?A.slice(0,-1):A) + '/' + (B.startsWith('/')?B.slice(1):B);
  }
  static baseName(path){ const s=BleMicroPythonNUS.normalizePath(path); const parts=s.split('/'); return parts[parts.length-1]||'main.py'; }
  static dirName(path){ const s=BleMicroPythonNUS.normalizePath(path); const i=s.lastIndexOf('/'); return i>0?s.slice(0,i):''; }

  async flashFiles(files, {
    basePath='',  // mặc định nạp vào root (/)
    preserveFolders=true,
    flattenToRoot=false,
    overwrite=true,
    autoReset=false,
    fileChunkSize=512,
    onOverallProgress,
    onFileStart,
    onFileDone,
  } = {}){
    this._requireConnected();
    const list = Array.from(files||[]).filter(f => (f instanceof File) && f.name.toLowerCase().endsWith('.py'));
    if(!list.length) throw new Error('No .py files');
    this._flashAbort=false;

    const total = list.reduce((s,f)=>s+(f.size||0),0);
    let sent=0;

    const bp=(basePath||'').trim();
    const base = (!bp || bp==='/' ) ? '' : (bp.startsWith('/')?bp:('/'+bp));

    const helper = `
import ubinascii, os
def __mkdir_p(d):
    if not d: return
    if d[0] != '/': d='/' + d
    parts=[p for p in d.split('/') if p]
    cur='/'
    for p in parts:
        cur = cur + p
        try: os.mkdir(cur)
        except OSError: pass
        cur = cur + '/'
def __fw(path, data_b64, mode):
    b = ubinascii.a2b_base64(data_b64)
    f = open(path, mode)
    f.write(b)
    f.close()
`;
    this._log(`--- FLASH START (${list.length} files, ${total} bytes) ---\n`);
    await this.enterRaw();
    await this.rawRun(helper);

    function relPath(file){
      const w=file.webkitRelativePath;
      if(w){
        const s=BleMicroPythonNUS.normalizePath(w);
        const k=s.indexOf('/');
        return k>=0 ? s.slice(k+1) : s;
      }
      return file.name;
    }

    for(let i=0;i<list.length;i++){
      if(this._flashAbort) throw new Error('FLASH_ABORTED');
      const f=list[i];
      const srcRel = preserveFolders ? relPath(f) : f.name;
      const dstRel = flattenToRoot ? BleMicroPythonNUS.baseName(srcRel) : BleMicroPythonNUS.normalizePath(srcRel);
      const dst = base ? BleMicroPythonNUS.joinUnix(base, dstRel) : ('/' + dstRel.replace(/^\/+/, ''));
      const dir = BleMicroPythonNUS.dirName(dst);

      onFileStart?.({ index:i, totalFiles:list.length, file:f, dst });
      this._log(`\n[FILE ${i+1}/${list.length}] ${f.name} -> ${dst}\n`);
      if(dir) await this.rawRun(`__mkdir_p(${BleMicroPythonNUS.pyQuote(dir)})\n`);

      const buf=new Uint8Array(await f.arrayBuffer());
      let written=0; let chunkIdx=0;

      while(written<buf.length){
        if(this._flashAbort) throw new Error('FLASH_ABORTED');
        const part=buf.subarray(written, written+fileChunkSize);
        const b64=BleMicroPythonNUS.base64FromBytes(part);
        const mode = (written===0) ? 'wb' : 'ab';
        const stmt = `__fw(${BleMicroPythonNUS.pyQuote(dst)}, ${BleMicroPythonNUS.pyQuote(b64)}, ${BleMicroPythonNUS.pyQuote(mode)})\n`;
        await this.rawRun(stmt);

        written += part.length;
        sent += part.length;
        chunkIdx++;

        onOverallProgress?.({ written: sent, total, file:f, fileWritten:written, fileTotal:buf.length });

        if(chunkIdx % 8 === 0 || written >= buf.length){
          this._log(`  - ${Math.min(100, Math.round(written/buf.length*100))}% (${written}/${buf.length})\n`);
        }
        await this._sleep(10);
      }

      onFileDone?.({ index:i, totalFiles:list.length, file:f, dst });
      this._log('[OK]\n');
    }

    await this.rawRun("try:\n  os.sync()\nexcept Exception:\n  pass\nprint('[FLASH_DONE]')\n");

    if(autoReset){
      this._log('\n[AUTO RESET]\n');
      await this.rawRun('import machine\nmachine.reset()\n');
    }else{
      await this.ctrlB();
    }
    this._log('\n--- FLASH DONE ---\n');
  }
}
