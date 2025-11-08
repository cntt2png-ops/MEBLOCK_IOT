/**
 * WebBleUploader — Upload file to ESP32 (MicroPython) over Web Bluetooth (Nordic UART)
 * Features:
 *  - Connect/Disconnect by name filter
 *  - ping/ls/reset helpers
 *  - upload(file|ArrayBuffer, target="main.py", {chunk, waitAck, autoReset, onProgress, onLog})
 *  - chunked write (20B) with small delay to keep BLE stable
 *
 * Requirements:
 *  - Serve over HTTPS or http://localhost
 *  - Only one BLE client at a time (close nRF Connect when using this)
 */

export class WebBleUploader {
  /** Nordic UART Service UUIDs */
  static NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
  static NUS_RX      = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // Write
  static NUS_TX      = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // Notify

  /**
   * @param {Object} [opts]
   * @param {(msg:string)=>void} [opts.onLog]       - log hook
   * @param {(p:number)=>void}   [opts.onProgress]  - progress hook 0..100
   * @param {number} [opts.writeChunk=20]           - BLE write size per packet
   * @param {number} [opts.writePauseMs=8]          - inter-packet delay (ms)
   */
  constructor(opts = {}) {
    this.onLog = opts.onLog || ((...a) => console.log('[BLE]', ...a));
    this.onProgress = typeof opts.onProgress === 'function' ? opts.onProgress : (()=>{});
    this.writeChunk = opts.writeChunk ?? 20;
    this.writePauseMs = opts.writePauseMs ?? 8;

    this.device = null;
    this.server = null;
    this.service = null;
    this.rxChar = null; // Write
    this.txChar = null; // Notify
    this.connected = false;

    this._rxTextBuf = '';
    this._pendingAckResolve = null;    // resolve(left)
    this._pendingSavedResolve = null;  // resolve(true/false)
  }

  /** Utils */
  _log(...a) { this.onLog(a.join(' ')); }
  _sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
  _setProgress(p) {
    const v = Math.max(0, Math.min(100, p|0));
    this.onProgress(v);
  }

  /** Connect to a device (filter by name, or accept all if empty) */
  async connect(nameFilter = '') {
    if (!navigator.bluetooth) throw new Error('Trình duyệt không hỗ trợ Web Bluetooth (dùng Chrome/Edge).');
    const options = nameFilter
      ? { filters: [{ name: nameFilter }], optionalServices: [WebBleUploader.NUS_SERVICE] }
      : { acceptAllDevices: true, optionalServices: [WebBleUploader.NUS_SERVICE] };

    this.device = await navigator.bluetooth.requestDevice(options);
    this.device.addEventListener('gattserverdisconnected', this._onDisconnected.bind(this));
    this._log('Đang kết nối:', this.device.name || '(không tên)');

    this.server  = await this.device.gatt.connect();
    this.service = await this.server.getPrimaryService(WebBleUploader.NUS_SERVICE);
    this.txChar  = await this.service.getCharacteristic(WebBleUploader.NUS_TX);
    this.rxChar  = await this.service.getCharacteristic(WebBleUploader.NUS_RX);

    await this.txChar.startNotifications();
    this.txChar.addEventListener('characteristicvaluechanged', this._onNotify.bind(this));

    this.connected = true;
    this._log('✅ Đã kết nối & bật notify.');
  }

  async disconnect() {
    try {
      if (this.device && this.device.gatt.connected) this.device.gatt.disconnect();
    } catch {}
    this._onDisconnected();
  }

  _onDisconnected() {
    this.connected = false;
    this.device = this.server = this.service = this.rxChar = this.txChar = null;
    this._rxTextBuf = '';
    this._pendingAckResolve = null;
    this._pendingSavedResolve = null;
    this._log('🔌 Đã ngắt kết nối.');
    this._setProgress(0);
  }

  _onNotify(ev) {
    const v = new TextDecoder().decode(ev.target.value);
    const toShow = v.replace(/\r/g,'');
    if (toShow.trim().length) this._log('<=', toShow.trimEnd());

    this._rxTextBuf += v;

    let idx;
    while ((idx = this._rxTextBuf.indexOf('\n')) >= 0) {
      const line = this._rxTextBuf.slice(0, idx).trim();
      this._rxTextBuf = this._rxTextBuf.slice(idx + 1);
      const low = line.toLowerCase();

      // OK <left>
      const m = low.match(/^ok\s+(\d+)\s*$/);
      if (m && this._pendingAckResolve) {
        const left = parseInt(m[1], 10);
        try { this._pendingAckResolve(left); } catch {}
        this._pendingAckResolve = null;
        continue;
      }
      // OK SAVED / WARN LEFT ...
      if ((low.indexOf('ok saved') === 0 || low.indexOf('warn left') === 0) && this._pendingSavedResolve) {
        try { this._pendingSavedResolve(true); } catch {}
        this._pendingSavedResolve = null;
        continue;
      }
      // ERR ...
      if (low.indexOf('err') === 0 && this._pendingSavedResolve) {
        try { this._pendingSavedResolve(false); } catch {}
        this._pendingSavedResolve = null;
        continue;
      }
    }
  }

  /** Low-level write (chunked 20B by default) */
  async _writeBLE(data) {
    if (!this.rxChar) throw new Error('RX characteristic chưa sẵn sàng');
    if (typeof data === 'string') data = new TextEncoder().encode(data);
    const n = data.length;
    for (let i = 0; i < n; i += this.writeChunk) {
      const slice = data.slice(i, i + this.writeChunk);
      if ('writeValueWithoutResponse' in this.rxChar) {
        await this.rxChar.writeValueWithoutResponse(slice);
      } else {
        await this.rxChar.writeValue(slice);
      }
      await this._sleep(this.writePauseMs);
    }
  }

  async _sendLine(s) {
    if (!this.connected) throw new Error('Chưa kết nối BLE.');
    if (!s.endsWith('\n')) s += '\n';
    await this._writeBLE(s);
  }

  /** Helpers */
  async ping()  { await this._sendLine('PING'); }
  async ls()    { await this._sendLine('LS'); }
  async reset() { await this._sendLine('reset'); }

  /** Waiters for ACK/SAVED */
  _waitAckOnce(timeoutMs = 5000) {
    return new Promise((resolve, reject) => {
      this._pendingAckResolve = resolve;
      setTimeout(() => {
        if (this._pendingAckResolve) {
          this._pendingAckResolve = null;
          reject(new Error('ACK timeout'));
        }
      }, timeoutMs);
    });
  }
  _waitSaved(timeoutMs = 10000) {
    return new Promise((resolve, reject) => {
      this._pendingSavedResolve = ok => ok ? resolve(true) : reject(new Error('Device ERR / not saved'));
      setTimeout(() => {
        if (this._pendingSavedResolve) {
          this._pendingSavedResolve = null;
          reject(new Error('No OK SAVED'));
        }
      }, timeoutMs);
    });
  }

  /**
   * Upload a file/buffer via PUT / DATA / DONE
   * @param {File|ArrayBuffer|Uint8Array} src
   * @param {string} [target="main.py"]
   * @param {Object} [opts]
   * @param {number} [opts.chunk=256]         - source bytes per DATA (before base64)
   * @param {boolean} [opts.waitAck=true]     - wait "OK <left>" after each DATA
   * @param {boolean} [opts.autoReset=false]  - send "reset" after upload
   */
  async upload(src, target = 'main.py', opts = {}) {
    if (!this.connected) throw new Error('Chưa kết nối BLE.');
    const chunkSz   = Math.max(64, Math.min(1024, opts.chunk ?? 256));
    const waitAck   = opts.waitAck !== false; // default true
    const autoReset = !!opts.autoReset;

    // Normalize to Uint8Array
    let buf;
    if (src instanceof Uint8Array) buf = src;
    else if (src instanceof ArrayBuffer) buf = new Uint8Array(src);
    else if (typeof File !== 'undefined' && src instanceof File) buf = new Uint8Array(await src.arrayBuffer());
    else throw new Error('src must be File, ArrayBuffer or Uint8Array');

    this._log(`⬆️ Upload → ${target} (${buf.length} bytes), chunk=${chunkSz}, waitAck=${waitAck}`);
    this._setProgress(0);

    // PUT
    await this._sendLine(`PUT ${target} ${buf.length}`);
    await this._sleep(200);

    // DATA loop (base64 per chunk)
    const total = buf.length;
    for (let off = 0; off < total; off += chunkSz) {
      const part = buf.subarray(off, off + chunkSz);
      // Uint8Array -> base64 (safe)
      let s = '';
      for (let j = 0; j < part.length; j++) s += String.fromCharCode(part[j]);
      const b64 = btoa(s);
      await this._sendLine('DATA ' + b64);

      if (waitAck) {
        try { await this._waitAckOnce(5000); }
        catch (e) { this._log('⚠️', e.message, '(tiếp tục)'); }
      }
      this._setProgress(((off + part.length) / total) * 100);
      await this._sleep(4);
    }

    // DONE
    await this._sendLine('DONE');
    this._log('📝 DONE — chờ "OK SAVED"...');
    try { await this._waitSaved(10000); this._log('✅ OK SAVED'); }
    catch (e) { this._log('⚠️', e.message); }

    if (autoReset) {
      await this._sleep(300);
      await this.reset();
    }
  }
}

/* ------- UMD-ish attach for non-module usage ------- */
try {
  // eslint-disable-next-line no-undef
  if (typeof window !== 'undefined' && !window.WebBleUploader) {
    // eslint-disable-next-line no-undef
    window.WebBleUploader = WebBleUploader;
  }
} catch {}
