/*
 * bluetooth_uploader_v2.js
 * Web Bluetooth helper cho MicroPython raw REPL qua Nordic UART Service (NUS)
 *
 * ✅ Fix chính: flashFile() KHÔNG gọi rawExec() cho từng chunk nữa.
 *    - Vào raw REPL 1 lần
 *    - Gộp nhiều chunk vào 1 "lần execute" → giảm số CTRL-D rất nhiều
 *
 * Lưu ý: Trong raw REPL, mỗi lần "execute" vẫn cần ít nhất 1 CTRL-D,
 * nên ta chỉ "giảm bớt" CTRL-D bằng batching (ví dụ 6 chunk / 1 CTRL-D).
 */

export class BleMicroPythonNUS {
  constructor({
    onLog = (t) => console.log(t),
    onStatus = (ok, name) => console.log('status:', ok, name),
    paceMs = 0,
  } = {}) {
    this.onLog = onLog;
    this.onStatus = onStatus;
    this.paceMs = paceMs;

    this.device = null;
    this.server = null;
    this.service = null;
    this.chrRX = null; // write
    this.chrTX = null; // notify
    this.connected = false;

    this._writeQueue = Promise.resolve();
    this._boundOnTx = this._onTxNotify.bind(this);
    this.enc = new TextEncoder();
    this.dec = new TextDecoder();

    this._inRaw = false;
  }

  static get NUS_SERVICE() {
    return '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
  }
  static get NUS_RX() {
    return '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // write
  }
  static get NUS_TX() {
    return '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // notify
  }

  static pyQuote(s) {
    return "'" + String(s).replace(/\\/g, "\\\\").replace(/'/g, "\\'") + "'";
  }

  static base64FromBytes(bytes) {
    // an toàn cho chunk nhỏ (512~1024B). Nếu bạn dùng chunk lớn hơn, nên tối ưu thêm.
    let binary = "";
    for (let i = 0; i < bytes.length; i++) {
      binary += String.fromCharCode(bytes[i]);
    }
    return btoa(binary);
  }

  _log(text) {
    this.onLog(text);
  }

  _requireConnected() {
    if (!this.connected || !this.device || !this.chrRX) {
      throw new Error('BLE not connected');
    }
  }

  async _sleep(ms) {
    if (!ms) return;
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  async connect({ namePrefix, timeoutMs = 0 } = {}) {
    if (!navigator.bluetooth) {
      throw new Error('Web Bluetooth API is not available');
    }

    const prefix = String(namePrefix || 'MEBLOCK-');

    const opts = {
      // 🔹 filter theo namePrefix
      filters: [{ namePrefix: prefix }],
      // 🔹 khai báo NUS ở optionalServices để đọc service sau khi connect
      optionalServices: [BleMicroPythonNUS.NUS_SERVICE],
    };

    this._log(`Requesting device with prefix '${prefix}'...\n`);
    this.device = await navigator.bluetooth.requestDevice(opts);
    this.device.addEventListener('gattserverdisconnected', () => this._onDisconnected());

    this._log(`Connecting to GATT...\n`);
    this.server = await this.device.gatt.connect();
    this.service = await this.server.getPrimaryService(BleMicroPythonNUS.NUS_SERVICE);
    this.chrRX = await this.service.getCharacteristic(BleMicroPythonNUS.NUS_RX);
    this.chrTX = await this.service.getCharacteristic(BleMicroPythonNUS.NUS_TX);

    await this.chrTX.startNotifications();
    this.chrTX.addEventListener('characteristicvaluechanged', this._boundOnTx);

    this.connected = true;
    this._inRaw = false;
    this.onStatus(true, this.device.name || '');
    this._log('[BLE] Connected.\n');
  }

  async disconnect() {
    if (!this.device) return;

    try {
      if (this.chrTX) {
        this.chrTX.removeEventListener('characteristicvaluechanged', this._boundOnTx);
        try { await this.chrTX.stopNotifications(); } catch (e) {}
      }
    } catch (e) {}

    try {
      if (this.server && this.server.connected) {
        await this.server.disconnect();
      }
    } catch (e) {}

    this.device = null;
    this.server = null;
    this.service = null;
    this.chrRX = null;
    this.chrTX = null;
    this.connected = false;
    this._inRaw = false;
    this.onStatus(false, '');
    this._log('[BLE] Disconnected.\n');
  }

  _onDisconnected() {
    this.connected = false;
    this._inRaw = false;
    this.onStatus(false, '');
    this._log('[BLE] Disconnected (remote).\n');
  }

  _onTxNotify(event) {
    const value = event.target.value;
    const text = this.dec.decode(value);
    this._log(text);
  }

  async _sendBytes(bytes, { chunkSize = 180 } = {}) {
    this._requireConnected();
    const chr = this.chrRX;
    this._writeQueue = this._writeQueue.then(async () => {
      for (let i = 0; i < bytes.length; i += chunkSize) {
        const slice = bytes.slice(i, i + chunkSize);
        await chr.writeValue(slice);
        if (this.paceMs > 0) await this._sleep(this.paceMs);
      }
    });
    return this._writeQueue;
  }

  async _sendText(text) {
    const bytes = this.enc.encode(text);
    return this._sendBytes(bytes);
  }

  async ctrlC() {
    this._requireConnected();
    await this._sendBytes(new Uint8Array([0x03]));
    await this._sleep(40);
  }

  async ctrlA() {
    this._requireConnected();
    await this._sendBytes(new Uint8Array([0x01]));
    await this._sleep(40);
  }

  async ctrlB() {
    this._requireConnected();
    await this._sendBytes(new Uint8Array([0x02]));
    await this._sleep(40);
  }

  async ctrlD() {
    this._requireConnected();
    await this._sendBytes(new Uint8Array([0x04]));
    await this._sleep(60);
  }

  /* ===== Raw REPL helpers ===== */

  async enterRaw({ force = false } = {}) {
    this._requireConnected();
    if (this._inRaw && !force) return;

    await this.ctrlC();
    await this.ctrlA();
    this._inRaw = true;
    await this._sleep(60);
  }

  async exitRaw() {
    if (!this.connected) return;
    try { await this.ctrlB(); } catch (e) {}
    this._inRaw = false;
  }

  async _rawExecInRaw(code) {
    this._requireConnected();
    if (!this._inRaw) await this.enterRaw({ force: true });

    await this._sendText(code);
    if (!code.endsWith('\n')) await this._sendText('\n');
    await this.ctrlD();
  }

  async rawExec(code, { forceEnter = true } = {}) {
    if (!this.connected) throw new Error('Not connected');
    if (forceEnter) await this.enterRaw({ force: true });
    await this._rawExecInRaw(code);
  }

  async runCode(code, { label } = {}) {
    if (label) this._log(`[RunCode] ${label}\n`);
    await this.rawExec(code, { forceEnter: true });
  }

  async runFile(file) {
    if (!file) throw new Error('No file');
    const text = await file.text();
    this._log(`[RunFile] ${file.name}\n`);
    await this.runCode(text, { label: file.name });
  }

  /**
   * Flash file .py vào filesystem
   * - batchingChunks: số chunk (mỗi chunkBytes) gộp chung 1 lần execute → giảm CTRL-D
   * - chunkBytes: size dữ liệu gốc mỗi chunk
   */
  async flashFile(
    file,
    {
      dst = 'main.py',
      onProgress,
      chunkBytes = 512,
      batchingChunks = 6,
      sync = true,
    } = {}
  ) {
    if (!file) throw new Error('No file');
    const buf = new Uint8Array(await file.arrayBuffer());
    this._log(`[Flash] ${file.name} -> ${dst} (${buf.length} bytes)\n`);

    await this.enterRaw({ force: true });

    // import 1 lần
    await this._rawExecInRaw("import ubinascii\n");

    let written = 0;
    let chunkIdx = 0;

    while (written < buf.length) {
      const mode = (written === 0) ? 'wb' : 'ab';

      const lines = [];
      lines.push(`f=open(${BleMicroPythonNUS.pyQuote(dst)}, ${BleMicroPythonNUS.pyQuote(mode)})`);

      let localChunks = 0;
      while (localChunks < batchingChunks && written < buf.length) {
        const part = buf.subarray(written, written + chunkBytes);
        const b64 = BleMicroPythonNUS.base64FromBytes(part);
        lines.push(`f.write(ubinascii.a2b_base64(${BleMicroPythonNUS.pyQuote(b64)}))`);

        written += part.length;
        chunkIdx += 1;
        localChunks += 1;

        if (typeof onProgress === 'function') {
          onProgress({ written, total: buf.length, idx: chunkIdx });
        }
      }
      lines.push("f.close()");

      await this._rawExecInRaw(lines.join("\n") + "\n");

      this._log(`  - batch done: chunks=${localChunks}, written=${written}/${buf.length}\n`);
      await this._sleep(15);
    }

    if (sync) {
      await this._rawExecInRaw("import os\ntry:\n  os.sync()\nexcept Exception:\n  pass\n");
    }

    this._log('[Flash] Done.\n');
  }

  async stop() {
    await this.ctrlC();
  }

  async reset() {
    await this.runCode('import machine; machine.reset()', { label: 'reset' });
  }
}
