/*
 * bluetooth_uploader.js
 * Web Bluetooth helper cho MicroPython raw REPL qua Nordic UART Service (NUS)
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
      // 🔹 Chỉ filter theo namePrefix, KHÔNG filter thêm service
      filters: [{ namePrefix: prefix }],
      // 🔹 Vẫn khai báo NUS ở optionalServices để sau khi connect đọc được service
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
    this.onStatus(false, '');
    this._log('[BLE] Disconnected.\n');
  }

  _onDisconnected() {
    this.connected = false;
    this.onStatus(false, '');
    this._log('[BLE] Disconnected (remote).\n');
  }

  _onTxNotify(event) {
    const value = event.target.value;
    const text = this.dec.decode(value);
    this._log(text);
  }

  async _sendBytes(bytes, { chunkSize = 100 } = {}) {
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
    await this._sleep(50);
  }

  async ctrlA() {
    this._requireConnected();
    await this._sendBytes(new Uint8Array([0x01]));
    await this._sleep(50);
  }

  async ctrlB() {
    this._requireConnected();
    await this._sendBytes(new Uint8Array([0x02]));
    await this._sleep(50);
  }

  async rawExec(code) {
    if (!this.connected) throw new Error('Not connected');

    await this.ctrlC();
    await this.ctrlA();
    await this._sleep(50);

    await this._sendText(code);
    if (!code.endsWith('\n')) {
      await this._sendText('\n');
    }
    await this._sendBytes(new Uint8Array([0x04])); // CTRL-D
    await this._sleep(50);
  }

  async runCode(code, { label } = {}) {
    if (label) {
      this._log(`[RunCode] ${label}\n`);
    }
    await this.rawExec(code);
  }

  async runFile(file) {
    if (!file) throw new Error('No file');
    const text = await file.text();
    this._log(`[RunFile] ${file.name}\n`);
    await this.runCode(text, { label: file.name });
  }

  async flashFile(file, { dst = 'main.py', onProgress } = {}) {
    if (!file) throw new Error('No file');
    const buf = new Uint8Array(await file.arrayBuffer());
    this._log(`[Flash] ${file.name} -> ${dst}\n`);

    const helper = `
import ubinascii
def __fw(path, data_b64, mode):
    b = ubinascii.a2b_base64(data_b64)
    f = open(path, mode)
    f.write(b)
    f.close()
`;
    await this.rawExec(helper);

    const chunkSize = 512;
    let written = 0;
    let idx = 0;

    while (written < buf.length) {
      const part = buf.subarray(written, written + chunkSize);
      const b64 = BleMicroPythonNUS.base64FromBytes(part);
      const mode = (written === 0) ? 'wb' : 'ab';
      const stmt = `__fw(${BleMicroPythonNUS.pyQuote(dst)}, ${BleMicroPythonNUS.pyQuote(b64)}, ${BleMicroPythonNUS.pyQuote(mode)})`;
      await this.rawExec(stmt);
      written += part.length;
      idx += 1;
      this._log(`  - chunk ${idx}: +${part.length} (${written}/${buf.length})\n`);
      if (typeof onProgress === 'function') {
        onProgress({ written, total: buf.length, idx });
      }
      await this._sleep(10);
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
