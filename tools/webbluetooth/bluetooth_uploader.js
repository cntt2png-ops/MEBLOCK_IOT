/*
 * ble_mpy_nus.js
 * Web Bluetooth helper for MicroPython raw REPL over Nordic UART Service (NUS)
 *
 * Features:
 *  - Connect by namePrefix (e.g., "MEBLOCK-TOPKID")
 *  - Read logs from TX notify
 *  - Run code (string) via raw REPL (CTRL-C → CTRL-A → send → CTRL-D)
 *  - Run local .py file without flashing
 *  - Flash .py file to device filesystem as main.py (or custom dst)
 *  - Stop (CTRL-C), Reset (machine.reset())
 *
 * Requirements: Chromium-based browser, served over HTTPS or localhost
 */

export class BleMicroPythonNUS {
  // NUS UUIDs
  static NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
  static NUS_RX      = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // write
  static NUS_TX      = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // notify

  constructor({ onLog, onStatus, paceMs = 3 } = {}) {
    this.device = null;
    this.server = null;
    this.service = null;
    this.chrRX = null; // write
    this.chrTX = null; // notify
    this.paceMs = paceMs; // per-chunk delay

    this.onLog = typeof onLog === 'function' ? onLog : () => {};
    this.onStatus = typeof onStatus === 'function' ? onStatus : () => {};

    this.enc = new TextEncoder();
    this.dec = new TextDecoder();
    this._writeQueue = Promise.resolve();
    this._boundOnTx = (ev) => {
      const v = ev.target.value;
      const text = this.dec.decode(v);
      this.onLog(text);
    };
  }

  /** Utilities **/
  _sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
  _chunk20(u8) {
    const chunks = [];
    for (let i = 0; i < u8.length; i += 20) chunks.push(u8.slice(i, i + 20));
    return chunks;
  }
  _sendBytes(u8) {
    if (!this.chrRX) throw new Error('Not connected');
    this._writeQueue = this._writeQueue.then(async () => {
      for (const slice of this._chunk20(u8)) {
        await this.chrRX.writeValueWithoutResponse(slice);
        if (this.paceMs > 0) await this._sleep(this.paceMs);
      }
    });
    return this._writeQueue;
  }
  _sendText(s) { return this._sendBytes(this.enc.encode(s)); }

  /** Controls **/
  ctrlA() { return this._sendBytes(Uint8Array.of(0x01)); } // enter raw
  ctrlB() { return this._sendBytes(Uint8Array.of(0x02)); } // exit raw
  ctrlC() { return this._sendBytes(Uint8Array.of(0x03)); } // KeyboardInterrupt
  ctrlD() { return this._sendBytes(Uint8Array.of(0x04)); } // soft EOF / exec

  async enterRaw() {
    await this.ctrlC(); await this._sleep(50);
    await this.ctrlA(); await this._sleep(120);
  }
  async rawExec(pyCode) {
    await this._sendText(pyCode);
    await this.ctrlD();
  }

  /** Base64 helpers **/
  static base64FromBytes(bytes) {
    let binary = '';
    const chunk = 0x8000; // 32KB
    for (let i=0; i<bytes.length; i+=chunk) {
      const sub = bytes.subarray(i, i+chunk);
      binary += String.fromCharCode.apply(null, sub);
    }
    return btoa(binary);
  }
  static pyQuote(s) {
    return '\'' + s.replace(/\\/g, '\\\\').replace(/'/g, "\\'") + '\'';
  }

  /** Connection **/
  async connect({ namePrefix } = {}) {
    // Request device
    const opts = { optionalServices: [BleMicroPythonNUS.NUS_SERVICE] };
    if (namePrefix && String(namePrefix).length) {
      opts.filters = [{ namePrefix: String(namePrefix) }];
    } else {
      opts.acceptAllDevices = true;
    }
    this.onLog(`Scanning${namePrefix ? ` for '${namePrefix}*'` : ''}...`);
    this.device = await navigator.bluetooth.requestDevice(opts);
    this.device.addEventListener('gattserverdisconnected', () => this._onDisconnected());

    // Connect GATT
    this.server = await this.device.gatt.connect();
    this.service = await this.server.getPrimaryService(BleMicroPythonNUS.NUS_SERVICE);
    this.chrRX = await this.service.getCharacteristic(BleMicroPythonNUS.NUS_RX);
    this.chrTX = await this.service.getCharacteristic(BleMicroPythonNUS.NUS_TX);
    await this.chrTX.startNotifications();
    this.chrTX.addEventListener('characteristicvaluechanged', this._boundOnTx);

    this.onStatus(true, this.device.name || '');
    this.onLog(`Connected to ${this.device.name || '(no name)'}\n`);
  }

  async disconnect() {
    try { if (this.chrTX) await this.chrTX.stopNotifications(); } catch {}
    if (this.device?.gatt?.connected) this.device.gatt.disconnect();
    this._clearRefs();
    this.onStatus(false);
    this.onLog('Disconnected.\n');
  }

  _onDisconnected() {
    this._clearRefs();
    this.onStatus(false);
    this.onLog('Disconnected.\n');
  }
  _clearRefs() {
    this.server = null; this.service = null; this.chrRX = null; this.chrTX = null;
  }

  /** High-level actions **/
  async runCode(text, { label = 'inline' } = {}) {
    if (!this.chrRX) throw new Error('Not connected');
    this.onLog(`--- RUN (${label}) ---\n`);
    await this.enterRaw();
    await this._sendText(text);
    await this.ctrlD();
  }

  async runFile(file) {
    if (!(file instanceof File)) throw new Error('runFile expects a File');
    const text = await file.text();
    return this.runCode(text, { label: file.name });
  }

  async stop() {
    if (!this.chrRX) throw new Error('Not connected');
    this.onLog('[CTRL-C]\n');
    await this.ctrlC();
  }

  async reset() {
    if (!this.chrRX) throw new Error('Not connected');
    this.onLog('[machine.reset()]\n');
    await this.enterRaw();
    await this.rawExec('import machine\nmachine.reset()\n');
  }

  /** Flash .py file to device filesystem **/
  async flashFile(file, { dst = 'main.py', chunkSize = 768, onProgress } = {}) {
    if (!(file instanceof File)) throw new Error('flashFile expects a File');
    const ab = await file.arrayBuffer();
    const bytes = new Uint8Array(ab);

    this.onLog(`--- FLASH ${file.name} -> ${dst} (${bytes.length} bytes) ---\n`);
    await this.enterRaw();

    const init = 'import ubinascii, os\n' +
                 'def __fw(p,b64,m):\n' +
                 '    f=open(p,m)\n' +
                 '    f.write(ubinascii.a2b_base64(b64))\n' +
                 '    f.close()\n';
    await this.rawExec(init);

    let written = 0, idx = 0;
    while (written < bytes.length) {
      const part = bytes.subarray(written, written + chunkSize);
      const b64 = BleMicroPythonNUS.base64FromBytes(part);
      const mode = (written === 0) ? 'wb' : 'ab';
      const stmt = `__fw(${BleMicroPythonNUS.pyQuote(dst)}, ${BleMicroPythonNUS.pyQuote(b64)}, ${BleMicroPythonNUS.pyQuote(mode)})\n`;
      await this.rawExec(stmt);
      written += part.length; idx += 1;
      this.onLog(`  - chunk ${idx}: +${part.length} (${written}/${bytes.length})\n`);
      if (typeof onProgress === 'function') onProgress({ written, total: bytes.length, idx });
      await this._sleep(10);
    }

    await this.rawExec('import os\ntry:\n    os.sync()\nexcept Exception:\n    pass\nprint("[FLASH_DONE]")\n');
    this.onLog('[Flashed] Done.\n');
  }
}

/*
Usage example (ES module):

import { BleMicroPythonNUS } from './ble_mpy_nus.js';

const logBox = document.querySelector('#log');
const ble = new BleMicroPythonNUS({
  onLog: (t) => { logBox.textContent += t; logBox.scrollTop = logBox.scrollHeight; },
  onStatus: (connected, name) => console.log('Connected?', connected, name),
});

// Connect
await ble.connect({ namePrefix: 'MEBLOCK-TOPKID' });

// Run local file (from <input type="file" id="runFile">)
const file = document.querySelector('#runFile').files[0];
await ble.runFile(file); // no flash

// Flash local file to main.py
const f2 = document.querySelector('#flashFile').files[0];
await ble.flashFile(f2, { dst: 'main.py', onProgress: p => console.log(p) });

// Stop / Reset
await ble.stop();
await ble.reset();

// Disconnect
await ble.disconnect();
*/
