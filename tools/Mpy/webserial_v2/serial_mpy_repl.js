/*
 * serial_mpy_repl.js — v2 (non-blocking connect)
 * Web Serial helper for MicroPython raw REPL over COM/TTY
 *
 * Features
 *  - Connect via Web Serial (Windows COM / Linux & macOS tty)
 *  - Stream logs (onLog)
 *  - Run code (raw REPL: CTRL-C → CTRL-A → send → CTRL-D)
 *  - Run local .py (no flash)
 *  - Flash .py to filesystem (dst = main.py default)
 *  - Stop (CTRL-C), Reset (machine.reset())
 *
 * Notes
 *  - connect() is NON-BLOCKING now (read loop runs in background)
 *  - Requires Chromium-based browser, served over HTTPS or localhost
 */

export class SerialMicroPythonREPL {
  constructor({
    onLog = (text) => console.log(text),
    onStatus = (ok) => console.log('status:', ok),
    paceMs = 0,           // thêm delay nhẹ giữa các chunk gửi
    vid = null,
    pid = null,
  } = {}) {
    this.onLog = onLog;
    this.onStatus = onStatus;
    this.paceMs = paceMs;
    this.vid = vid;
    this.pid = pid;

    this.port = null;
    this.reader = null;
    this.writer = null;
    this.enc = new TextEncoder();
    this.dec = new TextDecoder();
    this.connected = false;
    this._readLoopAbort = null;
    this._writeQueue = Promise.resolve();
  }

  /* ===== internal helpers ===== */
  _logChunk(text) {
    if (!text) return;
    this.onLog(text);
  }

  _requireConnected() {
    if (!this.connected || !this.port) throw new Error('Not connected');
  }

  async _sleep(ms) {
    if (!ms || ms <= 0) return;
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  async _startReadLoop() {
    if (!this.port) return;
    if (this.reader) {
      try { await this._stopReadLoop(); } catch (e) {}
    }

    this._readLoopAbort = new AbortController();
    const signal = this._readLoopAbort.signal;

    const textDecoder = new TextDecoder();

    const reader = this.port.readable.getReader();
    this.reader = reader;

    this.onLog('[ReadLoop] started.\n');

    const pump = async () => {
      try {
        while (true) {
          if (signal.aborted) break;
          const { value, done } = await reader.read();
          if (done) break;
          if (value) {
            const text = textDecoder.decode(value);
            this._logChunk(text);
          }
        }
      } catch (err) {
        if (!signal.aborted) {
          this.onLog(`[ReadLoop] error: ${err}\n`);
        }
      } finally {
        try { reader.releaseLock(); } catch (e) {}
        if (this.reader === reader) this.reader = null;
        this.onLog('[ReadLoop] stopped.\n');
      }
    };

    pump();
  }

  async _stopReadLoop() {
    if (!this.reader) return;
    try {
      this._readLoopAbort?.abort();
    } catch (e) {}
    this._readLoopAbort = null;
    try {
      await this.reader.cancel();
    } catch (e) {}
    try {
      this.reader.releaseLock();
    } catch (e) {}
    this.reader = null;
  }

  /* ===== connect / disconnect ===== */
  async connect() {
    if (!('serial' in navigator)) {
      throw new Error('Web Serial API not available in this browser');
    }

    const filters = [];
    if (this.vid != null && this.pid != null) {
      filters.push({ usbVendorId: this.vid, usbProductId: this.pid });
    }

    const port = await navigator.serial.requestPort(
      filters.length ? { filters } : {}
    );
    await port.open({ baudRate: 115200 });

    this.port = port;
    this.connected = true;

    const writable = port.writable;
    this.writer = writable.getWriter();

    this.onStatus(true);
    this.onLog('[Serial] Connected.\n');

    await this._startReadLoop();
  }

  async disconnect() {
    try { await this._stopReadLoop(); } catch (e) {}

    try {
      if (this.writer) await this.writer.close();
    } catch (e) {}
    try {
      this.writer?.releaseLock();
    } catch (e) {}
    this.writer = null;

    try {
      if (this.port) await this.port.close();
    } catch (e) {}

    this.port = null;
    this.connected = false;
    this.onStatus(false);
    this.onLog('[Serial] Disconnected.\n');
  }

  /* ===== sending bytes / text ===== */

  // 🔧 ĐÃ SỬA: giảm chunkSize từ 512 xuống 128 cho an toàn, kết hợp với paceMs
  async _sendBytes(bytes, { chunkSize = 128 } = {}) {
    this._requireConnected();
    const writer = this.writer;
    this._writeQueue = this._writeQueue.then(async () => {
      for (let i = 0; i < bytes.length; i += chunkSize) {
        const slice = bytes.slice(i, i + chunkSize);
        await writer.write(slice);
        if (this.paceMs > 0) await this._sleep(this.paceMs);
      }
    });
    return this._writeQueue;
  }

  _sendText(text) { return this._sendBytes(this.enc.encode(text)); }

  /* ===== REPL controls ===== */
  ctrlA() { return this._sendBytes(Uint8Array.of(0x01)); } // enter raw
  ctrlB() { return this._sendBytes(Uint8Array.of(0x02)); } // exit raw
  ctrlC() { return this._sendBytes(Uint8Array.of(0x03)); } // KeyboardInterrupt
  ctrlD() { return this._sendBytes(Uint8Array.of(0x04)); } // soft reboot / exec

  /* ===== high-level REPL operations ===== */

  async enterRaw() {
    this._requireConnected();
    await this.ctrlC();
    await this._sleep(40);
    await this.ctrlA();
    await this._sleep(40);
  }

  // 🔧 ĐÃ SỬA: sau khi chạy xong thì chờ 1 chút rồi thoát raw REPL (CTRL-B)
  async runCode(text, { label = 'inline' } = {}) {
    this._requireConnected();
    this.onLog(`--- RUN (${label}) ---\n`);
    await this.enterRaw();
    await this._sendText(text);
    await this.ctrlD();
    await this._sleep(100);
    await this.ctrlB();
  }

  async runFile(file) {
    if (!(file instanceof File)) throw new Error('runFile expects a File');
    const text = await file.text();
    return this.runCode(text, { label: file.name });
  }

  async stop() {
    this._requireConnected();
    this.onLog('[CTRL-C]\n');
    await this.ctrlC();
  }

  async reset() {
    this._requireConnected();
    this.onLog('[machine.reset()]\n');
    await this.enterRaw();
    await this.rawExec('import machine\nmachine.reset()\n');
  }

  async rawExec(pyCode) {
    await this._sendText(pyCode);
    await this.ctrlD();
  }

  /* ===== helpers ===== */
  static base64FromBytes(bytes) {
    let binary = '';
    const step = 0x8000; // 32KB
    for (let i = 0; i < bytes.length; i += step) {
      const sub = bytes.subarray(i, i + step);
      binary += String.fromCharCode.apply(null, sub);
    }
    return btoa(binary);
  }
  static pyQuote(s) { return '\'' + s.replace(/\\/g, '\\\\').replace(/'/g, "\\'") + '\''; }

  /* ===== flashing a file (like main.py) ===== */

  async flashFile(file, { dst = 'main.py', chunkSize = 1024, onProgress } = {}) {
    this._requireConnected();
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
      const b64 = SerialMicroPythonREPL.base64FromBytes(part);
      const mode = (written === 0) ? 'wb' : 'ab';
      const stmt = `__fw(${SerialMicroPythonREPL.pyQuote(dst)}, ${SerialMicroPythonREPL.pyQuote(b64)}, ${SerialMicroPythonREPL.pyQuote(mode)})\n`;
      await this.rawExec(stmt);
      written += part.length; idx += 1;
      this.onLog(`  - chunk ${idx}: +${part.length} (${written}/${bytes.length})\n`);
      if (typeof onProgress === 'function') onProgress({ written, total: bytes.length, idx });
      await this._sleep(10);
    }

    // 🔧 ĐÃ SỬA: sau khi sync, chờ 1 chút rồi thoát raw REPL
    await this.rawExec('import os\ntry:\n    os.sync()\nexcept Exception:\n    pass\nprint("[FLASH_DONE]")\n');
    await this._sleep(100);
    await this.ctrlB();
    this.onLog('[Flashed] Done.\n');
  }
}
