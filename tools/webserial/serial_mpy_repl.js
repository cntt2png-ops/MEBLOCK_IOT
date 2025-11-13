/*
 * serial_mpy_repl.js — v2 (non‑blocking connect)
 * Web Serial helper for MicroPython raw REPL over COM/TTY
 *
 * Features
 *  - Connect via Web Serial (Windows COM / Linux & macOS tty)
 *  - Stream logs (onLog)
 *  - Run code (raw REPL: CTRL‑C → CTRL‑A → send → CTRL‑D)
 *  - Run local .py (no flash)
 *  - Flash .py to filesystem (dst = main.py default)
 *  - Stop (CTRL‑C), Reset (machine.reset())
 *
 * Notes
 *  - connect() is NON‑BLOCKING now (read loop runs in background)
 *  - Requires Chromium‑based browser, served over HTTPS or localhost
 */

export class SerialMicroPythonREPL {
  constructor({ onLog, onStatus, paceMs = 0 } = {}) {
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.connected = false;

    this.paceMs = paceMs; // optional throttle per write slice

    this.onLog = typeof onLog === 'function' ? onLog : () => {};
    this.onStatus = typeof onStatus === 'function' ? onStatus : () => {};

    this.enc = new TextEncoder();
    this.dec = new TextDecoder();

    this._writeQueue = Promise.resolve();
    this._readLoopAbort = null;
  }

  /* ===== utils ===== */
  _sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
  _requireConnected() {
    if (!this.port || !this.writer || !this.connected) throw new Error('Not connected');
  }

  async _startReadLoop() {
    if (!this.port?.readable) return;
    this._readLoopAbort = new AbortController();
    const signal = this._readLoopAbort.signal;
    this.reader = this.port.readable.getReader(); // Web Serial: no AbortSignal param
    try {
      while (!signal.aborted) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (value) this.onLog(this.dec.decode(value));
      }
    } catch (err) {
      if (err?.name !== 'AbortError') this.onLog(`[READ ERROR] ${err}\n`);
    } finally {
      try { this.reader.releaseLock(); } catch {}
      this.reader = null;
    }
  }

  async _stopReadLoop() {
    try { this._readLoopAbort?.abort(); } catch {}
    try { await this.reader?.cancel(); } catch {}
    this._readLoopAbort = null;
  }

  async _sendBytes(bytes, { chunkSize = 512 } = {}) {
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
  ctrlD() { return this._sendBytes(Uint8Array.of(0x04)); } // soft EOF / exec

  async enterRaw() {
    await this.ctrlC(); await this._sleep(50);
    await this.ctrlA(); await this._sleep(120);
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

  /* ===== connect / disconnect ===== */
  async connect({ baudRate = 115200, parity = 'none', dataBits = 8, stopBits = 1 } = {}) {
    if (!('serial' in navigator)) throw new Error('Web Serial API not supported');

    // Select & open
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate, parity, dataBits, stopBits, flowControl: 'none' });

    // Prepare writer
    this.writer = this.port.writable.getWriter();

    // Mark connected and notify UI immediately (non‑blocking)
    this.connected = true;
    this.onStatus(true);
    this.onLog(`Connected @ ${baudRate} baud\n`);

    // Start the read loop in background
    this._startReadLoop().catch(e => this.onLog(`[READ LOOP ERROR] ${e}\n`));

    return true;
  }

  async disconnect() {
    try { await this._stopReadLoop(); } catch {}
    try { if (this.writer) await this.writer.close(); } catch {}
    try { this.writer?.releaseLock(); } catch {}
    this.writer = null;
    try { await this.port?.close(); } catch {}
    this.port = null;
    this.connected = false;
    this.onStatus(false);
    this.onLog('Disconnected.\n');
  }

  /* ===== high‑level actions ===== */
  async runCode(text, { label = 'inline' } = {}) {
    this._requireConnected();
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

    await this.rawExec('import os\ntry:\n    os.sync()\nexcept Exception:\n    pass\nprint("[FLASH_DONE]")\n');
    this.onLog('[Flashed] Done.\n');
  }
}
