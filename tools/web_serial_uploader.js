/**
 * WebSerialUploader — Upload file cho ESP32 (MicroPython) qua Web Serial (raw REPL)
 * - Connect/Disconnect (mặc định 115200)
 * - upload(File|ArrayBuffer|Uint8Array, target="main.py", {chunk, onProgress, onLog, autoReset})
 * - Ghi file theo khối base64 vào raw REPL
 * - Sau khi xong: Ctrl-D thực thi, thoát raw (Ctrl-B), rồi machine.reset() (mặc định BẬT) để chạy main.py
 *
 * Yêu cầu:
 *  - Mở trang bằng HTTPS hoặc http://localhost (Chrome/Edge)
 *  - Người dùng chọn cổng (browser prompt)
 */
export class WebSerialUploader {
  constructor(opts = {}) {
    this.onLog = opts.onLog || ((...a) => console.log('[SERIAL]', ...a));
    this.onProgress = typeof opts.onProgress === 'function' ? opts.onProgress : (()=>{});
    this.baudRate = opts.baudRate || 115200;

    this.port = null;
    this.writer = null;
    this.reader = null;
    this._readLoopAbort = false;
    this._textDecoder = new TextDecoder();
    this._textEncoder = new TextEncoder();
  }

  _log(...a) { this.onLog(a.join(' ')); }
  _sleep(ms) { return new Promise(r => setTimeout(r, ms)); }
  _progress(p) { this.onProgress(Math.max(0, Math.min(100, p|0))); }

  async connect() {
    if (!('serial' in navigator)) {
      throw new Error('Trình duyệt không hỗ trợ Web Serial. Dùng Chrome/Edge và mở qua HTTPS/localhost.');
    }
    // Có thể thêm filters theo VID/PID nếu muốn
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: this.baudRate });

    this.writer = this.port.writable.getWriter();
    this.reader = this.port.readable.getReader();
    this._readLoopAbort = false;
    this._log('✅ Đã mở cổng serial @', this.baudRate);

    // Read-loop: đổ log từ thiết bị ra UI
    (async () => {
      try {
        while (!this._readLoopAbort) {
          const { value, done } = await this.reader.read();
          if (done || !value) break;
          this._log(this._textDecoder.decode(value));
        }
      } catch (e) {
        this._log('Reader stopped:', e);
      }
    })();
  }

  async disconnect() {
    try {
      this._readLoopAbort = true;
      if (this.reader) { try { await this.reader.cancel(); } catch {} this.reader.releaseLock(); this.reader = null; }
      if (this.writer) { try { await this.writer.close(); } catch {} this.writer.releaseLock(); this.writer = null; }
      if (this.port)   { try { await this.port.close(); }  catch {} this.port = null; }
    } finally {
      this._progress(0);
      this._log('🔌 Đã ngắt kết nối Serial.');
    }
  }

  async _writeBytes(bytes) {
    if (!this.writer) throw new Error('Chưa kết nối serial.');
    await this.writer.write(bytes);
  }
  async _writeText(s) {
    await this._writeBytes(this._textEncoder.encode(s));
  }
  async _ctrl(code) {
    await this._writeBytes(Uint8Array.of(code));
  }

  /**
   * Upload file qua raw REPL (ghi file bằng base64 theo khối)
   * @param {File|ArrayBuffer|Uint8Array} src
   * @param {string} targetName
   * @param {object} opts
   *   - chunk: byte gốc mỗi khối base64 (mặc định 768; file lớn có thể 1024–1536)
   *   - autoReset: sau khi xong sẽ reset để chạy main.py (mặc định TRUE)
   */
  async upload(src, targetName = 'main.py', opts = {}) {
    if (!this.port) throw new Error('Chưa kết nối serial.');
    const CHUNK = Math.max(128, Math.min(2048, opts.chunk || 768));
    const autoReset = opts.autoReset !== false;   // ✅ mặc định BẬT

    // Chuẩn hoá buffer
    let buf;
    if (src instanceof Uint8Array) buf = src;
    else if (src instanceof ArrayBuffer) buf = new Uint8Array(src);
    else if (typeof File !== 'undefined' && src instanceof File) buf = new Uint8Array(await src.arrayBuffer());
    else throw new Error('src phải là File, ArrayBuffer hoặc Uint8Array');

    // 1) Vào raw REPL
    await this._ctrl(0x03); await this._ctrl(0x03); // Ctrl-C x2: dừng chương trình đang chạy
    await this._sleep(60);
    await this._ctrl(0x01); // Ctrl-A: vào raw REPL
    await this._sleep(40);

    // 2) Gửi script Python mở file & ghi theo base64
    await this._writeText(`import ubinascii\r\nf=open('${targetName}','wb')\r\n`);

    this._log(`⬆️ Upload ${targetName} — ${buf.length} bytes, chunk=${CHUNK}`);
    this._progress(0);

    const total = buf.length;
    for (let off = 0; off < total; off += CHUNK) {
      const part = buf.subarray(off, off + CHUNK);
      // Uint8Array -> base64
      let s = '';
      for (let i = 0; i < part.length; i++) s += String.fromCharCode(i in part ? part[i] : 0);
      const b64 = btoa(s);
      await this._writeText(`f.write(ubinascii.a2b_base64('${b64}'))\r\n`);
      this._progress(((off + part.length) / total) * 100);
      await this._sleep(1); // nhịp thở cho REPL
    }

    await this._writeText(`f.close()\r\nprint('OK SAVED')\r\n`);

    // 3) Ctrl-D để execute block vừa gửi
    await this._ctrl(0x04); // Ctrl-D
    await this._sleep(200);

    // 4) Thoát raw REPL rồi reset để chắc chắn chạy main.py
    if (autoReset) {
      try {
        await this._ctrl(0x02);      // Ctrl-B: raw -> friendly REPL
        await this._sleep(120);
      } catch (_) {}
      await this._writeText(`\r\nimport machine; machine.reset()\r\n`); // hard reset → boot.py → main.py
    }
    this._log('✅ Upload xong & đã yêu cầu reset để chạy main.py.');
  }
}

/* UMD attach cho non-module */
try {
  if (typeof window !== 'undefined' && !window.WebSerialUploader) {
    window.WebSerialUploader = WebSerialUploader;
  }
} catch {}
