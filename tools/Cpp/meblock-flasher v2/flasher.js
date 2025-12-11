// flasher.js
import { Transport, ESPLoader, hardReset } from "https://unpkg.com/esptool-js/lib/index.js";

const MANIFEST_PATH = "./firmware/manifest.json";

const logEl = document.getElementById("log");
const progressFillEl = document.getElementById("progress-fill");
const progressTextEl = document.getElementById("progress-text");
const flashBtn = document.getElementById("flash-btn");

function log(msg) {
  console.log(msg);
  if (!logEl) return;
  const line = document.createElement("div");
  line.textContent = msg;
  logEl.appendChild(line);
  logEl.scrollTop = logEl.scrollHeight;
}

function setProgress(pct, extra = "") {
  if (progressFillEl) {
    progressFillEl.style.width = `${Math.max(0, Math.min(100, pct))}%`;
  }
  if (progressTextEl) {
    progressTextEl.textContent = `Progress: ${pct}% ${extra}`;
  }
}

async function loadManifest(url) {
  const resp = await fetch(url);
  if (!resp.ok) {
    throw new Error(`Không tải được manifest (${resp.status})`);
  }
  return resp.json();
}

function blobToBinaryString(blob) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onerror = () => reject(reader.error);
    reader.onload = () => resolve(reader.result);
    reader.readAsBinaryString(blob);
  });
}

async function requestSerialPort() {
  // Có thể thêm filter VID/PID nếu muốn
  return navigator.serial.requestPort();
}

async function flashFromManifest(manifestPath = MANIFEST_PATH) {
  if (!("serial" in navigator)) {
    alert("Trình duyệt không hỗ trợ Web Serial. Hãy dùng Chrome/Edge mới.");
    return;
  }

  flashBtn.disabled = true;
  setProgress(0, "");
  log("=== MEBLOCK Web Flasher ===");

  try {
    log("Chọn cổng ESP32...");
    const port = await requestSerialPort();
    await port.open({ baudRate: 115200 });

    const transport = new Transport(port);
    const loader = new ESPLoader({
      transport,
      baudrate: 115200,
      romBaudrate: 115200,
      enableTracing: false
    });

    log("Kết nối tới chip...");
    await loader.main();    // sync ROM
    await loader.flashId(); // đọc flash ID, cũng giúp chắc chắn kết nối OK

    const chipFamily = loader.chip.CHIP_NAME;
    log(`Đã nhận dạng chip: ${chipFamily}`);

    // ----- Đọc manifest -----
    const manifest = await loadManifest(manifestPath);

    // Tìm build tương ứng chip
    const build = manifest.builds.find((b) => b.chipFamily === chipFamily);
    if (!build) {
      throw new Error(`Manifest không hỗ trợ chipFamily: ${chipFamily}`);
    }

    log(`Dùng build: "${manifest.name}" v${manifest.version}`);
    log(`Số phần firmware: ${build.parts.length}`);

    const manifestURL = new URL(manifestPath, window.location.href).toString();
    const filePromises = build.parts.map(async (part) => {
      const fileUrl = new URL(part.path, manifestURL).toString();
      log(`Tải firmware: ${part.path} @ 0x${part.offset.toString(16)}`);

      const resp = await fetch(fileUrl);
      if (!resp.ok) {
        throw new Error(`Lỗi tải ${part.path}: HTTP ${resp.status}`);
      }

      const blob = await resp.blob();
      const data = await blobToBinaryString(blob); // string nhị phân
      return { data, address: part.offset };
    });

    const fileArray = [];
    let totalSize = 0;

    for (let i = 0; i < filePromises.length; i++) {
      const filePart = await filePromises[i];
      fileArray.push(filePart);
      totalSize += filePart.data.length;
    }

    log(`Tổng dung lượng cần flash ~${totalSize} bytes`);

    let totalWritten = 0;
    setProgress(0, "(bắt đầu)");

    // Nếu muốn erase toàn bộ flash trước:
    // log("Erasing flash...");
    // await loader.eraseFlash();

    log("Bắt đầu ghi flash...");
    await loader.writeFlash({
      fileArray,
      flashSize: "keep",
      flashMode: "keep",
      flashFreq: "keep",
      eraseAll: false,
      compress: true,
      reportProgress: (fileIndex, written, total) => {
        const fileData = fileArray[fileIndex].data;
        const uncompressedWritten = (written / total) * fileData.length;
        const done = totalWritten + uncompressedWritten;
        const pct = Math.floor((done / totalSize) * 100);

        if (written === total) {
          totalWritten += uncompressedWritten;
        }

        setProgress(pct, `(file #${fileIndex + 1})`);
      }
    });

    setProgress(100, "(xong)");
    log("Flash thành công. Reset board...");

    await hardReset(transport);
    await transport.disconnect();
    log("Hoàn tất!");
  } catch (err) {
    console.error(err);
    log(`*** Lỗi: ${err.message || err}`);
  } finally {
    flashBtn.disabled = false;
  }
}

if (flashBtn) {
  flashBtn.addEventListener("click", () => {
    flashFromManifest().catch((err) => {
      console.error(err);
      log(`*** Lỗi ngoài dự kiến: ${err.message || err}`);
    });
  });
}
