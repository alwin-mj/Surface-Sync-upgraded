# -*- coding: utf-8 -*-
import sys
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

"""
SurfaceSync – Receiver Backend  v2.1  [8-FSK CNN]
==================================================
Run : python receiver_backend.py
Port: http://localhost:5002
Open: http://localhost:5002/   ← always open via this URL, never file://

Protocol sync with TX (v2.2)
-----------------------------
TX ESP32  : 20000 Hz sample rate, 1000 baud → 20 samples / symbol played
RX ESP32  : 44100 Hz sample rate, 1000 baud → 44 samples / symbol captured
Symbol dur: 1 ms on both sides (rates are independent hardware chains)
SPS       : 44  (44100 / 1000)  ✓
Tribit pack: matches tx_3bytes() in esp32_cam_transmitter.ino exactly
Sync word : 0xAA 0x55  (acoustic packet header, built by TX Python backend)
RX frame  : 0xBB 0x44  (from RX ESP32, parsed by _serial_reader)
AES pass  : must match transmitter_backend.py  (default: SurfaceSyncSecretKey2025)

Dependencies
------------
pip install flask flask-socketio flask-cors pycryptodome numpy pyserial scipy requests
pip install tensorflow   (optional — FFT fallback used if not installed)
"""

import os
import time
import struct
import threading
import queue

import numpy as np
import serial
import serial.tools.list_ports
import requests
from scipy.signal import butter, sosfilt
from flask import Flask, request, jsonify, send_file as flask_send_file
from flask_socketio import SocketIO, emit
from flask_cors import CORS
from Crypto.Cipher import AES
from Crypto.Util.Padding import unpad
from Crypto.Hash import SHA256

# ── ML import — sklearn works on Python 3.13, TF does not ───────────────────
# We use sklearn MLPClassifier as the 1D-CNN equivalent.
# It requires only:  pip install scikit-learn  (already fast on CPU)
# TensorFlow is NOT used — it does not support Python 3.13.
CNN_AVAILABLE = False
_clf = None   # sklearn MLPClassifier instance

try:
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler
    import joblib
    CNN_AVAILABLE = True
    print("[RX] scikit-learn loaded — MLP classifier mode available.")
except ImportError:
    print("[RX] scikit-learn not installed — FFT fallback mode.")
    print("[RX]   To enable MLP:  pip install scikit-learn")

# Keep TF_AVAILABLE alias so rest of code still works
TF_AVAILABLE = CNN_AVAILABLE

# ── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.config["SECRET_KEY"] = os.urandom(32)
app.config["MAX_CONTENT_LENGTH"] = 64 * 1024 * 1024  # 64 MB
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ── Constants ─────────────────────────────────────────────────────────────────
SAMPLE_RATE   = 20000          # RX ESP32 ADC sample rate (timer+analogRead)
BAUD_SYMBOLS  = 1000           # symbols per second (must match TX)
SPS           = SAMPLE_RATE // BAUD_SYMBOLS   # = 20 samples per symbol
FFT_PAD       = 4096
FREQ_SPACING  = 750            # Hz between 8-FSK tones (must match TX)
N_SYMBOLS     = 8              # 8-FSK → 3 bits per symbol
CNN_THRESHOLD = 0.70           # min softmax confidence to trust CNN
SUBPART_SIZE  = 8192           # must match transmitter_backend.py
PACKET_SIZE   = 256            # must match transmitter_backend.py

FRAME_SYNC    = bytes([0xBB, 0x44])   # serial frame from RX ESP32
ACK_BYTE      = 0x06
NACK_BYTE     = 0x15
CALIB_DONE    = 0xCA
CMD_CALIBRATE = 0xC2

TX_FEEDBACK   = "http://localhost:5001/surface_feedback"
OUTPUT_DIR    = "./received_files"
os.makedirs(OUTPUT_DIR, exist_ok=True)

SURFACE_BASE = {
    "glass":    2000,
    "hardwood": 1500,
    "softwood": 1000,
    "metal":    3500,
    "unknown":  2000,
}

STATE = {
    "listening":       False,
    "receiving":       False,
    "serial_port":     None,
    "cnn_model":       None,
    "surface":         "unknown",
    "base_freq":       2000,
    "freqs":           [2000 + i * FREQ_SPACING for i in range(N_SYMBOLS + 1)],
    "noise_floor":     0.0,
    "ecc_corrections": 0,
    "aes_password":    "SurfaceSyncSecretKey2025",
    "cnn_trained":     False,
    "tap_threshold":   80,     # RMS threshold in ADC counts — sensitive default
    "tap_active":      False,  # currently above threshold
    "tap_count":       0,
}

_adc_queue:     queue.Queue = queue.Queue(maxsize=8192)
_subpart_store: dict = {}
_file_parts:    dict = {}

# ── Logging ───────────────────────────────────────────────────────────────────
def log(msg: str, level: str = "INFO") -> None:
    socketio.emit("log", {"message": msg, "level": level})
    print(f"[RX-{level}] {msg}")

# ── CRC-16 / IBM ──────────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF

# ── AES-256-CBC ───────────────────────────────────────────────────────────────
def derive_key(password: str) -> bytes:
    return SHA256.new(password.encode()).digest()

def decrypt_subpart(iv: bytes, ct: bytes, key: bytes):
    try:
        return unpad(AES.new(key, AES.MODE_CBC, iv).decrypt(ct), AES.block_size)
    except (ValueError, KeyError):
        return None

# ── Frequency helpers ─────────────────────────────────────────────────────────
def compute_freqs(base_freq: int) -> list:
    return [base_freq + i * FREQ_SPACING for i in range(N_SYMBOLS + 1)]

# ── MLP model (sklearn — works on Python 3.13) ───────────────────────────────
def build_cnn_model():
    """
    Uses sklearn MLPClassifier as a drop-in replacement for the Keras CNN.
    Input  : flattened SPS=44 normalised float32 features
    Output : one of 8 symbol classes (0-7)
    Weights: saved/loaded from OUTPUT_DIR/cnn_fsk8_weights.pkl
    """
    global _clf
    if not CNN_AVAILABLE:
        return None

    wp = os.path.join(OUTPUT_DIR, "cnn_fsk8_weights.pkl")
    if os.path.exists(wp):
        try:
            _clf = joblib.load(wp)
            STATE["cnn_trained"] = True
            print(f"[RX] MLP weights loaded from {wp}")
        except Exception as e:
            print(f"[RX] Could not load weights: {e}")
            STATE["cnn_trained"] = False
            _clf = MLPClassifier(
                hidden_layer_sizes=(128, 64),
                activation="relu",
                max_iter=1,
                warm_start=True,
                random_state=42,
            )
    else:
        STATE["cnn_trained"] = False
        _clf = MLPClassifier(
            hidden_layer_sizes=(128, 64),
            activation="relu",
            max_iter=1,
            warm_start=True,
            random_state=42,
        )
        print("[RX] No MLP weights found — FFT fallback active.")
        print("[RX]   Train with: python collect_and_train.py --simulate")

    # Return a wrapper object that mimics Keras model.predict() interface
    return _clf

# ── Bandpass filter ───────────────────────────────────────────────────────────
def bandpass(samples: np.ndarray, lo: float, hi: float) -> np.ndarray:
    nyq = SAMPLE_RATE / 2.0
    sos = butter(5, [lo / nyq, hi / nyq], btype="band", output="sos")
    return sosfilt(sos, samples)

# ── Symbol detection ──────────────────────────────────────────────────────────
def _fft_detect(window: np.ndarray, freqs: list) -> int:
    """Zero-padded FFT energy — fallback when CNN is untrained."""
    fv  = np.abs(np.fft.rfft(window, n=FFT_PAD))
    ff  = np.fft.rfftfreq(FFT_PAD, d=1.0 / SAMPLE_RATE)
    tol = FREQ_SPACING // 3
    en  = []
    for f in freqs[:N_SYMBOLS]:
        m = (ff >= f - tol) & (ff <= f + tol)
        en.append(float(np.sum(fv[m])) if m.any() else 0.0)
    return int(np.argmax(en))

def detect_symbol(window: np.ndarray, model, freqs: list) -> tuple:
    """Returns (symbol_index, method_str)."""
    norm = window / (np.max(np.abs(window)) + 1e-9)
    if CNN_AVAILABLE and model is not None and STATE["cnn_trained"]:
        try:
            x     = norm.astype(np.float32).reshape(1, -1)  # (1, 44) flat
            probs = model.predict_proba(x)[0]                # shape (8,)
            sym   = int(np.argmax(probs))
            if float(probs[sym]) >= CNN_THRESHOLD:
                return sym, "cnn"
        except Exception:
            pass
    return _fft_detect(window, freqs), "fft"

# ── 8-FSK demodulator ─────────────────────────────────────────────────────────
def demodulate_stream(samples: np.ndarray, freqs: list, model) -> bytes:
    """
    Slide SPS-sample windows, detect symbol per window, pack 8 tribits → 3 bytes.

    Packing matches tx_3bytes() in esp32_cam_transmitter.ino:
      bits[23:21] = sym0 ... bits[2:0] = sym7  → MSB-first 3 bytes
    """
    tribits = []
    cnn_n = fft_n = 0

    for start in range(0, len(samples) - SPS, SPS):
        win = samples[start: start + SPS]
        sym, method = detect_symbol(win, model, freqs)
        tribits.append(sym & 0x07)
        if method == "cnn":
            cnn_n += 1
        else:
            fft_n += 1

    result = bytearray()
    for i in range(0, len(tribits) - 7, 8):
        bits = 0
        for j in range(8):
            bits = (bits << 3) | (tribits[i + j] & 0x07)
        result.append((bits >> 16) & 0xFF)
        result.append((bits >>  8) & 0xFF)
        result.append( bits        & 0xFF)

    if tribits:
        cnn_pct = 100 * cnn_n // len(tribits)
        socketio.emit("cnn_stats", {
            "cnn_pct":    cnn_pct,
            "cnn_symbols": cnn_n,
            "fft_symbols": fft_n,
        })

    return bytes(result)

# ── Packet parser ─────────────────────────────────────────────────────────────
def parse_packet(stream: bytes):
    """
    Find 0xAA 0x55 acoustic sync and parse the wire packet
    built by build_acoustic_packet() in transmitter_backend.py.
    """
    idx = stream.find(b"\xAA\x55")
    if idx == -1:
        return None
    p = idx + 2
    if len(stream) < p + 13:
        return None

    sub_id    = struct.unpack_from(">H", stream, p)[0]; p += 2
    sub_total = struct.unpack_from(">H", stream, p)[0]; p += 2
    pkt_id    = struct.unpack_from(">H", stream, p)[0]; p += 2
    pkt_total = struct.unpack_from(">H", stream, p)[0]; p += 2
    flags     = stream[p];                               p += 1
    data_len  = struct.unpack_from(">H", stream, p)[0]; p += 2

    if len(stream) < p + data_len + 2:
        return None

    has_iv = bool(flags & 0x01)
    if has_iv:
        if data_len < 16:
            return None
        iv    = stream[p: p + 16]
        chunk = stream[p + 16: p + data_len]
    else:
        iv    = None
        chunk = stream[p: p + data_len]

    p     += data_len
    body   = stream[idx + 2: p]
    rx_crc = struct.unpack_from(">H", stream, p)[0]

    return {
        "sub_id":    sub_id,
        "sub_total": sub_total,
        "pkt_id":    pkt_id,
        "pkt_total": pkt_total,
        "iv":        iv,
        "chunk":     chunk,
        "valid":     (crc16(body) == rx_crc),
    }

# ── Sub-part store + reassembly ───────────────────────────────────────────────
def store_packet(pkt: dict) -> None:
    sid = pkt["sub_id"]
    if sid not in _subpart_store:
        _subpart_store[sid] = {
            "total":     pkt["pkt_total"],
            "sub_total": pkt["sub_total"],
            "packets":   {},
            "iv":        None,
        }
    e = _subpart_store[sid]
    if pkt["iv"] is not None:
        e["iv"] = pkt["iv"]
    e["packets"][pkt["pkt_id"]] = pkt["chunk"]

def try_decrypt_subpart(sub_id: int, key: bytes) -> bool:
    e = _subpart_store.get(sub_id)
    if not e or len(e["packets"]) < e["total"] or e["iv"] is None:
        return False
    ct = b"".join(e["packets"][i] for i in range(e["total"]))
    pt = decrypt_subpart(e["iv"], ct, key)
    if pt is None:
        log(f"Decryption failed — sub-part {sub_id}", "ERROR")
        return False
    _file_parts[sub_id] = pt
    log(f"Sub-part {sub_id + 1}/{e['sub_total']} decrypted ({len(pt):,} B)", "SUCCESS")
    socketio.emit("subpart_done", {
        "sub_id":    sub_id,
        "sub_total": e["sub_total"],
        "size":      len(pt),
    })
    return True

def try_reassemble():
    if not _file_parts:
        return None
    st = next(iter(_subpart_store.values()))["sub_total"]
    if len(_file_parts) < st:
        return None
    return b"".join(_file_parts[i] for i in range(st))

def save_file(data: bytes) -> str:
    if   data[:2]  == b"\xFF\xD8": ext = ".jpg"
    elif data[:4]  == b"\x89PNG":  ext = ".png"
    elif data[:4]  == b"GIF8":     ext = ".gif"
    elif data[:4]  == b"%PDF":     ext = ".pdf"
    elif all(b < 128 for b in data[:64]): ext = ".txt"
    else:                               ext = ".bin"
    fpath = os.path.join(OUTPUT_DIR, f"received_{int(time.time())}{ext}")
    with open(fpath, "wb") as f:
        f.write(data)
    return fpath

# ── Surface sweep analysis ────────────────────────────────────────────────────
def analyze_sweep(samples: np.ndarray) -> tuple:
    if len(samples) < SAMPLE_RATE // 4:
        return "glass", 2000
    fv   = np.abs(np.fft.rfft(samples))
    ff   = np.fft.rfftfreq(len(samples), d=1.0 / SAMPLE_RATE)
    mask = (ff >= 1000) & (ff <= 7000)
    if not mask.any():
        return "unknown", 2000
    sv, sf = fv[mask], ff[mask]
    tot    = float(np.sum(sv)) + 1e-9

    def ber(lo: float, hi: float) -> float:
        m = (sf >= lo) & (sf <= hi)
        return float(np.sum(sv[m])) / tot if m.any() else 0.0

    pi  = int(np.argmax(sv))
    pf  = int(sf[pi])
    sn  = min(float(sv[pi]) / (float(np.mean(sv)) + 1e-9) / 20.0, 1.0)
    sc  = {
        "glass":    ber(2000, 5000) + sn * 0.3,
        "hardwood": ber(1500, 4000),
        "softwood": ber(1000, 2500),
        "metal":    ber(3500, 6500) + sn * 0.2 + (0.15 if pf > 3200 else 0),
    }
    surf = max(sc, key=sc.__getitem__)
    return surf, SURFACE_BASE[surf]

def _report_to_tx(surface: str, base_freq: int) -> None:
    try:
        r = requests.post(
            TX_FEEDBACK,
            json={"surface": surface, "base_freq": base_freq},
            timeout=5,
        )
        log(f"Sweep → TX: {surface} @ {base_freq} Hz (HTTP {r.status_code})", "SUCCESS")
    except requests.RequestException as e:
        log(f"TX feedback failed: {e}", "ERROR")

# ── Serial reader thread ──────────────────────────────────────────────────────
def _serial_reader(ser: serial.Serial) -> None:
    buf = bytearray()
    log("Serial reader started.", "INFO")
    while STATE["listening"]:
        raw = ser.read(ser.in_waiting or 1)
        if not raw:
            continue
        buf.extend(raw)
        while len(buf) >= 6:
            idx = buf.find(FRAME_SYNC)
            if idx == -1:
                buf = buf[-2:]; break
            if idx > 0:
                for b in buf[:idx]:
                    if b == CALIB_DONE:
                        log("ESP32 calibration done.", "SUCCESS")
                buf = buf[idx:]; continue
            if len(buf) < 4:
                break
            lw       = (buf[2] << 8) | buf[3]
            has_pre  = bool(lw & 0x8000)
            data_len = lw & 0x7FFF
            total    = 4 + data_len + 2
            if len(buf) < total:
                break
            fd  = buf[4: 4 + data_len]
            fc  = (buf[4 + data_len] << 8) | buf[4 + data_len + 1]
            buf = buf[total:]
            if crc16(fd) != fc:
                ser.write(bytes([NACK_BYTE]))
                STATE["ecc_corrections"] += 1
                continue
            ser.write(bytes([ACK_BYTE]))
            samp = np.frombuffer(fd, dtype=">i2").astype(np.float32)
            try:
                _adc_queue.put_nowait((samp, has_pre))
            except queue.Full:
                pass
    log("Serial reader stopped.", "INFO")

# ── DSP pipeline thread ───────────────────────────────────────────────────────
def _dsp_pipeline() -> None:
    model   = STATE.get("cnn_model")
    aes_key = derive_key(STATE["aes_password"])
    freqs   = STATE["freqs"]

    acc          = np.array([], dtype=np.float32)
    sweep_buf    = np.array([], dtype=np.float32)
    BLOCK        = SAMPLE_RATE // 10    # 100 ms = 4410 samples
    sweep_active = False
    sweep_dl     = 0.0
    preamble     = False
    locked       = False
    skip         = 0
    GUARD        = int(SAMPLE_RATE * 0.023)   # 23 ms preamble+guard

    log("DSP pipeline started.", "INFO")

    while STATE["listening"]:
        try:
            samp, has_pre = _adc_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        acc = np.concatenate([acc, samp])

        # Preamble detected by RX ESP32 hardware RMS gate
        if has_pre and not STATE["receiving"]:
            preamble           = True
            STATE["receiving"] = True
            locked             = False
            skip               = GUARD
            socketio.emit("status", {"state": "receiving"})
            log("Preamble detected — demodulating.", "SUCCESS")

        while len(acc) >= BLOCK:
            block = acc[:BLOCK]
            acc   = acc[BLOCK:]

            socketio.emit("waveform_raw", {"samples": block[::8].tolist()})

            # Bandpass: 800 Hz–9500 Hz covers all 8-FSK tones on all surfaces
            clean = bandpass(block, 800.0, 9500.0)
            socketio.emit("waveform_clean", {"samples": clean[::8].tolist()})

            # SNR + dominant frequency
            sig_rms   = float(np.sqrt(np.mean(clean ** 2)))
            noise_rms = STATE["noise_floor"] + 1e-9
            snr       = 20.0 * np.log10(sig_rms / noise_rms)
            fv = np.abs(np.fft.rfft(clean))
            ff = np.fft.rfftfreq(len(clean), d=1.0 / SAMPLE_RATE)
            bd = (ff >= 800) & (ff <= 9500)
            df = int(ff[bd][np.argmax(fv[bd])]) if bd.any() else 0
            # ── Tap / vibration detection ────────────────────────────────
            # block contains raw int16 ADC samples recentred around 0
            # Range is approx -2048 to +2047 (12-bit ADC, midpoint removed)
            # We compare RMS directly in ADC counts (not normalised floats)
            raw_rms = float(np.sqrt(np.mean(block ** 2)))  # ADC count units

            # Threshold in ADC counts:
            #   noise_floor is calibrated in ADC counts (same units as block)
            #   tap_threshold starts at 150 counts (~7% of full scale)
            #   After calibration it is set to max(noise_floor * 3, 80)
            thr = STATE["tap_threshold"]

            currently_above = raw_rms > thr
            if currently_above and not STATE["tap_active"]:
                STATE["tap_active"] = True
                STATE["tap_count"] += 1
                socketio.emit("tap_detected", {
                    "rms":       round(raw_rms, 1),
                    "threshold": round(thr, 1),
                    "count":     STATE["tap_count"],
                })
                log(f"Tap #{STATE['tap_count']}  RMS={raw_rms:.0f}  thr={thr:.0f}", "DATA")
            elif not currently_above and STATE["tap_active"]:
                STATE["tap_active"] = False
                socketio.emit("tap_released", {"rms": round(raw_rms, 1)})

            socketio.emit("signal_stats", {
                "snr_db":    round(snr, 1),
                "freq_hz":   df,
                "signal_pct": min(100, max(0, int((snr + 20) * 2.5))),
                "ecc":       STATE["ecc_corrections"],
                "rms":       round(raw_rms, 1),
                "tap_active": currently_above,
                "tap_count":  STATE["tap_count"],
            })

            # Sweep collection
            if sweep_active:
                sweep_buf = np.concatenate([sweep_buf, clean])
                if time.time() > sweep_dl:
                    sweep_active = False
                    surf, bf     = analyze_sweep(sweep_buf)
                    STATE.update({"surface": surf, "base_freq": bf,
                                  "freqs": compute_freqs(bf)})
                    freqs = STATE["freqs"]
                    socketio.emit("sweep_result", {"surface": surf, "freqs": freqs})
                    threading.Thread(target=_report_to_tx,
                                     args=(surf, bf), daemon=True).start()
                    sweep_buf = np.array([], dtype=np.float32)
                continue

            if not preamble:
                continue

            # Skip preamble + guard samples
            if skip > 0:
                sk    = min(skip, len(clean))
                clean = clean[sk:]
                skip -= sk
                if skip > 0 or len(clean) == 0:
                    continue

            # Phase-align to symbol boundary
            if not locked:
                leftover = len(clean) % SPS
                if leftover:
                    clean = clean[leftover:]
                if len(clean) < SPS:
                    continue
                locked = True
                log("Symbol phase locked.", "INFO")

            # 8-FSK demodulation
            raw_bytes = demodulate_stream(clean, freqs, model)
            if len(raw_bytes) < 16:
                continue

            pkt = parse_packet(raw_bytes)
            if pkt is None:
                continue
            if not pkt["valid"]:
                STATE["ecc_corrections"] += 1
                socketio.emit("ecc", {"count": STATE["ecc_corrections"]})
                continue

            store_packet(pkt)
            sid = pkt["sub_id"]

            # Progress
            rx_tot  = sum(len(e["packets"]) for e in _subpart_store.values())
            exp_tot = sum(e["total"] for e in _subpart_store.values() if e["total"])
            if exp_tot > 0:
                socketio.emit("progress", {
                    "percent":  round(rx_tot / exp_tot * 100, 1),
                    "received": rx_tot,
                    "total":    exp_tot,
                })

            # Decrypt + reassemble
            if try_decrypt_subpart(sid, aes_key):
                data = try_reassemble()
                if data is not None:
                    preamble = locked = False
                    skip               = 0
                    STATE["receiving"] = False
                    fpath  = save_file(data)
                    name   = os.path.basename(fpath)
                    is_txt = fpath.endswith(".txt")
                    log(f"File received: {name} ({len(data):,} B)", "SUCCESS")
                    socketio.emit("status", {"state": "done"})
                    socketio.emit("file_ready", {
                        "filename": name,
                        "size":     len(data),
                        "is_text":  is_txt,
                        "text":     data.decode("utf-8", errors="replace") if is_txt else None,
                    })
                    _subpart_store.clear()
                    _file_parts.clear()
                    socketio.emit("status", {"state": "listening"})

    log("DSP pipeline stopped.", "INFO")

# ── Flask routes ──────────────────────────────────────────────────────────────
@app.route("/")
def index():
    html = os.path.join(os.path.dirname(os.path.abspath(__file__)), "receiver.html")
    if os.path.exists(html):
        return flask_send_file(html)
    return (
        "<h2>SurfaceSync RX Backend v2.1</h2>"
        "<p>Put <b>receiver.html</b> in the same folder as this file, "
        "then open <a href='http://localhost:5002/'>http://localhost:5002/</a></p>"
    )

@app.route("/ping")
def ping():
    return jsonify({"ok": True}), 200

@app.route("/start_listening", methods=["POST"])
def start_listening():
    if STATE["listening"]:
        return jsonify({"error": "Already listening"}), 409

    body = request.get_json(silent=True) or {}
    port = body.get("port")
    baud = int(body.get("baud", 921600))

    # Load CNN if TensorFlow is available
    if CNN_AVAILABLE and STATE["cnn_model"] is None:
        log("Loading MLP classifier…", "INFO")
        STATE["cnn_model"] = build_cnn_model()
        trained = STATE["cnn_trained"]
        socketio.emit("cnn_status", {
            "trained": trained,
            "message": "CNN active (trained weights)" if trained
                       else "FFT fallback (no weights — POST /train_cnn to train)",
        })
    elif not CNN_AVAILABLE:
        socketio.emit("cnn_status", {
            "trained": False,
            "message": "FFT only (pip install scikit-learn to enable MLP)",
        })

    # Open serial port if specified
    if port:
        try:
            ser = serial.Serial(port, baud, timeout=2.0)
            STATE["serial_port"] = ser
            log(f"Serial: {port} @ {baud} baud", "SUCCESS")
        except serial.SerialException as e:
            return jsonify({"error": str(e)}), 500
    else:
        log("No serial port — simulation mode.", "INFO")

    STATE["listening"] = True
    socketio.emit("status", {"state": "listening"})

    if STATE.get("serial_port"):
        threading.Thread(
            target=_serial_reader,
            args=(STATE["serial_port"],),
            daemon=True,
        ).start()
    threading.Thread(target=_dsp_pipeline, daemon=True).start()
    log("Receiver listening for 8-FSK transmissions.", "SUCCESS")
    return jsonify({"status": "listening"}), 200

@app.route("/stop_listening", methods=["POST"])
def stop_listening():
    STATE["listening"] = STATE["receiving"] = False
    socketio.emit("status", {"state": "idle"})
    ser = STATE.get("serial_port")
    if ser and ser.is_open:
        ser.close()
    log("Receiver stopped.", "INFO")
    return jsonify({"status": "stopped"}), 200

@app.route("/calibrate", methods=["POST"])
def calibrate():
    if not STATE["listening"]:
        return jsonify({"error": "Call /start_listening first"}), 400

    def _cal():
        target = SAMPLE_RATE * 5
        col    = np.array([], dtype=np.float32)
        dl     = time.time() + 6
        ser    = STATE.get("serial_port")
        if ser and ser.is_open:
            ser.write(bytes([CMD_CALIBRATE]))
        while len(col) < target and time.time() < dl:
            try:
                s, _ = _adc_queue.get(timeout=0.1)
                col  = np.concatenate([col, s])
            except queue.Empty:
                pass
        if len(col) > 0:
            rms = float(np.sqrt(np.mean(col ** 2)))
            STATE["noise_floor"] = rms
            # Set tap threshold = 3× noise floor, minimum 80 ADC counts
            # This works in ADC count units (same as block RMS in DSP pipeline)
            auto_thr = max(rms * 3.0, 80.0)
            STATE["tap_threshold"] = auto_thr
            log(f"Noise floor: {rms:.1f} ADC counts  →  tap threshold: {auto_thr:.1f}", "INFO")
            socketio.emit("calibration_done", {
                "noise_floor": rms,
                "tap_threshold": auto_thr,
            })
            socketio.emit("calibration_complete", {
                "noise_floor":   round(rms, 1),
                "tap_threshold": round(auto_thr, 1),
                "message": f"Ready! Noise={rms:.0f} counts. Tap threshold={auto_thr:.0f} counts."
            })
            log(f"Calibration complete. Noise={rms:.1f} ADC counts. "
                f"Tap threshold auto-set to {auto_thr:.1f} ADC counts. "
                f"Now tap the piezo disc to test!", "SUCCESS")

    threading.Thread(target=_cal, daemon=True).start()
    return jsonify({"status": "calibrating"}), 202

@app.route("/train_cnn", methods=["POST"])
def train_cnn():
    """
    Train the MLP classifier on labelled symbol windows.
    Uses scikit-learn — works on Python 3.13.
    JSON: { "windows": [[44 floats], ...], "labels": [0-7, ...] }
    """
    import traceback

    if not CNN_AVAILABLE:
        return jsonify({"error": "scikit-learn not installed. Run: pip install scikit-learn"}), 503

    # Build model lazily
    if STATE["cnn_model"] is None:
        STATE["cnn_model"] = build_cnn_model()

    try:
        body = request.get_json(force=True, silent=True)
        if body is None:
            return jsonify({"error": "Could not parse JSON body"}), 400

        wins = np.array(body.get("windows", []), dtype=np.float32)
        lbls = np.array(body.get("labels",  []), dtype=np.int32)

        if len(wins) == 0 or len(wins) != len(lbls):
            return jsonify({"error": "windows and labels must be equal-length and non-empty"}), 400

        # Flatten + normalise: (N, 44)
        norms = wins / (np.max(np.abs(wins), axis=1, keepdims=True) + 1e-9)
        X     = norms.reshape(len(norms), -1)

        epochs = int(body.get("epochs", 30))
        print(f"[RX-INFO] Training MLP on {len(X)} windows, {epochs} iterations...")

        clf = MLPClassifier(
            hidden_layer_sizes=(256, 128, 64),
            activation="relu",
            solver="adam",
            max_iter=epochs,
            random_state=42,
            verbose=True,
            early_stopping=True,
            validation_fraction=0.15,
            n_iter_no_change=10,
        )
        clf.fit(X, lbls)

        # Accuracy on training set
        acc  = float(clf.score(X, lbls))
        loss = float(clf.best_validation_score_ if hasattr(clf, "best_validation_score_") else 0.0)

        # Save
        wp = os.path.join(OUTPUT_DIR, "cnn_fsk8_weights.pkl")
        joblib.dump(clf, wp)
        STATE["cnn_model"]   = clf
        STATE["cnn_trained"] = True
        global _clf
        _clf = clf

        log(f"MLP trained on {len(X)} windows — acc={acc:.3f}", "SUCCESS")
        socketio.emit("cnn_status", {"trained": True,
                                     "message": f"MLP trained (acc {acc:.1%})"})
        return jsonify({"status": "trained", "accuracy": acc, "loss": loss}), 200

    except Exception as e:
        tb = traceback.format_exc()
        print(f"[RX-ERROR] train_cnn:\n{tb}")
        return jsonify({"error": str(e), "traceback": tb}), 500

@app.route("/download/<filename>")
def download_file(filename: str):
    fpath = os.path.join(OUTPUT_DIR, filename)
    if not os.path.exists(fpath):
        return jsonify({"error": "Not found"}), 404
    return flask_send_file(fpath, as_attachment=True)

@app.route("/list_files")
def list_files():
    files = []
    for fn in sorted(os.listdir(OUTPUT_DIR)):
        fp = os.path.join(OUTPUT_DIR, fn)
        if os.path.isfile(fp) and not fn.endswith(".h5"):
            files.append({
                "name": fn,
                "size": os.path.getsize(fp),
                "time": int(os.path.getmtime(fp)),
            })
    return jsonify({"files": files}), 200

@app.route("/set_key", methods=["POST"])
def set_key():
    body = request.get_json(force=True)
    pwd  = body.get("password", "")
    if len(pwd) < 8:
        return jsonify({"error": "Password must be >= 8 characters"}), 400
    STATE["aes_password"] = pwd
    log("AES-256 key updated.", "INFO")
    return jsonify({"status": "ok"}), 200

@app.route("/list_ports")
def list_ports():
    return jsonify({
        "ports": [
            {"device": p.device, "description": p.description}
            for p in serial.tools.list_ports.comports()
        ]
    }), 200

@app.route("/set_threshold", methods=["POST"])
def set_threshold():
    body = request.get_json(force=True)
    thr  = float(body.get("threshold", 150))
    STATE["tap_threshold"] = thr
    log(f"Tap threshold set to {thr:.1f} ADC counts", "INFO")
    return jsonify({"status": "ok", "threshold": thr}), 200

@app.route("/tap_stats")
def tap_stats():
    return jsonify({
        "tap_count":     STATE["tap_count"],
        "tap_active":    STATE["tap_active"],
        "tap_threshold": STATE["tap_threshold"],
        "noise_floor":   STATE["noise_floor"],
    }), 200

@app.route("/status")
def get_status():
    return jsonify({
        "listening":       STATE["listening"],
        "receiving":       STATE["receiving"],
        "surface":         STATE["surface"],
        "freqs":           STATE["freqs"],
        "noise_floor":     round(STATE["noise_floor"], 6),
        "ecc_corrections": STATE["ecc_corrections"],
        "cnn_trained":     STATE["cnn_trained"],
        "tf_available":    CNN_AVAILABLE,
    }), 200

@app.route("/cnn_status")
def cnn_status():
    return jsonify({
        "tf_available": CNN_AVAILABLE,
        "model_loaded": STATE["cnn_model"] is not None,
        "trained":      STATE["cnn_trained"],
        "sps":          SPS,
        "sample_rate":  SAMPLE_RATE,
        "n_symbols":    N_SYMBOLS,
        "weights":      os.path.join(OUTPUT_DIR, "cnn_fsk8_weights.h5"),
    }), 200

@app.route("/inject_samples", methods=["POST"])
def inject_samples():
    """Simulation endpoint — inject raw ADC samples without hardware."""
    body = request.get_json(force=True)
    s    = np.array(body.get("samples", []), dtype=np.float32)
    if len(s) == 0:
        return jsonify({"error": "Empty samples array"}), 400
    _adc_queue.put_nowait((s, False))
    return jsonify({"queued": len(s)}), 200

# ── Socket.IO ─────────────────────────────────────────────────────────────────
@socketio.on("connect")
def on_connect():
    emit("status", {
        "state":   "receiving" if STATE["receiving"] else
                   "listening" if STATE["listening"] else "idle",
        "surface": STATE["surface"],
        "freqs":   STATE["freqs"],
    })

@socketio.on("ping_backend")
def on_ping():
    emit("pong", {"ts": time.time()})

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 58)
    print("  SurfaceSync Receiver Backend v2.1  [8-FSK CNN]")
    print("  http://localhost:5002")
    print()
    print("  Open browser → http://localhost:5002/")
    print()
    print("  Endpoints:")
    print("   POST /start_listening   start ADC + DSP pipeline")
    print("   POST /stop_listening    halt")
    print("   POST /calibrate         5-sec noise floor calibration")
    print("   POST /train_cnn         train 8-FSK CNN on labelled data")
    print("   GET  /cnn_status        CNN info")
    print("   GET  /list_files        received files")
    print("   GET  /download/<file>   download received file")
    print("   GET  /status            pipeline state")
    print("=" * 58)
    socketio.run(
        app,
        host="0.0.0.0",
        port=5002,
        debug=False,
        allow_unsafe_werkzeug=True,
    )