# -*- coding: utf-8 -*-
import sys
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

"""
SurfaceSync - Transmitter Backend  v2.1  [8-FSK, transmitter-only fixed]

Key fixes vs v2.0
-----------------
1. Surface sweep no longer hangs forever when no receiver is running.
   SWEEP_TIMEOUT is 3 s — after that it uses Glass defaults and proceeds.
2. /ping endpoint added so frontend can confirm backend is alive.
3. FTDI also matched in auto-detect keyword list.

Dependencies
------------
pip install flask flask-socketio flask-cors pycryptodome pyserial
"""

import os, time, struct, threading
import serial, serial.tools.list_ports
from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
from Crypto.Cipher import AES
from Crypto.Util.Padding import pad
from Crypto.Hash import SHA256

app = Flask(__name__)
app.config["SECRET_KEY"] = os.urandom(32)
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

SUBPART_SIZE  = 8192
PACKET_SIZE   = 256
RETRY_LIMIT   = 5
CMD_TIMEOUT   = 3.0
SWEEP_TIMEOUT = 3.0   # short — so transmitter-only mode never stalls
N_SYMBOLS     = 8
FREQ_SPACING  = 750

SYNC      = bytes([0xAA, 0x55])
ACK_BYTE  = 0x06
NACK_BYTE = 0x15
SWEEP_DONE= 0xA0
CMD_SWEEP     = 0x01
CMD_SET_FREQ  = 0x02
CMD_PACKET    = 0x03
CMD_RESET     = 0x04

SURFACE_BASE = {"glass": 2000, "hardwood": 1500, "softwood": 1000,
                "metal": 3500, "unknown": 2000}

STATE = {
    "transmitting": False, "serial_port": None,
    "surface": "glass", "base_freq": 2000,
    "freqs": [2000 + i * FREQ_SPACING for i in range(N_SYMBOLS + 1)],
    "aes_password": "SurfaceSyncSecretKey2025",
}

def log(msg, level="INFO"):
    socketio.emit("log", {"message": msg, "level": level})
    print(f"[TX-{level}] {msg}")

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF

def derive_key(password: str) -> bytes:
    return SHA256.new(password.encode()).digest()

def encrypt_subpart(plaintext: bytes, key: bytes):
    cipher = AES.new(key, AES.MODE_CBC)
    return cipher.iv, cipher.encrypt(pad(plaintext, AES.block_size))

def compute_freqs(base_freq: int) -> list:
    return [base_freq + i * FREQ_SPACING for i in range(N_SYMBOLS + 1)]

def build_serial_frame(cmd: int, data: bytes = b"") -> bytes:
    length = len(data)
    crc    = crc16(data) if data else 0xFFFF
    frame  = bytearray(SYNC)
    frame.append(cmd)
    frame.append((length >> 8) & 0xFF)
    frame.append(length & 0xFF)
    frame += data
    frame.append((crc >> 8) & 0xFF)
    frame.append(crc & 0xFF)
    return bytes(frame)

def send_serial_cmd(cmd: int, data: bytes, ser: serial.Serial,
                    retries: int = RETRY_LIMIT) -> bool:
    frame = build_serial_frame(cmd, data)
    for attempt in range(retries):
        ser.write(frame)
        deadline = time.time() + CMD_TIMEOUT
        while time.time() < deadline:
            if ser.in_waiting:
                resp = ser.read(1)
                if resp and resp[0] == ACK_BYTE:
                    return True
                if resp and resp[0] == NACK_BYTE:
                    break
            time.sleep(0.001)
        log(f"CMD 0x{cmd:02X} attempt {attempt+1} no ACK", "ERROR")
    return False

def build_acoustic_packet(sub_id, sub_total, pkt_id, pkt_total, iv, chunk) -> bytes:
    flags   = 0x01 if iv is not None else 0x00
    payload = (iv or b"") + chunk
    header  = (struct.pack(">H", sub_id)    + struct.pack(">H", sub_total) +
               struct.pack(">H", pkt_id)    + struct.pack(">H", pkt_total) +
               struct.pack("B",  flags)     + struct.pack(">H", len(payload)))
    body = header + payload
    return b"\xAA\x55" + body + struct.pack(">H", crc16(body))

def build_all_packets(raw_data: bytes, password: str) -> list:
    key      = derive_key(password)
    subparts = [raw_data[i: i + SUBPART_SIZE] for i in range(0, len(raw_data), SUBPART_SIZE)]
    log(f"Encrypting {len(raw_data):,} B → {len(subparts)} sub-part(s)", "INFO")
    all_pkts = []
    for sub_id, subpart in enumerate(subparts):
        iv, ct = encrypt_subpart(subpart, key)
        chunks = [ct[i: i + PACKET_SIZE] for i in range(0, len(ct), PACKET_SIZE)]
        for pkt_id, chunk in enumerate(chunks):
            all_pkts.append(build_acoustic_packet(
                sub_id, len(subparts), pkt_id, len(chunks),
                iv if pkt_id == 0 else None, chunk))
    log(f"{len(all_pkts)} packets ready", "DATA")
    return all_pkts

def run_surface_sweep(ser: serial.Serial):
    """Run sweep; fall back to Glass if no receiver responds within SWEEP_TIMEOUT."""
    log("Surface sweep starting…", "INFO")
    ser.write(build_serial_frame(CMD_SWEEP, b""))
    # Wait for ACK
    dl = time.time() + CMD_TIMEOUT
    while time.time() < dl:
        if ser.in_waiting and ser.read(1)[0] == ACK_BYTE:
            break
        time.sleep(0.001)
    # Wait for SWEEP_DONE byte from ESP32
    dl = time.time() + 30
    while time.time() < dl:
        if ser.in_waiting and ser.read(1)[0] == SWEEP_DONE:
            log("Sweep tones done. Checking for receiver…", "INFO")
            break
        time.sleep(0.01)
    # Short wait for receiver feedback
    dl = time.time() + SWEEP_TIMEOUT
    while time.time() < dl:
        if STATE.get("_sweep_result"):
            return STATE.pop("_sweep_result")
        time.sleep(0.1)
    log("No receiver feedback — using Glass defaults.", "INFO")
    return "glass", SURFACE_BASE["glass"]

def apply_surface_frequencies(surface, base_freq, ser):
    freqs = compute_freqs(base_freq)
    STATE.update({"surface": surface, "base_freq": base_freq, "freqs": freqs})
    log(f"Surface={surface}  base={base_freq} Hz", "SUCCESS")
    socketio.emit("sweep_result", {"surface": surface, "freqs": freqs})
    if ser and ser.is_open:
        freq_data = b"".join(struct.pack(">H", f) for f in freqs)
        ok = send_serial_cmd(CMD_SET_FREQ, freq_data, ser)
        log("Freqs sent to ESP32." if ok else "Failed to send freqs.", "SUCCESS" if ok else "ERROR")

def run_transmission_pipeline(raw_data: bytes, password: str):
    STATE["transmitting"] = True
    socketio.emit("status", {"state": "transmitting"})
    try:
        ser     = STATE.get("serial_port")
        dry_run = (ser is None or not ser.is_open)
        if dry_run:
            log("Simulation mode — no serial port connected.", "INFO")
            freqs = compute_freqs(SURFACE_BASE["glass"])
            STATE.update({"surface": "glass", "base_freq": SURFACE_BASE["glass"], "freqs": freqs})
            socketio.emit("sweep_result", {"surface": "glass", "freqs": freqs})
        else:
            surface, base_freq = run_surface_sweep(ser)
            apply_surface_frequencies(surface, base_freq, ser)

        packets = build_all_packets(raw_data, password)
        total   = len(packets)
        sent    = 0
        for pkt_bytes in packets:
            ok = True if dry_run else send_serial_cmd(CMD_PACKET, pkt_bytes, ser)
            if dry_run:
                time.sleep(0.005)
            if ok:
                sent += 1
            else:
                log(f"Packet {sent+1} failed after {RETRY_LIMIT} retries.", "ERROR")
            pct = (sent / total) * 100.0
            socketio.emit("progress", {"percent": round(pct, 1), "sent": sent, "total": total})
            if sent % max(1, total // 20) == 0:
                log(f"Sent {sent}/{total} ({pct:.0f}%)", "DATA")

        if sent == total:
            log(f"Done — {sent} packets transmitted.", "SUCCESS")
            socketio.emit("status", {"state": "done"})
        else:
            log(f"Incomplete — {total-sent} packets lost.", "ERROR")
            socketio.emit("status", {"state": "error"})
    except Exception as exc:
        log(f"Pipeline error: {exc}", "ERROR")
        import traceback; traceback.print_exc()
        socketio.emit("status", {"state": "error"})
    finally:
        STATE["transmitting"] = False

@app.route("/")
def index():
    # Try to serve the HTML file if it sits next to this script
    html = os.path.join(os.path.dirname(os.path.abspath(__file__)), "transmitter.html")
    if os.path.exists(html):
        from flask import send_file as sf
        return sf(html)
    return ("<h2>SurfaceSync TX Backend v2.1</h2>"
            "<p>Put transmitter.html next to this file, then visit "
            "<a href='/'>http://localhost:5001/</a></p>")

@app.route("/ping")
def ping():
    return jsonify({"ok": True}), 200

@app.route("/upload", methods=["POST"])
def upload():
    if STATE["transmitting"]:
        return jsonify({"error": "Already transmitting"}), 409
    password = STATE["aes_password"]
    raw_data = None
    if request.files.get("file"):
        f        = request.files["file"]
        raw_data = f.read()
        password = request.form.get("password", password)
        log(f"File: '{f.filename}'  {len(raw_data):,} B", "DATA")
    elif request.is_json:
        body     = request.get_json()
        raw_data = body.get("message", "").encode("utf-8")
        password = body.get("password", password)
        log(f"Message {len(raw_data)} B", "DATA")
    else:
        return jsonify({"error": "Send multipart file or JSON {message}"}), 400
    if not raw_data:
        return jsonify({"error": "Empty payload"}), 400
    threading.Thread(target=run_transmission_pipeline, args=(raw_data, password), daemon=True).start()
    return jsonify({"status": "queued", "size_bytes": len(raw_data), "modulation": "8-FSK"}), 202

@app.route("/surface_feedback", methods=["POST"])
def surface_feedback():
    body    = request.get_json(force=True)
    surface = body.get("surface", "unknown").lower()
    freq    = int(body.get("base_freq", SURFACE_BASE.get(surface, 2000)))
    STATE["_sweep_result"] = (surface, freq)
    log(f"Sweep feedback: {surface} @ {freq} Hz", "SUCCESS")
    return jsonify({"ack": True}), 200

@app.route("/connect_serial", methods=["POST"])
def connect_serial():
    body = request.get_json(force=True) or {}
    port = body.get("port")
    baud = int(body.get("baud", 921600))
    if not port:
        for p in serial.tools.list_ports.comports():
            if any(k in p.description.lower()
                   for k in ["cp210", "ch340", "uart", "usb serial", "ftdi"]):
                port = p.device; break
    if not port:
        return jsonify({"error": "No serial port found"}), 404
    try:
        existing = STATE.get("serial_port")
        if existing and existing.is_open:
            existing.close()
        ser = serial.Serial(port, baud, timeout=2.0)
        STATE["serial_port"] = ser
        log(f"Serial: {port} @ {baud}", "SUCCESS")
        return jsonify({"status": "connected", "port": port}), 200
    except serial.SerialException as e:
        return jsonify({"error": str(e)}), 500

@app.route("/set_key", methods=["POST"])
def set_key():
    body = request.get_json(force=True)
    pwd  = body.get("password", "")
    if len(pwd) < 8:
        return jsonify({"error": "Password >= 8 chars"}), 400
    STATE["aes_password"] = pwd
    log("AES key updated.", "INFO")
    return jsonify({"status": "ok"}), 200

@app.route("/list_ports")
def list_ports():
    return jsonify({"ports": [{"device": p.device, "description": p.description}
                               for p in serial.tools.list_ports.comports()]}), 200

@app.route("/status")
def get_status():
    return jsonify({
        "transmitting": STATE["transmitting"],
        "surface":      STATE["surface"],
        "base_freq":    STATE["base_freq"],
        "freqs":        STATE["freqs"],
        "serial_open":  bool(STATE.get("serial_port") and STATE["serial_port"].is_open),
    }), 200

@socketio.on("connect")
def on_connect():
    emit("status", {"state": "transmitting" if STATE["transmitting"] else "idle",
                    "surface": STATE["surface"], "freqs": STATE["freqs"]})

@socketio.on("ping_backend")
def on_ping():
    emit("pong", {"ts": time.time()})

if __name__ == "__main__":
    print("=" * 55)
    print("  SurfaceSync TX Backend v2.1")
    print("  http://localhost:5001")
    print("  Open browser -> http://localhost:5001/")
    print("=" * 55)
    socketio.run(app, host="0.0.0.0", port=5001, debug=False, allow_unsafe_werkzeug=True)