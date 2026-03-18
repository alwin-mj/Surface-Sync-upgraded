# -*- coding: utf-8 -*-
"""
SurfaceSync – CNN Synthetic Trainer
====================================
Standalone script. No Flask. No server.
Just generates data and POSTs it to receiver_backend.py.

Usage:
  python collect_and_train.py --simulate
"""

import argparse
import math
import time
import json
import urllib.request
import urllib.error

import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
SAMPLE_RATE      = 44100
BAUD_SYMBOLS     = 1000
SPS              = SAMPLE_RATE // BAUD_SYMBOLS   # 44
FREQ_SPACING     = 750
N_SYMBOLS        = 8
RX_BACKEND       = "http://localhost:5002"
WINDOWS_PER_TONE = 300


def make_windows(freq_hz, n, noise=0.15):
    wins = []
    for _ in range(n):
        phase = float(np.random.uniform(0, 2 * math.pi))
        t     = np.arange(SPS, dtype=np.float32) / SAMPLE_RATE
        sig   = np.sin(2 * math.pi * freq_hz * t + phase)
        sig  += (np.random.randn(SPS) * noise).astype(np.float32)
        mx    = float(np.max(np.abs(sig))) + 1e-9
        sig   = sig / mx
        wins.append(sig.tolist())
    return wins


def post_json(url, data, timeout=300):
    body    = json.dumps(data).encode("utf-8")
    req     = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", action="store_true", required=True,
                        help="Generate synthetic sine-wave training data")
    parser.add_argument("--base",    type=int,   default=2000,
                        help="Base frequency Hz (2000=glass, 1500=hardwood, "
                             "1000=softwood, 3500=metal)")
    parser.add_argument("--windows", type=int,   default=WINDOWS_PER_TONE,
                        help="Windows per symbol (default 300)")
    parser.add_argument("--noise",   type=float, default=0.15,
                        help="Noise level 0.0-1.0 (default 0.15)")
    parser.add_argument("--epochs",  type=int,   default=30)
    args = parser.parse_args()

    freqs = [args.base + i * FREQ_SPACING for i in range(N_SYMBOLS)]

    print("\nSurfaceSync CNN Synthetic Trainer")
    print(f"Base     : {args.base} Hz")
    print(f"8-FSK    : {freqs} Hz")
    print(f"Windows  : {args.windows} per symbol  ({args.windows * N_SYMBOLS} total)")
    print(f"Epochs   : {args.epochs}")
    print()

    # Check backend is alive
    print("Checking receiver_backend.py ...", end=" ", flush=True)
    try:
        with urllib.request.urlopen(f"{RX_BACKEND}/ping", timeout=3) as r:
            assert r.status == 200
        print("OK")
    except Exception:
        print("FAILED")
        print()
        print("ERROR: Cannot reach receiver_backend.py at", RX_BACKEND)
        print("       Make sure it is running in another terminal:")
        print("       python receiver_backend.py")
        return

    # Generate data
    print("\nGenerating synthetic data ...")
    all_windows = []
    all_labels  = []
    for sym_idx, freq in enumerate(freqs):
        wins = make_windows(freq, args.windows, noise=args.noise)
        all_windows.extend(wins)
        all_labels.extend([sym_idx] * len(wins))
        print(f"  Symbol {sym_idx}  {freq} Hz  →  {len(wins)} windows")

    print(f"\nTotal: {len(all_windows)} windows")
    print(f"\nSending to {RX_BACKEND}/train_cnn ...")
    print("Training takes 30-90 seconds — please wait ...\n")

    try:
        result = post_json(
            f"{RX_BACKEND}/train_cnn",
            {
                "windows":    all_windows,
                "labels":     all_labels,
                "epochs":     args.epochs,
                "batch_size": 32,
            },
            timeout=300,
        )
        print("\n========================================")
        print("  CNN TRAINING COMPLETE")
        print(f"  Accuracy : {result['accuracy']:.1%}")
        print(f"  Loss     : {result['loss']:.4f}")
        print("  Weights saved → received_files/cnn_fsk8_weights.h5")
        print("========================================")
        print("\nRestart receiver_backend.py to activate CNN.")

    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        print(f"\nERROR {e.code}: {body}")
    except urllib.error.URLError as e:
        print(f"\nERROR: Could not connect — {e.reason}")
    except Exception as e:
        print(f"\nERROR: {e}")


if __name__ == "__main__":
    main()