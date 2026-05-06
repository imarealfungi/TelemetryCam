"""
telemetry_broadcaster.py
Reads sim telemetry and broadcasts normalized data over localhost UDP
so the OBS script can pick it up every frame.

Run this alongside your sim. It auto-detects which sim is running.
Broadcasts on UDP localhost:9977 as JSON at ~60hz.

Usage:
    python telemetry_broadcaster.py

No UI - runs in terminal, Ctrl+C to stop.
"""

import socket
import json
import time
import sys
import threading
from telemetry import TelemetryManager, TelemetryFrame

BROADCAST_PORT = 9977
BROADCAST_HZ   = 60

# Low-pass filter state
_smooth_long_g = 0.0
_smooth_lat_g  = 0.0
_smooth_susp   = 0.0
_SMOOTH        = 0.80        # filter strength — adjust if OBS feels too laggy or too twitchy
_SPIKE_THRESH  = 0.30        # suspension spike bypass threshold

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_latest_payload = json.dumps({
    "long_g": 0.0,
    "lat_g": 0.0,
    "susp": 0.0,
    "speed_kmh": 0.0,
    "sim": ""
}).encode()
_payload_lock = threading.Lock()
_current_sim  = ""


def on_frame(frame: TelemetryFrame):
    global _smooth_long_g, _smooth_lat_g, _smooth_susp, _latest_payload

    alpha = 1.0 - _SMOOTH

    # Suspension: spike bypass
    if frame.susp_bump > _SPIKE_THRESH:
        smooth_susp = frame.susp_bump   # pass spike through raw
    else:
        smooth_susp = alpha * frame.susp_bump + _SMOOTH * _smooth_susp

    smooth_long = alpha * frame.long_g  + _SMOOTH * _smooth_long_g
    smooth_lat  = alpha * frame.lat_g   + _SMOOTH * _smooth_lat_g

    _smooth_long_g = smooth_long
    _smooth_lat_g  = smooth_lat
    _smooth_susp   = smooth_susp

    payload = json.dumps({
        "long_g":    round(smooth_long, 4),
        "lat_g":     round(smooth_lat,  4),
        "susp":      round(smooth_susp, 4),
        "speed_kmh": round(frame.speed_ms * 3.6, 1),
        "sim":       frame.sim_name
    }).encode()

    with _payload_lock:
        _latest_payload = payload


def on_sim_change(sim_name: str):
    global _current_sim
    _current_sim = sim_name
    if sim_name:
        print(f"[+] Sim detected: {sim_name}")
    else:
        print("[!] No sim detected - waiting...")


def broadcast_loop():
    """Sends latest payload at BROADCAST_HZ regardless of telemetry rate"""
    interval = 1.0 / BROADCAST_HZ
    target = ("127.0.0.1", BROADCAST_PORT)

    while True:
        with _payload_lock:
            data = _latest_payload
        sock.sendto(data, target)
        time.sleep(interval)


def _probe_ac_directly():
    """Diagnostic: try to open AC's shared memory directly and report state.
    Helps tell whether 'no sim detected' means AC hasn't created the page,
    or AC created it but isn't ticking."""
    import mmap, struct
    # Need a real length on Windows — use a small power of two large enough
    # to read packetId (first 4 bytes). 4096 = one page, always fits.
    PROBE_SIZE = 4096
    try:
        mm = mmap.mmap(-1, PROBE_SIZE, tagname="Local\\acpmf_physics",
                       access=mmap.ACCESS_READ)
    except FileNotFoundError:
        return ("AC shared memory page does NOT exist "
                "(AC not running, or running with different namespace/elevation)")
    except Exception as e:
        return f"AC shared memory probe failed: {type(e).__name__}: {e}"
    try:
        # First 4 bytes = packetId (int32). Read it twice with a small delay.
        pid1 = struct.unpack("<i", mm[0:4])[0]
        time.sleep(0.2)
        pid2 = struct.unpack("<i", mm[0:4])[0]
        if pid1 == pid2:
            return (f"AC page EXISTS but packetId is frozen at {pid1} "
                    f"(AC is paused, in menu, or frozen)")
        return (f"AC page EXISTS and packetId is ticking ({pid1} -> {pid2}) "
                f"— should be detected")
    finally:
        try: mm.close()
        except Exception: pass


def main():
    print("=" * 50)
    print("  SIM CAM TELEMETRY BROADCASTER")
    print(f"  Broadcasting on UDP localhost:{BROADCAST_PORT}")
    print(f"  Rate: {BROADCAST_HZ}hz")
    print("  Ctrl+C to stop")
    print("=" * 50)
    print()
    print("[*] Starting telemetry manager...")

    manager = TelemetryManager(
        on_frame=on_frame,
        on_sim_change=on_sim_change
    )
    manager.start()

    print("[*] Starting broadcaster...")
    broadcaster = threading.Thread(target=broadcast_loop, daemon=True)
    broadcaster.start()

    print("[*] Waiting for sim...")
    print()

    try:
        last_print = 0
        last_diag  = 0
        while True:
            now = time.time()

            # Every 5 seconds while no sim is detected, run a direct probe
            # so the user knows whether AC's page is even reachable.
            if not _current_sim and now - last_diag > 5.0:
                last_diag = now
                print(f"  [DIAG] {_probe_ac_directly()}")

            # Print live readout every second
            if now - last_print > 1.0:
                last_print = now
                with _payload_lock:
                    try:
                        d = json.loads(_latest_payload)
                        if d["sim"]:
                            print(
                                f"\r  [{d['sim']}]  "
                                f"LongG: {d['long_g']:+.3f}  "
                                f"LatG: {d['lat_g']:+.3f}  "
                                f"Bump: {d['susp']:.3f}  "
                                f"Speed: {d['speed_kmh']:.0f}km/h    ",
                                end="", flush=True
                            )
                    except Exception:
                        pass
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[>] Stopping...")
        manager.stop()
        sock.close()
        print("[OK] Done.")
        sys.exit(0)


if __name__ == "__main__":
    main()
