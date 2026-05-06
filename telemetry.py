"""
telemetry.py - Unified sim telemetry reader
Supports: iRacing, Assetto Corsa, AC Evo, ACC, Le Mans Ultimate / rF2
Auto-detects which sim is running.
"""

import time
import threading
import struct
import mmap
import socket
import ctypes
from ctypes import c_float, c_double, c_int, c_bool, c_char
from dataclasses import dataclass, field
from typing import Optional, Callable


@dataclass
class TelemetryFrame:
    """Normalized telemetry data from any sim"""
    long_g: float = 0.0          # Longitudinal G (braking/accel) — negative = braking
    susp_bump: float = 0.0       # Max suspension bump across 4 wheels (0.0 - 1.0 normalized)
    lat_g: float = 0.0           # Lateral G (cornering) — optional
    speed_ms: float = 0.0        # Speed in m/s
    sim_name: str = ""
    timestamp: float = field(default_factory=time.time)


# ─────────────────────────────────────────────────────────────
# iRacing shared memory structs
# ─────────────────────────────────────────────────────────────
IRSDK_MEM_MAP_FILE = "Local\\IRSDKMemMapFileName"
IRSDK_MEM_MAP_SIZE = 798720

class IRacingReader:
    """Reads iRacing telemetry via pyirsdk"""

    def __init__(self):
        self.ir = None
        self.available = False
        self._try_import()

    def _try_import(self):
        try:
            import irsdk
            self._irsdk = irsdk
            self.available = True
        except ImportError:
            self.available = False

    def connect(self) -> bool:
        if not self.available:
            return False
        try:
            self.ir = self._irsdk.IRSDK()
            return self.ir.startup()
        except Exception:
            return False

    def is_running(self) -> bool:
        if not self.ir:
            return False
        try:
            return self.ir.is_connected
        except Exception:
            return False

    def read(self) -> Optional[TelemetryFrame]:
        if not self.ir or not self.ir.is_connected:
            return None
        try:
            self.ir.freeze_var_buffer_latest()
            long_g = self.ir['LongAccel'] or 0.0      # m/s² positive = accel
            lat_g  = self.ir['LatAccel']  or 0.0
            speed  = self.ir['Speed']     or 0.0

            # Suspension: average of 4 wheel deflections normalized
            susp = [
                self.ir['WheelVertPosFR'] or 0.0,
                self.ir['WheelVertPosFL'] or 0.0,
                self.ir['WheelVertPosRR'] or 0.0,
                self.ir['WheelVertPosRL'] or 0.0,
            ]
            # Normalize to 0-1 assuming max ±0.1m deflection
            susp_norm = min(1.0, max(abs(v) for v in susp) / 0.1)

            return TelemetryFrame(
                long_g=long_g / 9.81,   # convert to G
                lat_g=lat_g / 9.81,
                susp_bump=susp_norm,
                speed_ms=speed,
                sim_name="iRacing"
            )
        except Exception:
            return None

    def disconnect(self):
        if self.ir:
            self.ir.shutdown()


# ─────────────────────────────────────────────────────────────
# Assetto Corsa (original + AC Evo) — shared memory
# ─────────────────────────────────────────────────────────────
AC_PHYSICS_MAP  = "Local\\acpmf_physics"
AC_GRAPHICS_MAP = "Local\\acpmf_graphics"
AC_PHYSICS_SIZE = 65536

# AC session status values
AC_OFF  = 0
AC_REPLAY = 1
AC_LIVE = 2
AC_PAUSE = 3

class ACPhysics(ctypes.Structure):
    # Matches Kunos SPageFilePhysics exactly. _pack_ = 4 is required.
    # Layout taken from sim_info.py / official AC shared-memory docs.
    # NOTE: This struct must be COMPLETE — if the size we tell mmap to map
    # disagrees with what AC's writer expects, the physics thread can trip
    # a bounds check inside SharedMemoryWriter::updatePhysics and crash.
    _pack_ = 4
    _fields_ = [
        ("packetId",          c_int),
        ("gas",               c_float),
        ("brake",             c_float),
        ("fuel",              c_float),
        ("gear",              c_int),
        ("rpms",              c_int),
        ("steerAngle",        c_float),
        ("speedKmh",          c_float),
        ("velocity",          c_float * 3),
        ("accG",              c_float * 3),   # [lat, long, vert]
        ("wheelSlip",         c_float * 4),
        ("wheelLoad",         c_float * 4),
        ("wheelsPressure",    c_float * 4),
        ("wheelAngularSpeed", c_float * 4),
        ("tyreWear",          c_float * 4),
        ("tyreDirtyLevel",    c_float * 4),
        ("tyreCoreTemp",      c_float * 4),
        ("camberRAD",         c_float * 4),
        ("suspensionTravel",  c_float * 4),
        ("drs",               c_float),
        ("tc",                c_float),
        ("heading",           c_float),
        ("pitch",             c_float),
        ("roll",              c_float),
        ("cgHeight",          c_float),
        ("carDamage",         c_float * 5),
        ("numberOfTyresOut",  c_int),
        ("pitLimiterOn",      c_int),
        ("abs",               c_float),
        # ── trailing fields that were missing — adding them fixes the size mismatch
        ("kersCharge",        c_float),
        ("kersInput",         c_float),
        ("autoshifterOn",     c_int),
        ("rideHeight",        c_float * 2),
        ("turboBoost",        c_float),
        ("ballast",           c_float),
        ("airDensity",        c_float),
        ("airTemp",           c_float),
        ("roadTemp",          c_float),
        ("localAngularVel",   c_float * 3),
        ("finalFF",           c_float),
        ("performanceMeter",  c_float),
        ("engineBrake",       c_int),
        ("ersRecoveryLevel",  c_int),
        ("ersPowerLevel",     c_int),
        ("ersHeatCharging",   c_int),
        ("ersIsCharging",     c_int),
        ("kersCurrentKJ",     c_float),
        ("drsAvailable",      c_int),
        ("drsEnabled",        c_int),
        ("brakeTemp",         c_float * 4),
        ("clutch",            c_float),
        ("tyreTempI",         c_float * 4),
        ("tyreTempM",         c_float * 4),
        ("tyreTempO",         c_float * 4),
        ("isAIControlled",    c_int),
        ("tyreContactPoint",  (c_float * 3) * 4),
        ("tyreContactNormal", (c_float * 3) * 4),
        ("tyreContactHeading",(c_float * 3) * 4),
        ("brakeBias",         c_float),
        ("localVelocity",     c_float * 3),
    ]

class ACReader:
    """Reads Assetto Corsa / AC Evo shared memory.
    Uses the same mmap pattern as sim_info.py — the official AC community standard.
    Safe to run before AC launches. Returns False on connect() until AC is running.
    """

    def __init__(self):
        self.mm = None
        self.available = False
        self._last_packet_id = -1
        self._stale_count = 0
        self._prev_susp = None

    def connect(self) -> bool:
        try:
            import mmap as _mmap
            # On Windows with fileno=-1 (anonymous), length=0 raises WinError 87.
            # We must pass the actual struct size. AC's page is at least this big;
            # asking for exactly sizeof(ACPhysics) is safe as long as our struct
            # matches AC's layout (which the completed _fields_ now does).
            self.mm = _mmap.mmap(-1, ctypes.sizeof(ACPhysics),
                                 tagname=AC_PHYSICS_MAP,
                                 access=_mmap.ACCESS_READ)
            # Reset liveness tracking on each fresh connect
            self._last_packet_id = -1
            self._stale_count = 0
            self.available = True
            return True
        except (FileNotFoundError, OSError, ValueError):
            # AC hasn't created the mapping yet — totally safe, just not running
            self.mm = None
            return False
        except Exception:
            self.available = False
            self.mm = None
            return False

    def _snapshot(self) -> Optional[ACPhysics]:
        """Take a single immutable copy of the shared memory.
        We never seek() or hold a slice — both can race AC's writer."""
        if not self.mm:
            return None
        try:
            size = ctypes.sizeof(ACPhysics)
            # mmap[start:stop] returns bytes — a fresh copy, safe to parse
            buf = self.mm[0:size]
            if len(buf) < size:
                return None
            return ACPhysics.from_buffer_copy(buf)
        except Exception:
            return None

    def is_running(self) -> bool:
        """AC is 'running' if packetId is advancing. The page can exist with
        stale data if AC was launched and quit, so checking != 0 isn't enough.
        We check that packetId has changed since last call."""
        data = self._snapshot()
        if not data:
            return False

        pid = data.packetId

        # First call after connect — accept any non-zero packetId,
        # or zero if we see physics activity (rpms or speed).
        if self._last_packet_id < 0:
            self._last_packet_id = pid
            if pid > 0 or data.rpms > 0 or data.speedKmh > 0.01:
                return True
            return False

        # Subsequent calls — packetId must advance for AC to be live.
        if pid != self._last_packet_id:
            self._last_packet_id = pid
            self._stale_count = 0
            return True

        # Same packetId as last time — AC is paused, in menu, or quit.
        # Allow a few stale reads before declaring dead (paused sessions
        # may briefly hold packetId steady).
        self._stale_count += 1
        return self._stale_count < 30   # ~1 sec at 30Hz

    def read(self) -> Optional[TelemetryFrame]:
        data = self._snapshot()
        if not data:
            return None
        try:
            # accG: [lat, long, vert]
            lat_g  = data.accG[0]
            long_g = data.accG[1]

            # Suspension BUMP — the original code used absolute suspensionTravel,
            # but that's a static value (~0.1m even when parked, because the car
            # sits on its springs). What we actually want is the RATE OF CHANGE
            # of suspension travel — that's what spikes during bumps & kerbs.
            #
            # We track previous values per-wheel and report the max abs velocity.
            current = list(data.suspensionTravel)
            if self._prev_susp is None:
                deltas = [0.0, 0.0, 0.0, 0.0]
            else:
                deltas = [c - p for c, p in zip(current, self._prev_susp)]
            self._prev_susp = current

            # Max wheel travel velocity (in meters per tick). A real bump
            # produces deltas of ~0.005-0.02m per 30Hz tick. Normalize to 0..1
            # using 0.015 as full-scale (sharp kerb hit).
            max_delta = max(abs(d) for d in deltas)
            susp_bump = min(1.0, max_delta / 0.015)

            return TelemetryFrame(
                long_g=long_g,
                lat_g=lat_g,
                susp_bump=susp_bump,
                speed_ms=data.speedKmh / 3.6,
                sim_name="Assetto Corsa"
            )
        except Exception:
            return None

    def disconnect(self):
        if self.mm:
            try:
                self.mm.close()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────
# ACC (Assetto Corsa Competizione) — UDP
# ─────────────────────────────────────────────────────────────
ACC_UDP_PORT = 9996  # AC broadcasts TO this port — we receive only
ACC_PACKET_SIZE = 328

class ACCReader:
    """Reads ACC telemetry via UDP broadcast.
    Only binds port 9996 when ACC.exe is actually running.
    Never touches the port otherwise — avoids conflicting with AC.
    """

    def __init__(self):
        self.sock = None
        self.latest: Optional[TelemetryFrame] = None
        self._thread = None
        self._running = False
        self.available = False

    def _acc_process_running(self) -> bool:
        """Check if ACC is actually running before touching port 9996"""
        try:
            import subprocess
            result = subprocess.run(
                ['tasklist', '/FI', 'IMAGENAME eq AC2-Win64-Shipping.exe', '/NH'],
                capture_output=True, text=True, timeout=2
            )
            return 'AC2-Win64-Shipping.exe' in result.stdout
        except Exception:
            return False

    def connect(self) -> bool:
        try:
            # ONLY bind 9996 if ACC is actually running
            if not self._acc_process_running():
                return False
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("", ACC_UDP_PORT))
            self.sock.settimeout(1.0)
            self._running = True
            self._thread = threading.Thread(target=self._listen, daemon=True)
            self._thread.start()
            self.available = True
            return True
        except Exception:
            return False

    def _listen(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(4096)
                frame = self._parse(data)
                if frame:
                    self.latest = frame
            except socket.timeout:
                continue
            except Exception:
                break

    def _parse(self, data: bytes) -> Optional[TelemetryFrame]:
        """
        ACC UDP physics packet — partial struct
        Full layout: https://www.assettocorsa.net/forum/index.php?threads/acc-udp-remote-telemetry-port.59394/
        """
        try:
            if len(data) < 100:
                return None
            # Offsets for key fields in ACC UDP physics packet
            # speedKmh at offset 4 (float)
            # accG at offset 8: lat, long, vert (3 floats = 12 bytes)
            # suspensionTravel at offset 68: 4 floats
            speed_kmh, = struct.unpack_from('<f', data, 4)
            lat_g, long_g, _ = struct.unpack_from('<fff', data, 8)
            susp = struct.unpack_from('<ffff', data, 68)
            susp_norm = min(1.0, max(abs(v) for v in susp) / 0.15)

            return TelemetryFrame(
                long_g=long_g,
                lat_g=lat_g,
                susp_bump=susp_norm,
                speed_ms=speed_kmh / 3.6,
                sim_name="ACC"
            )
        except Exception:
            return None

    def is_running(self) -> bool:
        if not self.latest:
            return False
        return (time.time() - self.latest.timestamp) < 2.0

    def read(self) -> Optional[TelemetryFrame]:
        return self.latest

    def disconnect(self):
        self._running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────
# rFactor2 / Le Mans Ultimate — shared memory plugin
# ─────────────────────────────────────────────────────────────
RF2_MEM_MAP = "$rFactor2SMMP_Telemetry$"
RF2_MAGIC = 0x4D534D52   # 'RMSM'

class RF2Reader:
    """Reads rFactor2 / LMU shared memory (requires plugin installed)"""

    def __init__(self):
        self.mm = None
        self.available = False

    def connect(self) -> bool:
        try:
            import mmap as _mmap
            # rF2 plugin writes ~730KB block
            self.mm = _mmap.mmap(-1, 730 * 1024,
                                  tagname=RF2_MEM_MAP,
                                  access=_mmap.ACCESS_READ)
            self.available = True
            return True
        except Exception:
            self.available = False
            return False

    def is_running(self) -> bool:
        if not self.mm:
            return False
        try:
            self.mm.seek(0)
            magic = struct.unpack_from('<I', self.mm.read(4))[0]
            return magic == RF2_MAGIC
        except Exception:
            return False

    def read(self) -> Optional[TelemetryFrame]:
        if not self.mm:
            return None
        try:
            self.mm.seek(0)
            raw = self.mm.read(512)
            # rF2 vehicle telemetry block offsets (from plugin SDK docs)
            # mLocalAcceleration: offset 72, 3 doubles (x=lat, y=vert, z=long)
            lat_g, vert_g, long_g = struct.unpack_from('<ddd', raw, 72)
            # mSpeed: offset 48, double
            speed, = struct.unpack_from('<d', raw, 48)
            # Suspension deflection at offset 264, 4 doubles
            susp = struct.unpack_from('<dddd', raw, 264)
            susp_norm = min(1.0, max(abs(v) for v in susp) / 0.15)

            return TelemetryFrame(
                long_g=long_g / 9.81,
                lat_g=lat_g / 9.81,
                susp_bump=susp_norm,
                speed_ms=speed,
                sim_name="LMU/rF2"
            )
        except Exception:
            return None

    def disconnect(self):
        if self.mm:
            try:
                self.mm.close()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────
# Unified auto-detecting telemetry manager
# ─────────────────────────────────────────────────────────────
class TelemetryManager:
    """
    Auto-detects running sim and feeds normalized TelemetryFrames
    to a callback at ~30hz.
    """

    POLL_HZ = 30
    DETECT_INTERVAL = 3.0   # How often to re-check which sim is running

    def __init__(self, on_frame: Callable[[TelemetryFrame], None],
                       on_sim_change: Callable[[str], None] = None):
        self.on_frame = on_frame
        self.on_sim_change = on_sim_change

        self.readers = {
            "iRacing":        IRacingReader(),
            "Assetto Corsa":  ACReader(),
            "ACC":            ACCReader(),
            "LMU/rF2":        RF2Reader(),
        }

        self.active_reader = None
        self.active_sim = ""
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        for reader in self.readers.values():
            try:
                reader.disconnect()
            except Exception:
                pass

    def _detect_sim(self):
        """Try to connect to each sim, return first one that responds.
        IMPORTANT: disconnect any reader whose connect() succeeded but
        is_running() failed — otherwise we leak handles each cycle."""
        for name, reader in self.readers.items():
            try:
                if reader.connect():
                    if reader.is_running():
                        return name, reader
                    # Connected but sim isn't actually live — release handle
                    reader.disconnect()
            except Exception:
                try:
                    reader.disconnect()
                except Exception:
                    pass
                continue
        return None, None

    def _loop(self):
        last_detect = 0.0
        interval = 1.0 / self.POLL_HZ
        # First detection should happen fast — wait only 0.5s after start,
        # not the full 3s DETECT_INTERVAL.
        first_detect = True

        while self._running:
            now = time.time()

            wait = 0.5 if first_detect else self.DETECT_INTERVAL

            # Periodically try to detect / re-detect sim
            if now - last_detect > wait:
                last_detect = now
                first_detect = False

                if not self.active_reader or not self.active_reader.is_running():
                    # Drop a dead active reader before scanning again
                    if self.active_reader:
                        try:
                            self.active_reader.disconnect()
                        except Exception:
                            pass
                        self.active_reader = None

                    sim_name, reader = self._detect_sim()

                    if sim_name and sim_name != self.active_sim:
                        self.active_reader = reader
                        self.active_sim = sim_name
                        if self.on_sim_change:
                            self.on_sim_change(sim_name)
                    elif not sim_name:
                        if self.active_sim:
                            self.active_sim = ""
                            self.active_reader = None
                            if self.on_sim_change:
                                self.on_sim_change("")

            # Read frame from active sim
            if self.active_reader:
                frame = self.active_reader.read()
                if frame:
                    self.on_frame(frame)

            time.sleep(interval)
