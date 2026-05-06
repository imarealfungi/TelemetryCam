# sim-cam — OBS Passenger Camera Shake from Sim Telemetry

Adds realistic camera shake to an OBS source driven by live sim telemetry — longitudinal G (braking/acceleration), lateral G (cornering), and suspension bumps. Looks like a backseat or helmet cam with real physics behind it.

**Supported sims:** Assetto Corsa, AC Evo, ACC, iRacing, Le Mans Ultimate / rFactor 2

---

## How it works

Three pieces run together:

```
telemetry.py             — reads shared memory / UDP from whichever sim is running
telemetry_broadcaster.py — rebroadcasts normalized data over UDP localhost:9977 at 60hz
obs_sim_cam.py           — OBS Python script; listens to that UDP and nudges your source
```

The broadcaster auto-detects which sim is running. The OBS script never writes an absolute position — it only applies small per-frame deltas, so your source stays wherever you placed it and returns to rest when telemetry goes idle.

---

## Setup

### 1. Install Python dependencies

```bash
pip install pyirsdk        # iRacing only — skip if you don't use it
```

No extra dependencies needed for AC, AC Evo, ACC, LMU, or rF2 — they use shared memory or UDP that's built into Windows.

### 2. Add the OBS script

- In OBS: **Tools → Scripts → + → obs_sim_cam.py**
- Pick your camera source from the dropdown
- Adjust shake intensity sliders to taste

### 3. Avoid black bars

Before a session, zoom your camera source ~5–10% larger than the canvas edge in OBS preview (Alt-drag a corner). The shake happens *within* the visible frame so you never see the edges.

### 4. Run the broadcaster before your session

```bash
python telemetry_broadcaster.py
```

Leave it running in a terminal. It will print a live readout once a sim is detected:

```
[+] Sim detected: Assetto Corsa
  [Assetto Corsa]  LongG: -0.312  LatG: +0.841  Bump: 0.043  Speed: 187km/h
```

---

## OBS script settings

| Setting | What it does |
|---|---|
| Camera Source | The OBS source to shake |
| Max Vertical Shift | Peak vertical displacement in pixels |
| Max Horizontal Shift | Peak horizontal displacement in pixels |
| Max Rotation | Peak rotation in degrees |
| Kerb Spike Boost | Multiplier for suspension spikes (kerbs, bumps) |
| Return Speed | How quickly the camera eases back to rest (lower = floatier) |
| Speed Zoom Amount | Subtle zoom-in at speed (0 = off) |
| Zoom Peak Speed | km/h at which peak zoom is reached |
| Auto-Hide Cam When Sim Idle | Fades the source out when the car is stopped or sim closes |
| Hide After (seconds idle) | How long to wait before hiding |
| Fade Duration | How long the fade in/out takes |

---

## Sim-specific notes

**Assetto Corsa / AC Evo**
Reads `acpmf_physics` shared memory directly. AC must be running and in an active session (not the main menu) for the page to be live.

**ACC**
Reads via ACC's built-in UDP telemetry. Enable it in ACC: *Settings → Gameplay → Broadcasting → Enable broadcasting → Port 9600*.

**iRacing**
Requires `pyirsdk` (`pip install pyirsdk`). iRacing must be running with a car on track.

**Le Mans Ultimate / rFactor 2**
Requires the rF2 Shared Memory plugin installed in the sim's plugin folder. Available from the rF2 forum.

---

## File overview

```
obs_sim_cam.py           OBS Python script — load this in OBS Tools > Scripts
telemetry_broadcaster.py Run this in a terminal before your session
telemetry.py             Telemetry reader library (used by broadcaster)
```

---

## Troubleshooting

**"No sim detected" in the broadcaster terminal**
The broadcaster runs a diagnostic every 5 seconds telling you whether the sim's shared memory page exists and whether it's ticking. Check that output — it will tell you if AC's page exists but is frozen (usually means you're in the menu, not on track).

**Camera snaps or jumps when the script loads/unloads**
The script applies an equal-and-opposite delta on unload to return the source to exactly where you had it. If it ever gets out of sync, just unload and reload the script.

**Black bars during shake**
Zoom your source bigger than the canvas. Alt-drag a corner in OBS preview until the edges are outside the frame.

**ACC not detected**
Check that broadcasting is enabled in ACC settings and that port 9600 is free.

---

## Contributing

Works as-is for the sims listed. Pull requests welcome for additional sim support — the `TelemetryFrame` dataclass in `telemetry.py` is the only interface the broadcaster needs: `long_g`, `lat_g`, `susp_bump`, `speed_ms`, `sim_name`.
