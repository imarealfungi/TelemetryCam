"""
obs_sim_cam.py — OBS Python Script
Passenger/backseat camera shake driven by sim racing telemetry.

DESIGN:
  This script never writes an absolute position. Each tick it reads
  where the source CURRENTLY is and nudges it by a small delta. That
  means:
    - Your source stays exactly where you put it.
    - There is no "base" coordinate to capture wrong, no alignment
      assumption, no lock-point — wherever you drag it is its rest.
    - When telemetry goes idle, accumulated nudges undo themselves
      and the source settles back to where it started.

  Listens for UDP telemetry from telemetry_broadcaster.py on port 9977.

NO BLACK BARS:
  Manually zoom your source in OBS ~5-10% past the canvas edge so the
  shake happens WITHIN the visible frame (Alt-drag a corner in preview).
"""

import obspython as obs
import socket
import json
import threading

UDP_PORT     = 9977
MAX_Y_SHIFT  = 18      # max vertical shake (px)
MAX_X_SHIFT  = 8       # max horizontal shake (px)
MAX_ROTATION = 2.5     # max rotation (deg)
SPIKE_BOOST  = 1.8     # how much suspension bumps amplify vertical jolt
RETURN_SPEED = 0.12    # smoothing — how quickly we ease toward target (0..1)

# Speed-based zoom — multiplier applied to scale, scales from 0 to ZOOM_FULL_SPEED
ZOOM_AMOUNT      = 0.05    # peak zoom-in (0.05 = 5% larger at full speed)
ZOOM_FULL_SPEED  = 300.0   # km/h at which peak zoom is reached
ZOOM_RETURN      = 0.08    # smoothing for zoom (slower than shake — feels cinematic)

# Auto-hide — fade cam out when sim isn't live, in when it is
AUTO_HIDE_ENABLED = False     # off by default; user opts in via checkbox
HIDE_TIMEOUT_SEC  = 3.0       # seconds of stale telemetry before hiding
FADE_DURATION_SEC = 1.5       # how long the fade in/out takes
FADE_FILTER_NAME  = "SimCam Auto-Hide"   # name we give the auto-created filter

LAT_G_FULL_SCALE  = 1.5
LONG_G_FULL_SCALE = 1.5

DEBUG_LOG = True

# ── runtime state ────────────────────────────────────────────────────
_long_g = _lat_g = _susp = _speed_kmh = 0.0
_sim_name = ""

# Cumulative shake offset relative to user's rest position
_cur_x = _cur_y = _cur_rot = 0.0
_susp_impulse = 0.0
_cur_zoom = 0.0          # current smoothed zoom factor (0 = no zoom)

# Previous frame's total displacement — used to compute per-frame delta
_prev_total_x = 0.0
_prev_total_y = 0.0
_prev_total_rot = 0.0
_prev_zoom_mult = 1.0    # previous frame's scale multiplier

# Auto-hide tracking
import time as _time
_last_change_time = 0.0      # last time telemetry values actually changed
_prev_telemetry = (0.0, 0.0, 0.0, 0.0, "")   # snapshot for change detection
_target_opacity = 1.0        # 0 or 1 (where we want to be)
_cur_opacity    = 1.0        # current opacity (smoothed)
_applied_opacity = -1.0      # what we last wrote (dirty check)

_sock = None
_running = False
_thread = None
_source_name = ""

_cached_sceneitem = None
_retry_counter = 0
_debug_counter = 0


def _listen():
    global _long_g, _lat_g, _susp, _speed_kmh, _running, _sock
    global _sim_name
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(1.0)
    try:
        s.bind(("", UDP_PORT))
    except Exception as e:
        obs.script_log(obs.LOG_WARNING, f"[SimCam] Bind failed on {UDP_PORT}: {e}")
        return
    _sock = s
    obs.script_log(obs.LOG_INFO, "[SimCam] Listening for telemetry...")
    while _running:
        try:
            data, _ = s.recvfrom(1024)
            d = json.loads(data.decode())
            _long_g    = d.get("long_g", 0.0)
            _lat_g     = d.get("lat_g",  0.0)
            _susp      = d.get("susp",   0.0)
            _speed_kmh = d.get("speed_kmh", 0.0)
            _sim_name  = d.get("sim", "")
        except socket.timeout:
            _long_g    *= 0.95
            _lat_g     *= 0.95
            _susp      *= 0.95
            _speed_kmh *= 0.95
        except Exception:
            pass
    try:
        s.close()
    except Exception:
        pass


def _refresh_scene_item():
    """Just find the scene item handle. We do NOT capture any position —
    the source stays wherever the user has it."""
    global _cached_sceneitem

    _cached_sceneitem = None

    if not _source_name:
        return

    src = obs.obs_frontend_get_current_scene()
    if src is None:
        return
    try:
        scene = obs.obs_scene_from_source(src)
        if scene is None:
            return
        item = obs.obs_scene_find_source(scene, _source_name)
        if item is None:
            obs.script_log(obs.LOG_WARNING,
                f"[SimCam] Source '{_source_name}' not in current scene")
            return
        _cached_sceneitem = item
        obs.script_log(obs.LOG_INFO, f"[SimCam] Locked on to '{_source_name}'")
    finally:
        obs.obs_source_release(src)


def _ensure_fade_filter():
    """Create the Color Correction filter on the source if it doesn't
    exist yet. We modulate ITS opacity to fade — never the source's own
    settings, never the sceneitem's visibility flag (that would jank
    the shake)."""
    if _cached_sceneitem is None:
        return None
    src = obs.obs_sceneitem_get_source(_cached_sceneitem)
    if src is None:
        return None
    f = obs.obs_source_get_filter_by_name(src, FADE_FILTER_NAME)
    if f is not None:
        return f
    # Create it
    settings = obs.obs_data_create()
    obs.obs_data_set_double(settings, "opacity", 1.0)
    f = obs.obs_source_create_private("color_filter", FADE_FILTER_NAME, settings)
    obs.obs_data_release(settings)
    if f is not None:
        obs.obs_source_filter_add(src, f)
        obs.script_log(obs.LOG_INFO, f"[SimCam] Added fade filter to '{_source_name}'")
    return f


def _set_fade_opacity(opacity):
    """Update the fade filter's opacity."""
    if _cached_sceneitem is None:
        return
    src = obs.obs_sceneitem_get_source(_cached_sceneitem)
    if src is None:
        return
    f = obs.obs_source_get_filter_by_name(src, FADE_FILTER_NAME)
    if f is None:
        return
    settings = obs.obs_data_create()
    obs.obs_data_set_double(settings, "opacity", max(0.0, min(1.0, opacity)))
    obs.obs_source_update(f, settings)
    obs.obs_data_release(settings)
    obs.obs_source_release(f)


def _remove_fade_filter():
    """Strip the fade filter off the source entirely. Used when disabling
    auto-hide — guarantees nothing left over can hold the cam invisible."""
    if _cached_sceneitem is None:
        return
    src = obs.obs_sceneitem_get_source(_cached_sceneitem)
    if src is None:
        return
    f = obs.obs_source_get_filter_by_name(src, FADE_FILTER_NAME)
    if f is None:
        return
    obs.obs_source_filter_remove(src, f)
    obs.obs_source_release(f)
    obs.script_log(obs.LOG_INFO, "[SimCam] Removed fade filter")


def _tick():
    global _cur_x, _cur_y, _cur_rot, _susp_impulse, _cur_zoom
    global _prev_total_x, _prev_total_y, _prev_total_rot, _prev_zoom_mult
    global _retry_counter, _debug_counter
    global _last_change_time, _prev_telemetry
    global _target_opacity, _cur_opacity, _applied_opacity

    if _cached_sceneitem is None:
        _retry_counter += 1
        if _retry_counter >= 30:
            _retry_counter = 0
            if _source_name:
                _refresh_scene_item()
        return
    _retry_counter = 0

    # ── Compute target shake offsets ────────────────────────────────
    long_norm = max(-1.0, min(1.0, _long_g / LONG_G_FULL_SCALE))
    target_y  = -long_norm * MAX_Y_SHIFT       # brake = nose dive = camera up

    lat_norm   = max(-1.0, min(1.0, _lat_g / LAT_G_FULL_SCALE))
    target_rot = lat_norm * MAX_ROTATION
    target_x   = lat_norm * MAX_X_SHIFT

    # Suspension impulse — decays back to 0, can't accumulate as drift
    _susp_impulse *= 0.85
    bump = _susp * MAX_Y_SHIFT * SPIKE_BOOST * 0.5
    if bump > _susp_impulse:
        _susp_impulse = bump

    # Speed-based zoom — scales smoothly from 0 to ZOOM_FULL_SPEED
    speed_norm  = max(0.0, min(1.0, _speed_kmh / ZOOM_FULL_SPEED))
    target_zoom = speed_norm * ZOOM_AMOUNT

    # Smooth toward targets
    _cur_x    += (target_x    - _cur_x)    * RETURN_SPEED
    _cur_y    += (target_y    - _cur_y)    * RETURN_SPEED
    _cur_rot  += (target_rot  - _cur_rot)  * RETURN_SPEED
    _cur_zoom += (target_zoom - _cur_zoom) * ZOOM_RETURN

    # Total desired displacement THIS frame relative to user's rest position
    total_x   = _cur_x
    total_y   = _cur_y - _susp_impulse           # impulse pushes UP (-y)
    total_rot = _cur_rot

    # Delta from last frame — this is what we actually apply
    dx   = total_x   - _prev_total_x
    dy   = total_y   - _prev_total_y
    drot = total_rot - _prev_total_rot

    _prev_total_x   = total_x
    _prev_total_y   = total_y
    _prev_total_rot = total_rot

    move = abs(dx) >= 0.05 or abs(dy) >= 0.05
    rot  = abs(drot) >= 0.005

    if move:
        pos = obs.vec2()
        obs.obs_sceneitem_get_pos(_cached_sceneitem, pos)
        new_pos = obs.vec2()
        new_pos.x = pos.x + dx
        new_pos.y = pos.y + dy
        obs.obs_sceneitem_set_pos(_cached_sceneitem, new_pos)

    if rot:
        cur_rot = obs.obs_sceneitem_get_rot(_cached_sceneitem)
        obs.obs_sceneitem_set_rot(_cached_sceneitem, cur_rot + drot)

    # ── Apply zoom as a scale multiplier delta ──────────────────────
    # Same delta-only approach: read current scale, multiply by ratio to
    # target zoom, write back. This means the user's manual scale stays
    # the baseline — we just nudge it up/down by the speed factor.
    target_mult = 1.0 + _cur_zoom
    if abs(target_mult - _prev_zoom_mult) >= 0.0005:
        ratio = target_mult / _prev_zoom_mult
        scale = obs.vec2()
        obs.obs_sceneitem_get_scale(_cached_sceneitem, scale)
        new_scale = obs.vec2()
        new_scale.x = scale.x * ratio
        new_scale.y = scale.y * ratio
        obs.obs_sceneitem_set_scale(_cached_sceneitem, new_scale)
        _prev_zoom_mult = target_mult

    # ── Auto-hide / fade ────────────────────────────────────────────
    if AUTO_HIDE_ENABLED:
        now = _time.time()
        # "Live" = car is moving. Engine vibration in pits causes constant
        # micro-fluctuation in G/susp values, so we can't use "values
        # changing" as the signal. Speed is the clean indicator:
        #   - speed > 1 km/h     → driving (or rolling) → reset timer
        #   - speed ~0 for 3s+   → parked / in menu / AC closed → hide
        # Also hide immediately if we lose the sim entirely.
        if _sim_name and _speed_kmh > 1.0:
            _last_change_time = now

        live = bool(_sim_name) and (now - _last_change_time) < HIDE_TIMEOUT_SEC
        _target_opacity = 1.0 if live else 0.0

        # Ease toward target — fade duration determines per-tick step
        # Tick is 33ms, so steps_in_fade = FADE_DURATION_SEC / 0.033
        steps = max(1.0, FADE_DURATION_SEC / 0.033)
        step  = 1.0 / steps
        if _cur_opacity < _target_opacity:
            _cur_opacity = min(_target_opacity, _cur_opacity + step)
        elif _cur_opacity > _target_opacity:
            _cur_opacity = max(_target_opacity, _cur_opacity - step)

        if abs(_cur_opacity - _applied_opacity) >= 0.005:
            _set_fade_opacity(_cur_opacity)
            _applied_opacity = _cur_opacity
    else:
        # Feature disabled — make sure source is fully visible
        if _applied_opacity != 1.0 and _applied_opacity != -1.0:
            _set_fade_opacity(1.0)
            _cur_opacity = 1.0
            _applied_opacity = 1.0

    # Debug log once a second
    _debug_counter += 1
    if DEBUG_LOG and _debug_counter >= 30:
        _debug_counter = 0
        obs.script_log(obs.LOG_INFO,
            f"[SimCam] G(long={_long_g:+.2f} lat={_lat_g:+.2f} susp={_susp:.2f}) "
            f"spd={_speed_kmh:.0f} shake=({_cur_x:+.1f},{_cur_y:+.1f}) "
            f"zoom={_cur_zoom*100:+.1f}%")


def script_description():
    return ("<b>Sim Racing Passenger Cam</b><br>"
            "Camera shake from sim telemetry. The source stays wherever "
            "you place it — the script only nudges by tiny per-frame "
            "deltas, so move your camera anywhere and it stays put.<br><br>"
            "To avoid black bars, zoom your camera ~10% larger than the "
            "canvas first (Alt-drag a corner in OBS preview).<br><br>"
            "Run telemetry_broadcaster.py before your session.")


def script_properties():
    props = obs.obs_properties_create()

    source_list = obs.obs_properties_add_list(
        props, "source_name", "Camera Source",
        obs.OBS_COMBO_TYPE_EDITABLE, obs.OBS_COMBO_FORMAT_STRING
    )
    sources = obs.obs_enum_sources()
    if sources:
        for s in sources:
            name = obs.obs_source_get_name(s)
            obs.obs_property_list_add_string(source_list, name, name)
        obs.source_list_release(sources)

    obs.obs_properties_add_int_slider  (props, "max_y_shift",  "Max Vertical Shift (px)",   0,    60,  1)
    obs.obs_properties_add_int_slider  (props, "max_x_shift",  "Max Horizontal Shift (px)", 0,    30,  1)
    obs.obs_properties_add_float_slider(props, "max_rotation", "Max Rotation (deg)",        0.0,  8.0, 0.1)
    obs.obs_properties_add_float_slider(props, "spike_boost",  "Kerb Spike Boost",          0.5,  4.0, 0.1)
    obs.obs_properties_add_float_slider(props, "return_speed", "Return Speed",              0.01, 0.5, 0.01)
    obs.obs_properties_add_float_slider(props, "zoom_amount",  "Speed Zoom Amount (0=off)", 0.0,  0.15, 0.005)
    obs.obs_properties_add_int_slider  (props, "zoom_full",    "Zoom Peak Speed (km/h)",    100,  400, 10)

    obs.obs_properties_add_bool        (props, "auto_hide",     "Auto-Hide Cam When Sim Idle")
    obs.obs_properties_add_float_slider(props, "hide_timeout",  "Hide After (seconds idle)", 1.0, 30.0, 0.5)
    obs.obs_properties_add_float_slider(props, "fade_duration", "Fade Duration (seconds)",   0.1, 5.0,  0.1)

    return props


def script_defaults(settings):
    obs.obs_data_set_default_string(settings, "source_name",  "")
    obs.obs_data_set_default_int   (settings, "max_y_shift",  MAX_Y_SHIFT)
    obs.obs_data_set_default_int   (settings, "max_x_shift",  MAX_X_SHIFT)
    obs.obs_data_set_default_double(settings, "max_rotation", MAX_ROTATION)
    obs.obs_data_set_default_double(settings, "spike_boost",  SPIKE_BOOST)
    obs.obs_data_set_default_double(settings, "return_speed", RETURN_SPEED)
    obs.obs_data_set_default_double(settings, "zoom_amount",  ZOOM_AMOUNT)
    obs.obs_data_set_default_int   (settings, "zoom_full",    int(ZOOM_FULL_SPEED))
    obs.obs_data_set_default_bool  (settings, "auto_hide",     AUTO_HIDE_ENABLED)
    obs.obs_data_set_default_double(settings, "hide_timeout",  HIDE_TIMEOUT_SEC)
    obs.obs_data_set_default_double(settings, "fade_duration", FADE_DURATION_SEC)


def script_update(settings):
    global _source_name, MAX_Y_SHIFT, MAX_X_SHIFT, MAX_ROTATION
    global SPIKE_BOOST, RETURN_SPEED, ZOOM_AMOUNT, ZOOM_FULL_SPEED
    global AUTO_HIDE_ENABLED, HIDE_TIMEOUT_SEC, FADE_DURATION_SEC

    new_name = obs.obs_data_get_string(settings, "source_name")
    name_changed = (new_name != _source_name)
    _source_name = new_name

    MAX_Y_SHIFT     = obs.obs_data_get_int   (settings, "max_y_shift")
    MAX_X_SHIFT     = obs.obs_data_get_int   (settings, "max_x_shift")
    MAX_ROTATION    = obs.obs_data_get_double(settings, "max_rotation")
    SPIKE_BOOST     = obs.obs_data_get_double(settings, "spike_boost")
    RETURN_SPEED    = obs.obs_data_get_double(settings, "return_speed")
    ZOOM_AMOUNT     = obs.obs_data_get_double(settings, "zoom_amount")
    ZOOM_FULL_SPEED = float(obs.obs_data_get_int(settings, "zoom_full"))

    new_auto = obs.obs_data_get_bool(settings, "auto_hide")
    auto_changed = (new_auto != AUTO_HIDE_ENABLED)
    AUTO_HIDE_ENABLED  = new_auto
    HIDE_TIMEOUT_SEC   = obs.obs_data_get_double(settings, "hide_timeout")
    FADE_DURATION_SEC  = obs.obs_data_get_double(settings, "fade_duration")

    if name_changed:
        _refresh_scene_item()

    # Auto-hide just got toggled — make sure the filter is set up
    # correctly. When turning ON, ensure the filter exists. When turning
    # OFF, REMOVE the filter entirely — guarantees the cam comes back
    # fully visible with no leftover opacity state.
    if auto_changed:
        global _cur_opacity, _applied_opacity, _target_opacity
        if AUTO_HIDE_ENABLED:
            _ensure_fade_filter()
            _cur_opacity = 1.0
            _target_opacity = 1.0
            _applied_opacity = 1.0
        else:
            _set_fade_opacity(1.0)        # belt
            _remove_fade_filter()         # and suspenders
            _cur_opacity = 1.0
            _target_opacity = 1.0
            _applied_opacity = 1.0


def script_load(settings):
    global _running, _thread
    script_update(settings)
    _running = True
    _thread  = threading.Thread(target=_listen, daemon=True)
    _thread.start()
    obs.timer_add(_tick, 33)   # ~30Hz
    _refresh_scene_item()
    obs.script_log(obs.LOG_INFO, "[SimCam] Loaded")


def script_unload():
    """Undo our accumulated shake by applying an equal-and-opposite delta,
    so we leave the source exactly where the user had it."""
    global _running, _cached_sceneitem
    obs.timer_remove(_tick)
    _running = False

    if _cached_sceneitem is not None:
        try:
            undo_x = -_prev_total_x
            undo_y = -_prev_total_y
            if abs(undo_x) > 0.05 or abs(undo_y) > 0.05:
                pos = obs.vec2()
                obs.obs_sceneitem_get_pos(_cached_sceneitem, pos)
                new_pos = obs.vec2()
                new_pos.x = pos.x + undo_x
                new_pos.y = pos.y + undo_y
                obs.obs_sceneitem_set_pos(_cached_sceneitem, new_pos)

            if abs(_prev_total_rot) > 0.005:
                cur_rot = obs.obs_sceneitem_get_rot(_cached_sceneitem)
                obs.obs_sceneitem_set_rot(_cached_sceneitem, cur_rot - _prev_total_rot)

            # Undo accumulated zoom — divide out the multiplier we applied
            if abs(_prev_zoom_mult - 1.0) > 0.0005:
                scale = obs.vec2()
                obs.obs_sceneitem_get_scale(_cached_sceneitem, scale)
                new_scale = obs.vec2()
                new_scale.x = scale.x / _prev_zoom_mult
                new_scale.y = scale.y / _prev_zoom_mult
                obs.obs_sceneitem_set_scale(_cached_sceneitem, new_scale)

            # Restore opacity to 1.0 and strip the filter so we don't
            # leave anything behind that could hide the cam later.
            _set_fade_opacity(1.0)
            _remove_fade_filter()
        except Exception:
            pass

    _cached_sceneitem = None
    if _sock is not None:
        try:
            _sock.close()
        except Exception:
            pass
    obs.script_log(obs.LOG_INFO, "[SimCam] Unloaded")
