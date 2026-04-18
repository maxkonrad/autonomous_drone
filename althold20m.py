#!/usr/bin/env python3
"""
Autonomous Drone Controller for iNAV 8.x via MSP Protocol
==========================================================
Takes off to 2m altitude, holds for 20 seconds, then lands.

Two configurable flight modes:
  A) RC Override  — Sends virtual RC via MSP, uses FC's ALTHOLD
  B) Waypoint     — Uploads & executes a waypoint mission (requires GPS)

Hardware : Raspberry Pi  →  UART  →  iFlight Blitz F7 (iNAV 8.x)
Port     : /dev/ttyAMA0 @ 115200 baud

PREREQUISITES (see setup_instructions() at bottom):
  1. Set receiver type to MSP in iNAV Configurator
  2. Or configure msp_override_channels in CLI

Press Ctrl+C at any time for EMERGENCY STOP.
"""

import serial
import struct
import time
import sys
import signal

# ╔══════════════════════════════════════════════════════════════╗
# ║                     CONFIGURATION                           ║
# ╚══════════════════════════════════════════════════════════════╝

SERIAL_PORT         = '/dev/ttyAMA0'
BAUD_RATE           = 115200
TARGET_ALTITUDE_CM  = 200       # 2 meters
HOLD_DURATION_SEC   = 20        # Hold time at altitude
ALTITUDE_TOLERANCE  = 30        # ±30 cm to consider "at target"
LAND_THRESHOLD_CM   = 15        # Below this = landed
CLIMB_THROTTLE      = 1600      # Throttle for climbing in ALTHOLD
DESCEND_THROTTLE    = 1350      # Throttle for descending in ALTHOLD
HOVER_THROTTLE      = 1500      # Mid-stick = hold altitude in ALTHOLD
CLIMB_TIMEOUT_SEC   = 30        # Max time to reach altitude
LAND_TIMEOUT_SEC    = 30        # Max time to land
MAX_FLIGHT_TIME_SEC = 120       # Total safety timeout
RC_LOOP_HZ          = 20        # RC send rate (20 Hz = every 50 ms)

# ╔══════════════════════════════════════════════════════════════╗
# ║              MSP COMMAND IDs (from iNAV source)             ║
# ╚══════════════════════════════════════════════════════════════╝

MSP_FC_VARIANT      = 2
MSP_FC_VERSION      = 3
MSP_MODE_RANGES     = 34
MSP_SET_MODE_RANGE  = 35
MSP_STATUS          = 101
MSP_RAW_GPS         = 106
MSP_ATTITUDE        = 108
MSP_ALTITUDE        = 109
MSP_ANALOG          = 110
MSP_BOXIDS          = 119
MSP_NAV_STATUS      = 121
MSP_STATUS_EX       = 150
MSP_SET_RAW_RC      = 200
MSP_SET_WP          = 209
MSP_SET_MOTOR       = 214
MSP_EEPROM_WRITE    = 250
MSP_REBOOT          = 68

# Permanent Box IDs (from fc_msp_box.c)
PERM_ARM            = 0
PERM_ANGLE          = 1
PERM_NAV_ALTHOLD    = 3
PERM_NAV_RTH        = 10
PERM_NAV_POSHOLD    = 11
PERM_NAV_WP         = 28
PERM_MSP_OVERRIDE   = 50

# RC constants
RC_LOW  = 1000
RC_MID  = 1500
RC_HIGH = 2000

# Channel indices (0-based)
CH_ROLL     = 0
CH_PITCH    = 1
CH_THROTTLE = 2
CH_YAW      = 3
CH_AUX1     = 4   # ARM
CH_AUX2     = 5   # ANGLE + NAV ALTHOLD
CH_AUX3     = 6   # NAV POSHOLD / NAV WP
CH_AUX4     = 7   # spare

# Waypoint actions
WP_WAYPOINT = 1
WP_RTH      = 4

# ╔══════════════════════════════════════════════════════════════╗
# ║                  MSP PROTOCOL LAYER                         ║
# ╚══════════════════════════════════════════════════════════════╝

class MSP:
    """MSP v1 protocol handler for iNAV FC communication."""

    def __init__(self, port, baudrate, timeout=1.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for FC to initialize

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd, payload=None):
        """Send MSP v1 command, return parsed response or None."""
        if payload is None:
            payload = []
        size = len(payload)
        cs = size ^ cmd
        for b in payload:
            cs ^= b

        frame = struct.pack('<3sBB', b'$M<', size, cmd)
        frame += bytes(payload) + struct.pack('<B', cs & 0xFF)

        self.ser.reset_input_buffer()
        self.ser.write(frame)
        return self._read()

    def _read(self):
        """Read MSP v1 response frame."""
        hdr = self.ser.read(3)
        if len(hdr) < 3 or hdr not in (b'$M>', b'$M!'):
            return None

        raw = self.ser.read(2)
        if len(raw) < 2:
            return None
        size, cmd = struct.unpack('<BB', raw)

        data = list(self.ser.read(size))
        if len(data) < size:
            return None

        cs = struct.unpack('<B', self.ser.read(1))[0]
        expected = size ^ cmd
        for b in data:
            expected ^= b
        if cs != (expected & 0xFF):
            return None

        return {'cmd': cmd, 'data': data, 'error': hdr == b'$M!'}

    # ── Telemetry ──────────────────────────────────────────────

    def get_fc_variant(self):
        r = self.send(MSP_FC_VARIANT)
        if r and not r['error']:
            return ''.join(chr(b) for b in r['data'][:4])
        return None

    def get_fc_version(self):
        r = self.send(MSP_FC_VERSION)
        if r and not r['error'] and len(r['data']) >= 3:
            return f"{r['data'][0]}.{r['data'][1]}.{r['data'][2]}"
        return None

    def get_status(self):
        """Read MSP_STATUS_EX — returns sensor flags, arming state, etc."""
        r = self.send(MSP_STATUS_EX)
        if not r or r['error'] or len(r['data']) < 15:
            return None
        d = r['data']
        sensors = struct.unpack('<H', bytes(d[4:6]))[0]
        flags   = struct.unpack('<I', bytes(d[6:10]))[0]
        arming  = struct.unpack('<H', bytes(d[13:15]))[0]
        return {
            'cycle_time':  struct.unpack('<H', bytes(d[0:2]))[0],
            'i2c_errors':  struct.unpack('<H', bytes(d[2:4]))[0],
            'sensors':     sensors,
            'flags':       flags,
            'arming_flags': arming,
            'armed':       bool(flags & 1),
            'has_acc':     bool(sensors & (1 << 0)),
            'has_baro':    bool(sensors & (1 << 1)),
            'has_mag':     bool(sensors & (1 << 2)),
            'has_gps':     bool(sensors & (1 << 3)),
        }

    def get_altitude(self):
        """Returns (altitude_cm, vario_cm_s) or (None, None)."""
        r = self.send(MSP_ALTITUDE)
        if r and not r['error'] and len(r['data']) >= 6:
            alt   = struct.unpack('<i', bytes(r['data'][0:4]))[0]
            vario = struct.unpack('<h', bytes(r['data'][4:6]))[0]
            return alt, vario
        return None, None

    def get_attitude(self):
        """Returns (roll_deg, pitch_deg, yaw_deg) or Nones."""
        r = self.send(MSP_ATTITUDE)
        if r and not r['error'] and len(r['data']) >= 6:
            roll  = struct.unpack('<h', bytes(r['data'][0:2]))[0] / 10.0
            pitch = struct.unpack('<h', bytes(r['data'][2:4]))[0] / 10.0
            yaw   = struct.unpack('<h', bytes(r['data'][4:6]))[0]
            return roll, pitch, yaw
        return None, None, None

    def get_analog(self):
        """Battery voltage, current draw, etc."""
        r = self.send(MSP_ANALOG)
        if r and not r['error'] and len(r['data']) >= 7:
            d = r['data']
            return {
                'vbat':      d[0] / 10.0,
                'mah_drawn': struct.unpack('<H', bytes(d[1:3]))[0],
                'rssi':      struct.unpack('<H', bytes(d[3:5]))[0],
                'amps':      struct.unpack('<H', bytes(d[5:7]))[0] / 100.0,
            }
        return None

    def get_gps(self):
        """GPS fix data — only useful if GPS hardware is present."""
        r = self.send(MSP_RAW_GPS)
        if r and not r['error'] and len(r['data']) >= 16:
            d = r['data']
            return {
                'fix':       d[0],
                'sats':      d[1],
                'lat':       struct.unpack('<i', bytes(d[2:6]))[0] / 1e7,
                'lon':       struct.unpack('<i', bytes(d[6:10]))[0] / 1e7,
                'alt_m':     struct.unpack('<h', bytes(d[10:12]))[0],
                'speed_cms': struct.unpack('<H', bytes(d[12:14]))[0],
                'course':    struct.unpack('<H', bytes(d[14:16]))[0] / 10.0,
            }
        return None

    def get_nav_status(self):
        r = self.send(MSP_NAV_STATUS)
        if r and not r['error'] and len(r['data']) >= 7:
            d = r['data']
            return {
                'mode': d[0], 'state': d[1],
                'wp_action': d[2], 'wp_number': d[3], 'error': d[4],
            }
        return None

    # ── Control ────────────────────────────────────────────────

    def set_raw_rc(self, channels):
        """Send RC override (list of 8+ channel values, 1000–2000)."""
        payload = []
        for ch in channels[:16]:
            payload.extend(struct.pack('<H', max(RC_LOW, min(RC_HIGH, ch))))
        self.send(MSP_SET_RAW_RC, payload)

    def set_waypoint(self, wp_no, action, lat, lon, alt_cm, p1=0, p2=0, p3=0, flag=0):
        """Upload a single waypoint. lat/lon in degrees, alt in cm."""
        payload = struct.pack('<BBlliHHHB',
            wp_no, action,
            int(lat * 1e7), int(lon * 1e7),
            alt_cm, p1, p2, p3, flag
        )
        return self.send(MSP_SET_WP, list(payload))

    def set_mode_range(self, idx, perm_id, aux_ch, step_start, step_end):
        """Configure a mode activation range slot.
        aux_ch: 0=AUX1, 1=AUX2 ...
        steps : 0=900, 32=1700, 48=2100  (step × 25 + 900)
        """
        payload = struct.pack('<BBBBB', idx, perm_id, aux_ch, step_start, step_end)
        return self.send(MSP_SET_MODE_RANGE, list(payload))

    def save_eeprom(self):
        return self.send(MSP_EEPROM_WRITE)

    def reboot(self):
        return self.send(MSP_REBOOT)

    def set_motors(self, m1, m2, m3, m4):
        """Direct motor output (bypasses PID — use only for ground tests!)."""
        data = struct.pack('<HHHHHHHH', m1, m2, m3, m4, 1000, 1000, 1000, 1000)
        self.send(MSP_SET_MOTOR, list(data))


# ╔══════════════════════════════════════════════════════════════╗
# ║                  DRONE CONTROLLER                           ║
# ╚══════════════════════════════════════════════════════════════╝

class DroneController:
    """High-level autonomous drone operations."""

    def __init__(self, msp: MSP):
        self.msp = msp
        self.abort = False
        self.armed = False
        self.start_alt = 0
        self.rc = [RC_MID] * 8
        self.rc[CH_THROTTLE] = RC_LOW
        self.rc[CH_AUX1] = RC_LOW   # disarmed
        self.rc[CH_AUX2] = RC_LOW   # no mode
        self.rc[CH_AUX3] = RC_LOW
        self.rc[CH_AUX4] = RC_LOW
        signal.signal(signal.SIGINT, self._emergency)

    def _emergency(self, *_):
        print("\n🚨 EMERGENCY STOP!")
        self.abort = True
        self._disarm_now()
        sys.exit(1)

    # ── RC helpers ─────────────────────────────────────────────

    def _tx(self):
        """Send current RC state once."""
        self.msp.set_raw_rc(self.rc)

    def _tx_for(self, seconds):
        """Keep sending RC state for a duration."""
        interval = 1.0 / RC_LOOP_HZ
        end = time.time() + seconds
        while time.time() < end and not self.abort:
            self._tx()
            time.sleep(interval)

    # ── Arming ─────────────────────────────────────────────────

    def arm(self):
        """Arm the FC via AUX1 high."""
        print("⚡ Arming ...")
        self.rc[CH_THROTTLE] = RC_LOW
        self.rc[CH_AUX1] = RC_HIGH
        self._tx_for(2.0)
        time.sleep(0.3)

        status = self.msp.get_status()
        if status and status['armed']:
            self.armed = True
            alt, _ = self.msp.get_altitude()
            self.start_alt = alt if alt is not None else 0
            print(f"✅ Armed!  Reference altitude = {self.start_alt} cm")
            return True

        af = status.get('arming_flags', 0) if status else 0
        print(f"❌ Arming FAILED — arming_flags=0x{af:04X}")
        print("   Run 'status' in iNAV CLI to see arming prevention flags.")
        return False

    def _disarm_now(self):
        """Emergency disarm — cut everything."""
        self.rc[CH_AUX1] = RC_LOW
        self.rc[CH_AUX2] = RC_LOW
        self.rc[CH_AUX3] = RC_LOW
        self.rc[CH_THROTTLE] = RC_LOW
        for _ in range(40):
            self._tx()
            time.sleep(0.05)
        self.armed = False

    def disarm(self):
        """Normal disarm."""
        print("🔒 Disarming ...")
        self._disarm_now()
        print("🔒 Disarmed.")

    # ── Relative altitude ──────────────────────────────────────

    def rel_alt(self):
        """Current altitude relative to arm point, in cm."""
        alt, _ = self.msp.get_altitude()
        if alt is None:
            return 0
        return alt - self.start_alt

    # ── Pre-flight checks ──────────────────────────────────────

    def preflight(self, need_gps=False):
        print("\n" + "=" * 52)
        print("   PRE-FLIGHT CHECKS")
        print("=" * 52)

        var = self.msp.get_fc_variant()
        ver = self.msp.get_fc_version()
        if not var:
            print("❌ Cannot communicate with FC!")
            return False
        print(f"✅ FC: {var} v{ver}")

        st = self.msp.get_status()
        if not st:
            print("❌ Cannot read status!")
            return False
        print(f"   Sensors — ACC: {'✅' if st['has_acc'] else '❌'}  "
              f"BARO: {'✅' if st['has_baro'] else '❌'}  "
              f"GPS: {'✅' if st['has_gps'] else '⬜'}  "
              f"MAG: {'✅' if st['has_mag'] else '⬜'}")

        if not st['has_acc']:
            print("❌ Accelerometer required!"); return False
        if not st['has_baro']:
            print("❌ Barometer required for altitude hold!"); return False
        if need_gps and not st['has_gps']:
            print("❌ GPS required for waypoint mode!"); return False
        if st['armed']:
            print("⚠️  Already armed — disarm first."); return False

        alt, vario = self.msp.get_altitude()
        if alt is not None:
            print(f"✅ Altitude: {alt} cm  (vario {vario} cm/s)")
        else:
            print("❌ Cannot read altitude!"); return False

        analog = self.msp.get_analog()
        if analog and analog['vbat'] > 1.0:
            print(f"✅ Battery: {analog['vbat']:.1f} V")
            if analog['vbat'] < 13.2:  # ~3.3V/cell for 4S
                print("⚠️  Low battery warning!")

        roll, pitch, _ = self.msp.get_attitude()
        if roll is not None:
            print(f"✅ Attitude: roll={roll:.1f}° pitch={pitch:.1f}°")
            if abs(roll) > 10 or abs(pitch) > 10:
                print("⚠️  Drone not level!"); return False

        if need_gps:
            gps = self.msp.get_gps()
            if gps:
                print(f"✅ GPS: fix={gps['fix']}  sats={gps['sats']}  "
                      f"lat={gps['lat']:.7f}  lon={gps['lon']:.7f}")
                if gps['fix'] < 2 or gps['sats'] < 6:
                    print("❌ Insufficient GPS fix (need fix≥2, sats≥6)")
                    return False
            else:
                print("❌ Cannot read GPS!"); return False

        print("=" * 52)
        print("   ALL CHECKS PASSED ✅")
        print("=" * 52 + "\n")
        return True

    # ── Mode configuration ─────────────────────────────────────

    def configure_modes(self, include_wp=False):
        """Program mode ranges into FC via MSP and save to EEPROM."""
        print("⚙️  Configuring mode ranges ...")
        # Activation range: AUX high (1700–2100) → steps 32–48
        H_START, H_END = 32, 48

        # Slot 0: ARM on AUX1
        self.msp.set_mode_range(0, PERM_ARM, 0, H_START, H_END)
        print("   [0] ARM           → AUX1 [1700–2100]")

        # Slot 1: ANGLE on AUX2
        self.msp.set_mode_range(1, PERM_ANGLE, 1, H_START, H_END)
        print("   [1] ANGLE         → AUX2 [1700–2100]")

        # Slot 2: NAV ALTHOLD on AUX2 (stacked with ANGLE)
        self.msp.set_mode_range(2, PERM_NAV_ALTHOLD, 1, H_START, H_END)
        print("   [2] NAV ALTHOLD   → AUX2 [1700–2100]")

        if include_wp:
            # Slot 3: NAV POSHOLD on AUX3
            self.msp.set_mode_range(3, PERM_NAV_POSHOLD, 2, H_START, H_END)
            print("   [3] NAV POSHOLD   → AUX3 [1700–2100]")
            # Slot 4: NAV WP on AUX4
            self.msp.set_mode_range(4, PERM_NAV_WP, 3, H_START, H_END)
            print("   [4] NAV WP        → AUX4 [1700–2100]")

        time.sleep(0.2)
        self.msp.save_eeprom()
        print("✅ Saved to EEPROM.\n")
        time.sleep(1)


# ╔══════════════════════════════════════════════════════════════╗
# ║         MODE A  —  RC OVERRIDE  (ALTHOLD)                  ║
# ╚══════════════════════════════════════════════════════════════╝

class RCOverrideFlight:
    """Take off, hold, land using RC override + NAV ALTHOLD."""

    def __init__(self, drone: DroneController):
        self.d = drone
        self.interval = 1.0 / RC_LOOP_HZ

    def run(self):
        print("\n" + "═" * 52)
        print("   MODE A: RC OVERRIDE FLIGHT")
        print(f"   Target {TARGET_ALTITUDE_CM} cm  •  Hold {HOLD_DURATION_SEC}s")
        print("═" * 52 + "\n")

        try:
            # 1 — Arm
            if not self.d.arm():
                return False
            time.sleep(0.5)

            # 2 — Activate ANGLE + ALTHOLD, set mid throttle
            print("\n🚀 TAKEOFF — climbing to {} cm ...".format(TARGET_ALTITUDE_CM))
            self.d.rc[CH_AUX2] = RC_HIGH      # ANGLE + ALTHOLD on
            self.d.rc[CH_THROTTLE] = HOVER_THROTTLE
            self.d._tx_for(0.5)

            # 3 — Climb
            if not self._climb():
                print("❌ Failed to reach altitude!")
                self.d.disarm()
                return False

            # 4 — Hold
            print(f"\n⏸️  HOLDING at altitude for {HOLD_DURATION_SEC}s ...")
            self._hold()

            # 5 — Land
            print("\n🛬 LANDING ...")
            self._land()

            # 6 — Disarm
            self.d.disarm()
            print("\n✅ Flight complete!")
            return True

        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.d.disarm()
            return False

    def _climb(self):
        self.d.rc[CH_THROTTLE] = CLIMB_THROTTLE
        t0 = time.time()
        while not self.d.abort:
            if time.time() - t0 > CLIMB_TIMEOUT_SEC:
                print("\n   ⏰ Timeout!"); return False

            self.d._tx()
            alt = self.d.rel_alt()
            _, vario = self.d.msp.get_altitude()
            vario = vario or 0
            sys.stdout.write(
                f"\r   Alt: {alt:5d} cm | Vario: {vario:+4d} cm/s | "
                f"Target: {TARGET_ALTITUDE_CM} cm    "
            )
            sys.stdout.flush()

            if alt >= TARGET_ALTITUDE_CM - ALTITUDE_TOLERANCE:
                print(f"\n   ✅ Reached {alt} cm")
                self.d.rc[CH_THROTTLE] = HOVER_THROTTLE
                self.d._tx()
                return True
            time.sleep(self.interval)
        return False

    def _hold(self):
        self.d.rc[CH_THROTTLE] = HOVER_THROTTLE
        t0 = time.time()
        while time.time() - t0 < HOLD_DURATION_SEC and not self.d.abort:
            self.d._tx()
            alt = self.d.rel_alt()
            remaining = HOLD_DURATION_SEC - (time.time() - t0)
            sys.stdout.write(
                f"\r   Alt: {alt:5d} cm | Remaining: {remaining:5.1f}s    "
            )
            sys.stdout.flush()
            time.sleep(self.interval)
        print()

    def _land(self):
        self.d.rc[CH_THROTTLE] = DESCEND_THROTTLE
        t0 = time.time()
        while not self.d.abort:
            if time.time() - t0 > LAND_TIMEOUT_SEC:
                print("\n   ⏰ Land timeout!"); break

            self.d._tx()
            alt = self.d.rel_alt()
            sys.stdout.write(f"\r   Alt: {alt:5d} cm    ")
            sys.stdout.flush()

            if alt <= LAND_THRESHOLD_CM:
                print(f"\n   ✅ Landed ({alt} cm)")
                self.d.rc[CH_THROTTLE] = RC_LOW
                self.d._tx_for(1.0)
                return
            time.sleep(self.interval)

        # Safety fallback
        self.d.rc[CH_THROTTLE] = RC_LOW
        self.d._tx_for(1.0)


# ╔══════════════════════════════════════════════════════════════╗
# ║         MODE B  —  WAYPOINT MISSION  (requires GPS)         ║
# ╚══════════════════════════════════════════════════════════════╝

class WaypointFlight:
    """Take off, hold, land using an uploaded waypoint mission."""

    def __init__(self, drone: DroneController):
        self.d = drone
        self.interval = 1.0 / RC_LOOP_HZ

    def run(self):
        print("\n" + "═" * 52)
        print("   MODE B: WAYPOINT MISSION FLIGHT")
        print(f"   Target {TARGET_ALTITUDE_CM} cm  •  Hold {HOLD_DURATION_SEC}s")
        print("═" * 52 + "\n")

        try:
            # 1 — Get current GPS position
            gps = self.d.msp.get_gps()
            if not gps or gps['fix'] < 2:
                print("❌ No GPS fix!"); return False
            lat, lon = gps['lat'], gps['lon']
            print(f"📍 Home: {lat:.7f}, {lon:.7f}")

            # 2 — Upload waypoints
            #   WP1: fly to current position at target altitude, hold time = HOLD_DURATION_SEC
            #   WP2: RTH (return to home and land)
            print("📋 Uploading waypoints ...")
            self.d.msp.set_waypoint(
                wp_no=1, action=WP_WAYPOINT,
                lat=lat, lon=lon,
                alt_cm=TARGET_ALTITUDE_CM,
                p1=0,   # speed (0 = default)
                p2=0, p3=0,
                flag=0  # not last
            )
            print(f"   WP1: HOLD at {TARGET_ALTITUDE_CM} cm")

            self.d.msp.set_waypoint(
                wp_no=2, action=WP_RTH,
                lat=0, lon=0,   # RTH ignores lat/lon
                alt_cm=0,       # land
                p1=0, p2=0, p3=0,
                flag=0xA5       # last waypoint flag
            )
            print("   WP2: RTH + LAND")

            # 3 — Arm
            if not self.d.arm():
                return False
            time.sleep(0.5)

            # 4 — Activate ANGLE + ALTHOLD first
            self.d.rc[CH_AUX2] = RC_HIGH
            self.d.rc[CH_THROTTLE] = HOVER_THROTTLE
            self.d._tx_for(1.0)

            # 5 — Activate NAV WP mode
            print("🛫 Starting waypoint mission ...")
            self.d.rc[CH_AUX4] = RC_HIGH  # NAV WP on AUX4
            self.d._tx_for(1.0)

            # 6 — Monitor altitude (climb phase)
            print("⬆️  Climbing ...")
            t0 = time.time()
            reached = False
            while not self.d.abort and time.time() - t0 < CLIMB_TIMEOUT_SEC:
                self.d._tx()
                alt = self.d.rel_alt()
                sys.stdout.write(f"\r   Alt: {alt:5d} cm    ")
                sys.stdout.flush()

                if alt >= TARGET_ALTITUDE_CM - ALTITUDE_TOLERANCE:
                    if not reached:
                        print(f"\n   ✅ Reached {alt} cm — holding {HOLD_DURATION_SEC}s ...")
                        reached = True
                        break
                time.sleep(self.interval)

            # 7 — Hold phase (keep sending RC to keep MSP alive)
            if reached:
                hold_start = time.time()
                while time.time() - hold_start < HOLD_DURATION_SEC and not self.d.abort:
                    self.d._tx()
                    alt = self.d.rel_alt()
                    remaining = HOLD_DURATION_SEC - (time.time() - hold_start)
                    sys.stdout.write(
                        f"\r   Alt: {alt:5d} cm | Remaining: {remaining:5.1f}s    "
                    )
                    sys.stdout.flush()
                    time.sleep(self.interval)
                print()

            # 8 — The RTH waypoint should trigger descent & landing
            #     Monitor until nav state indicates landing done
            print("🛬 RTH descent ...")
            land_start = time.time()
            while not self.d.abort and time.time() - land_start < LAND_TIMEOUT_SEC + 30:
                self.d._tx()
                alt = self.d.rel_alt()
                nav = self.d.msp.get_nav_status()
                sys.stdout.write(f"\r   Alt: {alt:5d} cm    ")
                sys.stdout.flush()

                if alt <= LAND_THRESHOLD_CM:
                    print(f"\n   ✅ Landed ({alt} cm)")
                    break

                # Check if nav state = idle (mission done)
                if nav and nav['mode'] == 0 and nav['state'] == 0:
                    print(f"\n   ✅ Mission complete (alt={alt} cm)")
                    break
                time.sleep(self.interval)

            # 9 — Disarm
            self.d.disarm()
            print("\n✅ Waypoint flight complete!")
            return True

        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.d.disarm()
            return False


# ╔══════════════════════════════════════════════════════════════╗
# ║                     MAIN ENTRY POINT                        ║
# ╚══════════════════════════════════════════════════════════════╝

def print_setup_instructions():
    """Print required iNAV configuration steps."""
    print("""
╔══════════════════════════════════════════════════════════════╗
║              REQUIRED iNAV CONFIGURATION                     ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  Before first use, configure the FC via iNAV Configurator:   ║
║                                                              ║
║  OPTION 1 — Set receiver type to MSP (simplest):            ║
║    • Configurator → Receiver tab → Receiver Type → MSP      ║
║    • Save & Reboot                                           ║
║                                                              ║
║  OPTION 2 — Use MSP RC Override (if using a real TX too):    ║
║    • CLI → set msp_override_channels = 255                   ║
║    • CLI → save                                              ║
║    • Modes tab → assign MSP RC OVERRIDE to a switch          ║
║                                                              ║
║  The script will auto-configure ARM, ANGLE, and ALTHOLD      ║
║  mode ranges via MSP and save to EEPROM on startup.          ║
║                                                              ║
║  For Waypoint mode, a GPS module must be connected and       ║
║  have a valid fix (≥6 satellites).                           ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
""")


def countdown(seconds, message="Starting in"):
    """Visual countdown before action."""
    for i in range(seconds, 0, -1):
        sys.stdout.write(f"\r⏳ {message} {i}s ...  ")
        sys.stdout.flush()
        time.sleep(1)
    print()


def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║          AUTONOMOUS DRONE CONTROLLER  v1.0                   ║
║          iNAV 8.x  •  MSP Protocol  •  Raspberry Pi         ║
╠══════════════════════════════════════════════════════════════╣
║  Takeoff → 2m altitude → Hold 20s → Land                    ║
║  Press Ctrl+C at ANY time for emergency stop                 ║
╚══════════════════════════════════════════════════════════════╝
""")

    # ── Mode selection ─────────────────────────────────────────
    print("Select flight mode:")
    print("  [A] RC Override  — uses ALTHOLD (barometer only, no GPS needed)")
    print("  [B] Waypoint     — uploads mission to FC (requires GPS)")
    print("  [S] Show setup instructions")
    print("  [Q] Quit\n")

    while True:
        choice = input("➜ Enter mode (A/B/S/Q): ").strip().upper()
        if choice == 'Q':
            print("Bye!"); return
        if choice == 'S':
            print_setup_instructions(); continue
        if choice in ('A', 'B'):
            break
        print("Invalid choice, try again.")

    use_wp = (choice == 'B')

    # ── Connect ────────────────────────────────────────────────
    print(f"\n🔌 Connecting to {SERIAL_PORT} @ {BAUD_RATE} ...")
    try:
        msp = MSP(SERIAL_PORT, BAUD_RATE)
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        return

    drone = DroneController(msp)

    try:
        # ── Pre-flight ─────────────────────────────────────────
        if not drone.preflight(need_gps=use_wp):
            print("❌ Pre-flight checks failed. Fix issues and retry.")
            return

        # ── Configure modes on FC ──────────────────────────────
        print("Do you want the script to auto-configure mode ranges on the FC?")
        print("  [Y] Yes — write ARM/ANGLE/ALTHOLD to EEPROM (safe if no existing modes)")
        print("  [N] No  — I already configured modes manually\n")
        conf = input("➜ Configure modes? (Y/N): ").strip().upper()
        if conf == 'Y':
            drone.configure_modes(include_wp=use_wp)
            print("⚠️  If this is the first time, you may need to reboot the FC.")
            reboot = input("➜ Reboot FC now? (Y/N): ").strip().upper()
            if reboot == 'Y':
                print("🔄 Rebooting FC ...")
                msp.reboot()
                msp.close()
                time.sleep(5)
                print("🔌 Reconnecting ...")
                msp = MSP(SERIAL_PORT, BAUD_RATE)
                drone = DroneController(msp)
                if not drone.preflight(need_gps=use_wp):
                    print("❌ Post-reboot checks failed.")
                    return

        # ── Safety countdown ───────────────────────────────────
        print("\n🔴 REMOVE HANDS FROM THE DRONE!")
        print("🔴 ENSURE AREA IS CLEAR!")
        countdown(5, "Flight begins in")

        # ── Execute flight ─────────────────────────────────────
        if use_wp:
            flight = WaypointFlight(drone)
        else:
            flight = RCOverrideFlight(drone)

        success = flight.run()

        if success:
            print("\n🎉 Mission accomplished!")
        else:
            print("\n⚠️  Mission did not complete successfully.")

    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        drone.disarm()

    finally:
        msp.close()
        print("🔌 Serial port closed.")


if __name__ == '__main__':
    main()
