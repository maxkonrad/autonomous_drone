#!/usr/bin/env python3
"""
Autonomous Drone Controller for iNAV 8.x via MSP Protocol
==========================================================
Takes off to 2m altitude, holds for 20 seconds, then lands.
Includes PID altitude controller for smooth throttle management.

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
HOVER_THROTTLE      = 1500      # Base hover throttle (mid-stick)
CLIMB_TIMEOUT_SEC   = 30        # Max time to reach altitude
LAND_TIMEOUT_SEC    = 30        # Max time to land
MAX_FLIGHT_TIME_SEC = 120       # Total safety timeout
RC_LOOP_HZ          = 50        # RC send rate (50 Hz = every 20 ms)
LAND_DESCENT_RATE   = 50        # cm/s — target descent rate during landing

# ── PID Altitude Controller Gains ──────────────────────────
# These tune the software-side PID that converts altitude error
# (target − actual) into a throttle offset around HOVER_THROTTLE.
PID_KP              = 1.5       # Proportional gain (throttle-µs per cm error)
PID_KI              = 0.3       # Integral gain     (slow drift correction)
PID_KD              = 1.0       # Derivative gain   (damp oscillations via vario)
PID_I_LIMIT         = 150       # Max integral term  (anti-windup, in µs)
PID_OUTPUT_MIN      = 1100      # Throttle floor     (never below this)
PID_OUTPUT_MAX      = 1800      # Throttle ceiling   (never above this)

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

# Arming prevention flags (from runtime_config.h — bit positions start at 6)
# Used to decode arming_flags from MSP_STATUS_EX into human-readable names.
ARMING_FLAG_NAMES = {
    (1 <<  6): 'GEOZONE',
    (1 <<  7): 'FAILSAFE_SYSTEM',
    (1 <<  8): 'NOT_LEVEL',
    (1 <<  9): 'SENSORS_CALIBRATING',
    (1 << 10): 'SYSTEM_OVERLOADED',
    (1 << 11): 'NAVIGATION_UNSAFE',
    (1 << 12): 'COMPASS_NOT_CALIBRATED',
    (1 << 13): 'ACCELEROMETER_NOT_CALIBRATED',
    (1 << 14): 'ARM_SWITCH',
    (1 << 15): 'HARDWARE_FAILURE',
    (1 << 16): 'BOXFAILSAFE',
    (1 << 18): 'RC_LINK',
    (1 << 19): 'THROTTLE',
    (1 << 20): 'CLI',
    (1 << 21): 'CMS_MENU',
    (1 << 22): 'OSD_MENU',
    (1 << 23): 'ROLLPITCH_NOT_CENTERED',
    (1 << 24): 'SERVO_AUTOTRIM',
    (1 << 25): 'OOM',
    (1 << 26): 'INVALID_SETTING',
    (1 << 27): 'PWM_OUTPUT_ERROR',
    (1 << 28): 'NO_PREARM',
    (1 << 29): 'DSHOT_BEEPER',
    (1 << 30): 'LANDING_DETECTED',
}

def decode_arming_flags(flags):
    """Return list of human-readable arming prevention flag names."""
    active = []
    for bit, name in ARMING_FLAG_NAMES.items():
        if flags & bit:
            active.append(name)
    return active

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
        """Read MSP_STATUS_EX — returns sensor flags, arming state, etc.

        Note on boxModeFlags (bytes 6-9):
        In iNAV v9, this is a packed bitmask where bit positions map to
        the activeBoxIds[] array order, NOT permanent box IDs. BOXARM is
        always the first entry added (initActiveBoxIds line 191), so
        bit 0 = ARMED. This holds for all iNAV builds.

        Note on armingFlags (bytes 13-14):
        Written as uint16_t, but the actual variable is uint32_t.
        Flags above bit 15 (BOXFAILSAFE, RC_LINK, THROTTLE, etc.)
        are truncated in MSP_STATUS_EX.
        """
        r = self.send(MSP_STATUS_EX)
        if not r or r['error'] or len(r['data']) < 15:
            return None
        d = r['data']
        sensors   = struct.unpack('<H', bytes(d[4:6]))[0]
        box_flags = struct.unpack('<I', bytes(d[6:10]))[0]
        arming    = struct.unpack('<H', bytes(d[13:15]))[0]
        return {
            'cycle_time':  struct.unpack('<H', bytes(d[0:2]))[0],
            'i2c_errors':  struct.unpack('<H', bytes(d[2:4]))[0],
            'sensors':     sensors,
            'box_flags':   box_flags,
            'arming_flags': arming,
            'armed':       bool(box_flags & 1),  # BOXARM = bit 0 (always first in activeBoxIds)
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

        NOT: Bu fonksiyon devre dışı bırakıldı.
        Mode range ayarları iNAV Configurator'dan yapılacak.
        """
        #payload = struct.pack('<BBBBB', idx, perm_id, aux_ch, step_start, step_end)
        #return self.send(MSP_SET_MODE_RANGE, list(payload))
        pass

    def save_eeprom(self):
        # NOT: EEPROM yazma devre dışı bırakıldı.
        # Bu ayar iNAV Configurator'dan yapılacak.
        #return self.send(MSP_EEPROM_WRITE)
        pass

    def reboot(self):
        # NOT: Reboot devre dışı bırakıldı.
        # FC reboot işlemi iNAV Configurator'dan yapılacak.
        #return self.send(MSP_REBOOT)
        pass

    def set_motors(self, m1, m2, m3, m4):
        """Direct motor output (bypasses PID — use only for ground tests!)."""
        data = struct.pack('<HHHHHHHH', m1, m2, m3, m4, 1000, 1000, 1000, 1000)
        self.send(MSP_SET_MOTOR, list(data))


# ╔══════════════════════════════════════════════════════════════╗
# ║                    PID CONTROLLER                           ║
# ╚══════════════════════════════════════════════════════════════╝

class PIDController:
    """Discrete PID controller for altitude → throttle.

    Output = HOVER_THROTTLE + Kp·e + Ki·∫e·dt + Kd·(de/dt)

    * Uses barometer vario for the derivative term instead of
      differentiating the (noisy) altitude reading.
    * Integral term is clamped to ±PID_I_LIMIT to prevent windup.
    * Final output is clamped to [PID_OUTPUT_MIN, PID_OUTPUT_MAX].
    """

    def __init__(self, kp=PID_KP, ki=PID_KI, kd=PID_KD,
                 i_limit=PID_I_LIMIT,
                 out_min=PID_OUTPUT_MIN, out_max=PID_OUTPUT_MAX,
                 base=HOVER_THROTTLE):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.out_min = out_min
        self.out_max = out_max
        self.base = base

        self._integral = 0.0
        self._last_time = None
        self._last_error = None

    def reset(self):
        """Reset integrator and timing state."""
        self._integral = 0.0
        self._last_time = None
        self._last_error = None

    def update(self, error, vario_cm_s=None):
        """Compute throttle value given altitude error (target − actual).

        Args:
            error:       target_alt − current_alt  (positive = too low)
            vario_cm_s:  vertical speed from barometer (positive = climbing)
                         Used as derivative feedback; if None, finite-diff
                         on the error signal is used instead.

        Returns:
            Throttle value clamped to [out_min, out_max].
        """
        now = time.time()
        if self._last_time is None:
            dt = 1.0 / RC_LOOP_HZ   # assume one loop period on first call
        else:
            dt = now - self._last_time
            if dt <= 0:
                dt = 1.0 / RC_LOOP_HZ
        self._last_time = now

        # ── P term ─────────────────────────────────────────────
        p_term = self.kp * error

        # ── I term (with anti-windup clamp) ────────────────────
        self._integral += error * dt
        if self.ki != 0:
            i_limit_val = self.i_limit / self.ki
            self._integral = max(-i_limit_val, min(i_limit_val, self._integral))
        else:
            self._integral = 0
        i_term = self.ki * self._integral

        # ── D term ─────────────────────────────────────────────
        # Prefer vario from the FC (already filtered) over raw
        # finite-difference of the altitude error.
        if vario_cm_s is not None:
            # vario > 0 means climbing → error is shrinking → negative d
            d_term = -self.kd * vario_cm_s
        elif self._last_error is not None:
            d_term = self.kd * (error - self._last_error) / dt
        else:
            d_term = 0.0
        self._last_error = error

        # ── Sum + clamp ────────────────────────────────────────
        output = self.base + p_term + i_term + d_term
        return int(max(self.out_min, min(self.out_max, output)))

    def __repr__(self):
        return (f"PID(Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, "
                f"I={self._integral:.1f}, base={self.base})")


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
        reasons = decode_arming_flags(af)
        print(f"❌ Arming FAILED — arming_flags=0x{af:04X}")
        if reasons:
            print(f"   Blockers: {', '.join(reasons)}")
        else:
            print("   Run 'status' in iNAV CLI to see arming prevention flags.")
        return False

    def _disarm_now(self):
        """Emergency disarm — cut everything."""
        self.rc[CH_AUX1] = RC_LOW
        self.rc[CH_AUX2] = RC_LOW
        self.rc[CH_AUX3] = RC_LOW
        self.rc[CH_AUX4] = RC_LOW
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
        """Program mode ranges into FC via MSP, save to EEPROM, and reboot.

        NOT: Bu fonksiyon devre dışı bırakıldı.
        Mode range, EEPROM ve reboot işlemleri iNAV Configurator'dan yapılacak.
        """
        print("⚙️  Mode yapılandırması atlandı — iNAV Configurator'dan yapılacak.")
        # A reboot is required because iNAV caches which navigation modes
        # are configured at boot (via updateUsedModeActivationConditionFlags).
        # Without a reboot, clearing GPS nav slots has no effect on the
        # ARMING_DISABLED_NAVIGATION_UNSAFE check.

        # # Activation range: AUX high (1700–2100) → steps 32–48
        # H_START, H_END = 32, 48
        #
        # # Slot 0: ARM on AUX1
        # self.msp.set_mode_range(0, PERM_ARM, 0, H_START, H_END)
        # print("   [0] ARM           → AUX1 [1700–2100]")
        #
        # # Slot 1: ANGLE on AUX2
        # self.msp.set_mode_range(1, PERM_ANGLE, 1, H_START, H_END)
        # print("   [1] ANGLE         → AUX2 [1700–2100]")
        #
        # # Slot 2: NAV ALTHOLD on AUX2 (stacked with ANGLE)
        # self.msp.set_mode_range(2, PERM_NAV_ALTHOLD, 1, H_START, H_END)
        # print("   [2] NAV ALTHOLD   → AUX2 [1700–2100]")
        #
        # if include_wp:
        #     # Slot 3: NAV POSHOLD on AUX3
        #     self.msp.set_mode_range(3, PERM_NAV_POSHOLD, 2, H_START, H_END)
        #     print("   [3] NAV POSHOLD   → AUX3 [1700–2100]")
        #     # Slot 4: NAV WP on AUX4
        #     self.msp.set_mode_range(4, PERM_NAV_WP, 3, H_START, H_END)
        #     print("   [4] NAV WP        → AUX4 [1700–2100]")
        # else:
        #     # Clear GPS-dependent mode slots so leftover EEPROM config
        #     # from a previous waypoint run doesn't block arming without GPS.
        #     # step_start == step_end (0,0) → iNAV treats the slot as disabled.
        #     self.msp.set_mode_range(3, PERM_NAV_POSHOLD, 2, 0, 0)
        #     self.msp.set_mode_range(4, PERM_NAV_WP, 3, 0, 0)
        #     print("   [3–4] Cleared GPS nav slots (not needed for RC Override)")
        #
        # time.sleep(0.2)
        # self.msp.save_eeprom()
        # print("✅ Saved to EEPROM.")
        # time.sleep(1)
        #
        # # Reboot FC so it re-evaluates which nav modes are configured.
        # # Without this, isUsingNavigationModes() still returns True from
        # # cached boot-time state, causing NAVIGATION_UNSAFE arming block.
        # print("🔄 Rebooting FC to apply mode changes ...")
        # self.msp.reboot()
        # self.msp.close()
        # time.sleep(5)
        # print("🔌 Reconnecting ...")
        # self.msp = MSP(SERIAL_PORT, BAUD_RATE)
        # print("✅ FC rebooted and reconnected.\n")


# ╔══════════════════════════════════════════════════════════════╗
# ║         MODE A  —  RC OVERRIDE  (PID ALTITUDE CTRL)        ║
# ╚══════════════════════════════════════════════════════════════╝

class RCOverrideFlight:
    """Take off, hold, land using RC override + PID altitude control.

    The PID loop runs at RC_LOOP_HZ and converts (target − actual)
    altitude error into a throttle value each iteration.
    """

    def __init__(self, drone: DroneController):
        self.d = drone
        self.interval = 1.0 / RC_LOOP_HZ
        self.pid = PIDController()

    def run(self):
        print("\n" + "═" * 52)
        print("   MODE A: RC OVERRIDE FLIGHT  (PID)")
        print(f"   Target {TARGET_ALTITUDE_CM} cm  •  Hold {HOLD_DURATION_SEC}s")
        print(f"   PID  Kp={PID_KP}  Ki={PID_KI}  Kd={PID_KD}")
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

            # 3 — Climb (PID targets TARGET_ALTITUDE_CM)
            self.pid.reset()
            if not self._climb():
                print("❌ Failed to reach altitude!")
                self.d.disarm()
                return False

            # 4 — Hold (PID keeps targeting TARGET_ALTITUDE_CM)
            print(f"\n⏸️  HOLDING at altitude for {HOLD_DURATION_SEC}s ...")
            self._hold()

            # 5 — Land (PID target ramps down to 0)
            print("\n🛬 LANDING ...")
            self.pid.reset()
            self._land()

            # 6 — Disarm
            self.d.disarm()
            print("\n✅ Flight complete!")
            return True

        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.d.disarm()
            return False

    # ── PID-driven climb ──────────────────────────────────────

    def _climb(self):
        """PID climbs to TARGET_ALTITUDE_CM."""
        t0 = time.time()
        while not self.d.abort:
            if time.time() - t0 > CLIMB_TIMEOUT_SEC:
                print("\n   ⏰ Timeout!"); return False

            alt = self.d.rel_alt()
            _, vario = self.d.msp.get_altitude()
            vario = vario or 0

            error = TARGET_ALTITUDE_CM - alt
            throttle = self.pid.update(error, vario)
            self.d.rc[CH_THROTTLE] = throttle
            self.d._tx()

            sys.stdout.write(
                f"\r   Alt: {alt:5d} cm | Vario: {vario:+4d} cm/s | "
                f"Thr: {throttle:4d} | Err: {error:+5d} cm    "
            )
            sys.stdout.flush()

            if alt >= TARGET_ALTITUDE_CM - ALTITUDE_TOLERANCE:
                print(f"\n   ✅ Reached {alt} cm")
                return True
            time.sleep(self.interval)
        return False

    # ── PID-driven hold ───────────────────────────────────────

    def _hold(self):
        """PID holds TARGET_ALTITUDE_CM for HOLD_DURATION_SEC."""
        t0 = time.time()
        while time.time() - t0 < HOLD_DURATION_SEC and not self.d.abort:
            alt = self.d.rel_alt()
            _, vario = self.d.msp.get_altitude()
            vario = vario or 0

            error = TARGET_ALTITUDE_CM - alt
            throttle = self.pid.update(error, vario)
            self.d.rc[CH_THROTTLE] = throttle
            self.d._tx()

            remaining = HOLD_DURATION_SEC - (time.time() - t0)
            sys.stdout.write(
                f"\r   Alt: {alt:5d} cm | Thr: {throttle:4d} | "
                f"Err: {error:+5d} | Rem: {remaining:5.1f}s    "
            )
            sys.stdout.flush()
            time.sleep(self.interval)
        print()

    # ── PID-driven landing ────────────────────────────────────

    def _land(self):
        """Ramp the PID target altitude down at LAND_DESCENT_RATE cm/s."""
        t0 = time.time()
        start_alt = self.d.rel_alt()
        # Target starts at current altitude and decreases linearly
        while not self.d.abort:
            elapsed = time.time() - t0
            if elapsed > LAND_TIMEOUT_SEC:
                print("\n   ⏰ Land timeout!"); break

            # Ramp target downward
            target = max(0, start_alt - LAND_DESCENT_RATE * elapsed)

            alt = self.d.rel_alt()
            _, vario = self.d.msp.get_altitude()
            vario = vario or 0

            error = target - alt
            throttle = self.pid.update(error, vario)
            self.d.rc[CH_THROTTLE] = throttle
            self.d._tx()

            sys.stdout.write(
                f"\r   Alt: {alt:5d} cm | Target: {target:5.0f} | "
                f"Thr: {throttle:4d}    "
            )
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
║  1. Receiver Type:                                           ║
║    • Configurator → Receiver tab → Receiver Type → MSP       ║
║    • Or CLI → set msp_override_channels = 255 (if using TX)  ║
║                                                              ║
║  2. Set Mode Ranges (Modes Tab):                             ║
║    You MUST manually configure these for the script to work: ║
║    • ARM          → Add Range → AUX 1 [1700 - 2100]          ║
║    • ANGLE        → Add Range → AUX 2 [1700 - 2100]          ║
║    • NAV ALTHOLD  → Add Range → AUX 2 [1700 - 2100]          ║
║    • NAV WP       → Add Range → AUX 4 [1700 - 2100] (for WP) ║
║                                                              ║
║  WARNING: If you do NOT have a GPS module, do NOT add        ║
║  NAV POSHOLD or NAV WP. Otherwise, you will get a            ║
║  NAVIGATION_UNSAFE error and the drone will not arm!         ║
║                                                              ║
║  After setting these, Save and Reboot your flight controller.║
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
║          iNAV 9.x  •  MSP Protocol  •  Raspberry Pi         ║
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

        # ── Setup completed manually ───────────────────────────
        # Mode range, EEPROM, and Reboot are handled manually via iNAV Configurator.

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
