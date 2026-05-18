#!/usr/bin/env python3
import time
import sys
import termios
import tty
import select

from pid2m import (
    MSP, DroneController, 
    SERIAL_PORT, BAUD_RATE, 
    CH_THROTTLE, RC_LOW, RC_LOOP_HZ
)

def get_key():
    """Non-blocking function to read a single character from standard input."""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def format_imu(imu):
    """Formats the IMU dictionary into a neat, compact string."""
    if not imu:
        return "N/A"
    
    parts = []
    if 'acc_x' in imu: parts.append(f"ax:{imu['acc_x']:4d}")
    if 'acc_y' in imu: parts.append(f"ay:{imu['acc_y']:4d}")
    if 'acc_z' in imu: parts.append(f"az:{imu['acc_z']:4d}")
    if 'gyro_x' in imu: parts.append(f"gx:{imu['gyro_x']:4d}")
    if 'gyro_y' in imu: parts.append(f"gy:{imu['gyro_y']:4d}")
    if 'gyro_z' in imu: parts.append(f"gz:{imu['gyro_z']:4d}")
    
    if parts:
        return " ".join(parts)
    return str(imu)

def main():
    current_throttle = 1000

    print(f"🔌 Connecting to FC on {SERIAL_PORT} @ {BAUD_RATE} ...")
    try:
        msp = MSP(SERIAL_PORT, BAUD_RATE)
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        return

    drone = DroneController(msp)

    print("\n" + "="*50)
    print("🚀 Drone Control Initialized (WASD Terminal Mode).")
    print("="*50)
    print("🎮 Controls:")
    print("   [W] : Increase Throttle (+10)")
    print("   [S] : Decrease Throttle (-10)")
    print("   [Q] : Quit / Emergency Stop")
    print("="*50 + "\n")

    print("⚠️  Ensure the drone is ARMED if you want motors to spin.\n")

    interval = 1.0 / RC_LOOP_HZ

    try:
        while True:
            start_loop = time.time()
            
            # Read keyboard input
            key = get_key()
            if key:
                if key.lower() == 'w':
                    current_throttle = min(2000, current_throttle + 10)
                elif key.lower() == 's':
                    current_throttle = max(1000, current_throttle - 10)
                elif key.lower() == 'q':
                    print("\n🛑 Quit requested. Stopping motors and exiting...")
                    break
                    
            drone.rc[CH_THROTTLE] = current_throttle
            drone._tx()

            # Read sensors
            alt, _ = drone.msp.get_altitude()
            imu = drone.msp.get_imu()
            
            alt_val = alt if alt is not None else 0.0
            imu_str = format_imu(imu)

            # Print to terminal
            # Use carriage return `\r` and ljust to overwrite the same line continuously and neatly
            output_line = f"\r[ DATA ] Thr: {current_throttle:4d} | Alt: {alt_val:5.1f} cm | IMU: {imu_str}"
            sys.stdout.write(output_line.ljust(110))
            sys.stdout.flush()

            # Maintain loop rate (typically 50Hz as per RC_LOOP_HZ)
            elapsed = time.time() - start_loop
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n🛑 Exiting due to KeyboardInterrupt...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        print("\n⬇️  Shutting down safely...")
        # Ensure motors stop on exit
        drone.rc[CH_THROTTLE] = RC_LOW
        drone._tx_for(1.0)
        drone.disarm()
        msp.close()
        print("🔌 Serial port closed.")

if __name__ == '__main__':
    main()
