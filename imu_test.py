#!/usr/bin/env python3
import time
import sys

from pid2m import MSP, SERIAL_PORT, BAUD_RATE

def main():
    print(f"🔌 Connecting to FC on {SERIAL_PORT} @ {BAUD_RATE} ...")
    try:
        msp = MSP(SERIAL_PORT, BAUD_RATE)
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        return

    print("✅ Connected! Starting to read IMU data (Press Ctrl+C to stop)...\n")
    
    try:
        while True:
            imu = msp.get_imu()
            if imu:
                sys.stdout.write(
                    f"\rACC: x={imu['acc_x']:+6d} y={imu['acc_y']:+6d} z={imu['acc_z']:+6d} | "
                    f"GYR: x={imu['gyr_x']:+6d} y={imu['gyr_y']:+6d} z={imu['gyr_z']:+6d} | "
                    f"MAG: x={imu['mag_x']:+6d} y={imu['mag_y']:+6d} z={imu['mag_z']:+6d}   "
                )
                sys.stdout.flush()
            else:
                sys.stdout.write("\r⚠️ Failed to read IMU data... Retrying...                       ")
                sys.stdout.flush()
                
            time.sleep(0.05) # 20Hz refresh rate
    except KeyboardInterrupt:
        print("\n\n🛑 Stopped by user.")
    finally:
        msp.close()
        print("🔌 Serial port closed.")

if __name__ == '__main__':
    main()
