#!/usr/bin/env python3
import time
import sys
import json

from pid2m import (
    MSP, DroneController, 
    SERIAL_PORT, BAUD_RATE, 
    CH_THROTTLE, CH_AUX2, RC_HIGH, RC_LOW, RC_LOOP_HZ
)

target = 200 # 200 cm target

def main():
    print(f"🔌 Connecting to FC on {SERIAL_PORT} @ {BAUD_RATE} ...")
    try:
        msp = MSP(SERIAL_PORT, BAUD_RATE)
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        return

    drone = DroneController(msp)

    try:
        # Wait for manual arming via switch, just like in pid2m.py
        if not drone.wait_for_arm():
            return
            
        time.sleep(0.5)

        # 1. Takeoff Phase
        print("\n🚀 TAKEOFF — Throttle 1185 for 1 second...")
        flight_log = []
        start_time = time.time()
        
        drone.rc[CH_THROTTLE] = 1185
        interval = 1.0 / RC_LOOP_HZ
        
        end_time = time.time() + 1.0
        while time.time() < end_time and not drone.abort:
            drone._tx()
            
            alt, _ = drone.msp.get_altitude()
            alt = alt if alt is not None else 0
            rel_alt = alt - drone.start_alt
            
            flight_log.append({
                "time": time.time() - start_time,
                "altitude": rel_alt,
                "target": target,
                "error": target - rel_alt
            })
            sys.stdout.write(f"\r   Takeoff Alt: {rel_alt:5d} cm    ")
            sys.stdout.flush()
            time.sleep(interval)

        # 2. Landing Phase
        print("\n\n🛬 LANDING — Gradually decreasing throttle...")
        current_throttle = 1185.0
        drone.rc[CH_THROTTLE] = int(current_throttle)
        
        land_start = time.time()
        recent_alts = [] # To track if we stopped descending
        baseline_acc_z = None
        
        while not drone.abort:
            # Gradually decrease throttle by 30 units per second
            current_throttle = max(1000.0, current_throttle - (30.0 / RC_LOOP_HZ))
            drone.rc[CH_THROTTLE] = int(current_throttle)
            drone._tx()
            
            alt, _ = drone.msp.get_altitude()
            alt = alt if alt is not None else 0
            rel_alt = alt - drone.start_alt
            
            imu = drone.msp.get_imu()
            
            flight_log.append({
                "time": time.time() - start_time,
                "altitude": rel_alt,
                "target": target,
                "error": target - rel_alt
            })
            
            if imu:
                acc_z = imu['acc_z']
                sys.stdout.write(f"\r   Landing Alt: {rel_alt:5d} cm | acc_z: {acc_z:5d}    ")
                
                # Smooth out a baseline for 1G acceleration over time
                if baseline_acc_z is None:
                    baseline_acc_z = acc_z
                else:
                    baseline_acc_z = 0.95 * baseline_acc_z + 0.05 * acc_z
                
                time_landing = time.time() - land_start
                
                # Check for an instantaneous bump (spike in acceleration)
                # We ignore the first 0.5 seconds to avoid the initial descent deceleration spike
                if baseline_acc_z != 0 and abs(acc_z - baseline_acc_z) > abs(baseline_acc_z) * 0.5:
                    if time_landing > 0.5:
                        print(f"\n\n✅ Ground BUMP detected! (acc_z spiked to {acc_z})")
                        break
            else:
                sys.stdout.write(f"\r   Landing Alt: {rel_alt:5d} cm    ")
                time_landing = time.time() - land_start
                
            sys.stdout.flush()
            
            # Barometer ground detection fallback logic
            recent_alts.append(rel_alt)
            if len(recent_alts) > RC_LOOP_HZ * 1.5: # keep 1.5 seconds of history
                recent_alts.pop(0)
            
            variation = max(recent_alts) - min(recent_alts) if recent_alts else 999
            
            # Safe to assume landed if:
            # 1. We are below 15cm relative to start
            if rel_alt <= 15:
                print(f"\n\n✅ Reached ground threshold ({rel_alt} cm).")
                break
                
            # 2. OR we've been landing for > 2 seconds and altitude variation is tiny
            if time_landing > 2.0 and variation < 3 and len(recent_alts) >= RC_LOOP_HZ:
                print(f"\n\n✅ Ground detected by lack of descent (variation: {variation} cm).")
                break
                
            # Failsafe timeout just in case
            if time_landing > 15.0:
                print("\n\n⚠️ Landing timeout! Forcing cut.")
                break
                
            time.sleep(interval)
            
        print("\n✅ Landed.")
        
        # Send RC_LOW for 1 second just to be safe
        drone.rc[CH_THROTTLE] = RC_LOW
        drone._tx_for(1.0)
        
        drone.disarm()
        print("\n✅ Flight complete!")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        drone.disarm()
    finally:
        try:
            if 'flight_log' in locals() and flight_log:
                with open("altitude_data.json", "w") as f:
                    json.dump(flight_log, f, indent=2)
                print(f"\n💾 Flight data saved to altitude_data.json ({len(flight_log)} records)")
        except Exception as ex:
            print(f"\n❌ Failed to save flight log: {ex}")
            
        msp.close()
        print("🔌 Serial port closed.")

if __name__ == '__main__':
    main()
