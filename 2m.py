import serial
import struct
import time
import sys, select

# MSP Command Codes
MSP_IDENT        = 100
MSP_STATUS       = 101
MSP_RAW_IMU      = 102
MSP_ALTITUDE     = 109  # Important for feedback
MSP_SET_RAW_RC   = 200

class AutonomousDrone:
    def __init__(self, port='/dev/ttyACM0'):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.rc = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000] # R, P, Y, T, AUX1-4

    def get_altitude(self):
        """Requests and parses MSP_ALTITUDE (Returns alt in cm)"""
        # Header + Size(0) + Command(109) + Checksum(109)
        cmd = b'$M<\x00\x6d\x6d'
        self.ser.write(cmd)
        
        # Expecting: $M> + Size(6) + 109 + Alt(i32) + Vario(i16) + CRC
        header = self.ser.read(3)
        if header == b'$M>':
            size = ord(self.ser.read(1))
            code = ord(self.ser.read(1))
            payload = self.ser.read(size)
            crc = self.ser.read(1)
            
            if code == MSP_ALTITUDE:
                # iNav MSP_ALTITUDE: 4 bytes for altitude (cm), 2 bytes for vario
                alt_cm = struct.unpack('<i', payload[0:4])[0]
                return alt_cm
        return None

    def send_rc(self):
        """Sends current RC channels to FC"""
        size = 16 # 8 channels * 2 bytes
        code = MSP_SET_RAW_RC
        payload = bytearray()
        checksum = size ^ code
        for val in self.rc:
            payload += struct.pack('<H', val)
        for b in payload:
            checksum ^= b
        
        self.ser.write(b'$M<' + bytes([size, code]) + payload + bytes([checksum]))

    def execute_mission(self):
        # 1. Arm
        print("Arming...")
        self.rc[3] = 1000; self.rc[4] = 1900 # Throttle low, AUX1 high
        for _ in range(20): self.send_rc(); time.sleep(0.05)

        # 2. Ascend to 200cm
        print("Ascending...")
        target_alt = 200 
        while True:
            current_alt = self.get_altitude()
            if current_alt is not None:
                print(f"Alt: {current_alt}cm")
                if current_alt >= target_alt:
                    break
            self.rc[3] = 1600 # Ascent throttle
            self.send_rc()
            time.sleep(0.1)

        # 3. Wait 5 seconds (Hover)
        print("Target reached. Hovering...")
        start_hover = time.time()
        while time.time() - start_hover < 5:
            self.rc[3] = 1500 # Assume mid-throttle hover
            self.send_rc()
            time.sleep(0.1)

        # 4. Land
        print("Landing...")
        while True:
            current_alt = self.get_altitude()
            if current_alt is not None and current_alt <= 10: # Near ground
                break
            self.rc[3] = 1350 # Descent throttle
            self.send_rc()
            time.sleep(0.1)

        # 5. Disarm
        self.rc[4] = 1000
        self.send_rc()
        print("Finished.")

if __name__ == "__main__":
    drone = AutonomousDrone()
    print("Monitoring altitude. Press Enter to execute mission, or type 'q' then Enter to quit.")
    try:
        while True:
            alt = drone.get_altitude()
            if alt is not None:
                print(f"Alt: {alt} cm")
            else:
                print("No altitude data")

            # wait up to 1 second for user input; if present, handle it
            r, _, _ = select.select([sys.stdin], [], [], 1.0)
            if r:
                line = sys.stdin.readline().strip().lower()
                if line in ('', 'run', 'r'):
                    print("Prompt received — executing mission.")
                    drone.execute_mission()
                    break
                elif line in ('q', 'quit', 'exit'):
                    print("Exiting.")
                    break
    except KeyboardInterrupt:
        print("Interrupted. Exiting.")
