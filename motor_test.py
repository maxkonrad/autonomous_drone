import serial
import struct
import time

# MSP Command Codes
MSP_SET_RAW_RC = 200

# Macros
SECONDS = 10  # Duration for motor test in seconds

class MotorTest:
    def __init__(self, port='/dev/ttyAMA0'):  # Adjust port as needed for your setup
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        # Default channels: Roll, Pitch, Yaw, Throttle, Aux1, Aux2, Aux3, Aux4
        self.channels = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]

    def send_msp(self, code, payload):
        size = len(payload)
        header = b'$M<'
        checksum = size ^ code
        for b in payload:
            checksum ^= b

        packet = header + struct.pack('<BB', size, code) + payload + struct.pack('<B', checksum)
        self.ser.write(packet)

    def update_rc(self):
        """Send RC channels to maintain failsafe"""
        payload = struct.pack('<8H', *self.channels)
        self.send_msp(MSP_SET_RAW_RC, payload)

    def arm(self):
        print("Arming drone...")
        self.channels[4] = 2000  # AUX1 to arm position
        self.channels[3] = 1000  # Throttle at minimum
        for _ in range(10):
            self.update_rc()
            time.sleep(0.1)
        print("Armed.")

    def test_motors(self):
        print("Testing motors at 10% throttle for 1 second...")
        # 10% throttle: 1000 + 0.1*(2000-1000) = 1100
        self.channels[3] = 1100
        self.update_rc()
        time.sleep(SECONDS)
        # Stop motors
        self.channels[3] = 1000
        self.update_rc()
        print("Motor test complete.")

    def close(self):
        self.ser.close()

if __name__ == '__main__':
    tester = MotorTest()
    try:
        tester.arm()
        time.sleep(1)
        tester.test_motors()
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        tester.close()