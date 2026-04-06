import serial
import struct
import time

# MSP Command Codes
MSP_ALTITUDE = 109  # Important for feedback

class AltitudeMonitor:
    def __init__(self, port='/dev/ttyAMA0'):
        # open serial at 115200 baud, short timeout so reads don't block
        self.ser = serial.Serial(port, 115200, timeout=0.1)

    def get_altitude(self):
        """Requests and parses MSP_ALTITUDE (Returns alt in cm)"""

        cmd = b'$M<\x00\x6d\x6d'
        self.ser.write(cmd)

        header = self.ser.read(3)
        if header != b'$M>':
            return None

        size_b = self.ser.read(1)
        code_b = self.ser.read(1)
        if not size_b or not code_b:
            return None

        size = size_b[0]
        code = code_b[0]

        payload = self.ser.read(size)
        if len(payload) != size:
            return None
        print(f"Received payload: {payload.hex()}")
        crc_b = self.ser.read(1)
        if not crc_b:
            return None
        crc = crc_b[0]

        # CRC check
        calc_crc = size ^ code
        for b in payload:
            calc_crc ^= b

        if crc != calc_crc:
            return None

        if code == MSP_ALTITUDE and size >= 6:
            alt_cm = struct.unpack('<i', payload[6:10])[0]
            return alt_cm

        return None

if __name__ == '__main__':
    monitor = AltitudeMonitor(port='/dev/ttyAMA0')
    print('Starting altitude monitor on /dev/ttyAMA0. Ctrl-C to exit.')
    try:
        while True:
            alt = monitor.get_altitude()
            if alt is not None:
                print(f'Altitude: {alt} cm')
            else:
                print('No altitude data')
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('Exiting.')
        monitor.ser.close()
