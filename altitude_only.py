import serial
import struct
import time

# MSP Command Codes
MSP_ALTITUDE = 109  # Important for feedback

class AltitudeMonitor:
    def __init__(self, port='\/dev\/ttyACM0'):
        # open serial at 115200 baud, short timeout so reads don't block
        self.ser = serial.Serial(port, 115200, timeout=0.1)

    def get_altitude(self):
        """Requests and parses MSP_ALTITUDE (Returns alt in cm)"""
        # Header + Size(0) + Command(109) + Checksum(109)
        cmd = b'$M<\x00\x6d\x6d'
        self.ser.write(cmd)
        
        header = self.ser.read(3)
        if header == b'$M>':
            size = ord(self.ser.read(1))
            code = ord(self.ser.read(1))
            payload = self.ser.read(size)
            crc = self.ser.read(1)
            
            if code == MSP_ALTITUDE and len(payload) >= 4:
                alt_cm = struct.unpack('<i', payload[0:4])[0]
                return alt_cm
        return None

if __name__ == '__main__':
    monitor = AltitudeMonitor(port='/dev/ttyUSB0')
    print('Starting altitude monitor on /dev/ttyUSB0. Ctrl-C to exit.')
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
