import serial
import struct
import time

# MSP Command Codes
MSP_ALTITUDE = 109
MSP_SET_RAW_RC = 200  # Required to send control commands

class AltitudeMonitor:
    def __init__(self, port='/dev/ttyAMA0'):
        # Open serial at 115200 baud, short timeout so reads don't block
        self.ser = serial.Serial(port, 115200, timeout=0.1)

    def send_command(self, code, payload=b''):
        """Constructs and sends an MSP command with the correct header and CRC."""
        size = len(payload)
        checksum = size ^ code
        for b in payload:
            checksum ^= b
            
        cmd = b'$M<' + bytes([size, code]) + payload + bytes([checksum])
        self.ser.write(cmd)

    def trigger_landing_mode(self):
        """
        Sends RC commands to trigger landing. 
        Format is 8 unsigned 16-bit integers (Roll, Pitch, Yaw, Throttle, Aux1, Aux2, Aux3, Aux4).
        Values typically range from 1000 to 2000. 1500 is neutral.
        """
        # Neutral roll/pitch/yaw, mid-throttle. 
        # AUX1 is set to 2000 (High) to trigger the pre-configured 'Land' mode on the FC.
        payload = struct.pack('<8H', 1500, 1500, 1500, 1500, 2000, 1000, 1000, 1000)
        self.send_command(MSP_SET_RAW_RC, payload)

    def get_altitude(self):
        """Requests and parses MSP_ALTITUDE (Returns alt in cm)"""
        self.send_command(MSP_ALTITUDE)

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

        if code == MSP_ALTITUDE and size >= 4:
            # Note: Standard MSP_ALTITUDE uses the first 4 bytes (0:4) for EstAlt (int32).
            # Your original code used [6:10]. I have updated it to the standard [0:4].
            # If you are using a custom firmware that shifts this payload, revert to [6:10].
            alt_cm = struct.unpack('<i', payload[0:4])[0]
            return alt_cm

        return None

if __name__ == '__main__':
    monitor = AltitudeMonitor(port='/dev/ttyAMA0')
    print('Starting flight monitor on /dev/ttyAMA0. Ctrl-C to exit.')
    
    target_altitude_cm = 200
    landing_initiated = False

    try:
        while True:
            if not landing_initiated:
                # Phase 1: Monitor Altitude
                alt = monitor.get_altitude()
                if alt is not None:
                    print(f'Altitude: {alt} cm')
                    
                    if alt >= target_altitude_cm:
                        print(f'\nTarget altitude of {target_altitude_cm}cm reached! Initiating autonomous landing...')
                        landing_initiated = True
                else:
                    print('No altitude data')
                
                time.sleep(0.1) # Faster poll rate for flight dynamics
                
            else:
                # Phase 2: Landing Control Loop
                # FCs require continuous RC signal updates (e.g., 20-50Hz) to prevent RX Failsafe.
                monitor.trigger_landing_mode()
                print('Landing mode active...', end='\r')
                time.sleep(0.05) 

    except KeyboardInterrupt:
        print('\nExiting.')
        monitor.ser.close()