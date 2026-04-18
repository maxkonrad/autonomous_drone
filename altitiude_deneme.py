import serial
import struct
import time

class MSPAltitude:
    """Raw MSP (MultiWii Serial Protocol) implementation for altitude reading"""
    
    # MSP Command IDs
    MSP_ALTITUDE = 109
    
    def __init__(self, port, baudrate=115200, timeout=0.1):
        """Initialize serial connection"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connect()
    
    def connect(self):
        """Open serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)  # Wait for board to wake up
        except Exception as e:
            print(f"Error opening serial port: {e}")
            raise
    
    def disconnect(self):
        """Close serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")
    
    def send_msp_command(self, cmd_id, data=None):
        """
        Send MSP command to board
        
        MSP Message Format:
        $M< dataLength cmdID [data] checksum
        
        Args:
            cmd_id: Command ID
            data: Optional data bytes (list)
        
        Returns:
            True if sent successfully
        """
        if data is None:
            data = []
        
        # Build message
        data_length = len(data)
        message = [cmd_id] + data
        
        # Calculate checksum (XOR of dataLength, cmdID, and all data)
        checksum = data_length ^ cmd_id
        for byte in data:
            checksum ^= byte
        
        # Build complete packet: $M< dataLength cmdID data checksum
        packet = b'$M<' + bytes([data_length, cmd_id]) + bytes(data) + bytes([checksum])
        
        try:
            self.ser.write(packet)
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def receive_msp_response(self, expected_cmd_id=None, max_attempts=5):
        """
        Receive MSP response from board
        
        MSP Response Format:
        $M> dataLength cmdID [data] checksum
        
        Args:
            expected_cmd_id: Expected command ID (optional)
            max_attempts: Max attempts to read response
        
        Returns:
            Tuple (cmd_id, data_bytes) or (None, None) if error
        """
        attempts = 0
        while attempts < max_attempts:
            try:
                # Wait for header
                header = b''
                while len(header) < 3:
                    byte = self.ser.read(1)
                    if not byte:
                        attempts += 1
                        break
                    header += byte
                
                if len(header) < 3:
                    continue
                
                # Check header
                if header != b'$M>':
                    # Skip this byte and try again
                    header = header[1:] + self.ser.read(1)
                    continue
                
                # Read data length and command ID
                length_cmd = self.ser.read(2)
                if len(length_cmd) < 2:
                    attempts += 1
                    continue
                
                data_length = length_cmd[0]
                cmd_id = length_cmd[1]
                
                # Check if this is expected response
                if expected_cmd_id is not None and cmd_id != expected_cmd_id:
                    # Wrong response, skip
                    self.ser.read(data_length + 1)  # Skip data and checksum
                    attempts += 1
                    continue
                
                # Read data
                data = self.ser.read(data_length)
                if len(data) < data_length:
                    attempts += 1
                    continue
                
                # Read checksum
                checksum = self.ser.read(1)
                if not checksum:
                    attempts += 1
                    continue
                
                # Verify checksum
                calculated_checksum = data_length ^ cmd_id
                for byte in data:
                    calculated_checksum ^= byte
                
                if calculated_checksum != checksum[0]:
                    print(f"Checksum mismatch! Expected {calculated_checksum}, got {checksum[0]}")
                    attempts += 1
                    continue
                
                return (cmd_id, data)
            
            except Exception as e:
                print(f"Error receiving response: {e}")
                attempts += 1
        
        return (None, None)
    
    def get_altitude(self):
        """
        Get altitude from board
        
        Returns:
            Dict with 'altitude' (cm) and 'vario' (cm/s), or None if error
        """
        # Send request
        if not self.send_msp_command(self.MSP_ALTITUDE):
            return None
        
        # Receive response
        cmd_id, data = self.receive_msp_response(expected_cmd_id=self.MSP_ALTITUDE)
        
        if cmd_id is None or len(data) < 4:
            return None
        
        # Parse altitude data (2 signed shorts: altitude, vario)
        altitude_cm, vario_cms = struct.unpack('<hh', data[:4])
        
        return {
            'altitude_cm': altitude_cm,
            'altitude_m': altitude_cm / 100.0,
            'vario_cms': vario_cms,
            'vario_ms': vario_cms / 100.0,
            'timestamp': time.time()
        }


def main():
    """Main function to continuously read altitude"""
    
    # Initialize MSP
    msp = MSPAltitude("COM3")  # Adjust port as needed
    
    try:
        print("\nReading altitude constantly from raw MSP...")
        print("-" * 90)
        print(f"{'Timestamp':<20} {'Altitude (m)':<15} {'Vertical Vel (m/s)':<20} {'Raw Alt (cm)':<15}")
        print("-" * 90)
        
        while True:
            # Get altitude data
            alt_data = msp.get_altitude()
            
            if alt_data:
                timestamp = alt_data['timestamp']
                altitude_m = alt_data['altitude_m']
                vario_ms = alt_data['vario_ms']
                altitude_cm = alt_data['altitude_cm']
                
                print(f"{timestamp:<20.2f} {altitude_m:<15.2f} {vario_ms:<20.2f} {altitude_cm:<15}")
            else:
                print("Failed to read altitude")
            
            # Sampling rate: 10 Hz
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        msp.disconnect()


if __name__ == "__main__":
    main()
