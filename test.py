import serial
import time
import struct

# --- Configuration ---
# /dev/serial0 is the default primary UART on Raspberry Pi
SERIAL_PORT = '/dev/ttyAMA0' 
# This must perfectly match the baud rate set in the INAV Ports tab
BAUD_RATE = 115200           

# MSP Command IDs
MSP_API_VERSION = 1

def create_msp_request(command):
    """
    Constructs an MSP V1 request frame.
    Format: Header ($M<) | Size (0) | Command | Checksum (Size XOR Command)
    """
    header = b'$M<'
    size = 0
    checksum = size ^ command
    
    # Pack into bytes: 3 chars, uint8, uint8, uint8
    return struct.pack('<3sBBB', header, size, command, checksum)

def test_uart_connection():
    try:
        # Open serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Port opened: {SERIAL_PORT} at {BAUD_RATE} baud.")
        
        # Allow the connection to initialize
        time.sleep(0.1)

        # Send the MSP request
        print("Sending MSP_API_VERSION request...")
        ser.write(create_msp_request(MSP_API_VERSION))

        # Read the 3-byte response header
        header = ser.read(3)
        
        if header == b'$M>':
            # Unpack Size and Command ID
            size = struct.unpack('<B', ser.read(1))[0]
            cmd = struct.unpack('<B', ser.read(1))[0]
            
            # Read Payload and Checksum
            payload = ser.read(size)
            checksum = struct.unpack('<B', ser.read(1))[0]

            print("\n[SUCCESS] Valid MSP response received.")
            print(f"Command ID: {cmd}")
            print(f"Payload Size: {size} bytes")
            
            # Parse the specific payload for MSP_API_VERSION (3 bytes)
            if size == 3:
                protocol, major, minor = struct.unpack('<BBB', payload)
                print(f"MSP Protocol: {protocol}")
                print(f"INAV API Version: {major}.{minor}")
        else:
            print("\n[FAILURE] No valid MSP header received.")
            print(f"Raw data received: {header}")

    except serial.SerialException as e:
        print(f"\n[ERROR] Serial exception: {e}")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
    finally:
        # Ensure the port is released back to the OS
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == '__main__':
    test_uart_connection()