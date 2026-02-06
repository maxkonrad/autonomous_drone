import serial
import struct
import time

class INavControl:
    def __init__(self, port='/dev/ttyS0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        # Başlangıç değerleri (AETR: Roll, Pitch, Yaw, Throttle, Aux1, Aux2...)
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
        """iNAV Failsafe'e girmemesi için bu sürekli (döngüde) çağrılmalıdır."""
        payload = struct.pack('<8H', *self.channels)
        self.send_msp(200, payload)

    def arm(self):
        print("Arming...")
        # iNAV'da AUX1'in 1800-2100 arası ARM olarak ayarlandığını varsayıyoruz.
        self.channels[4] = 2000 
        self.channels[3] = 1000 # Throttle en altta olmalı
        # iNAV'ın Arm sinyalini algılaması için birkaç kez gönder
        for _ in range(10):
            self.update_rc()
            time.sleep(0.1)

    def takeoff_to_2_meters(self):
        """
        NOT: iNAV'da gerçek bir '2 metre' komutu için GPS/Barometre geri beslemesi gerekir.
        Burada basitçe Throttle artırarak yükselme mantığı kurulmuştur.
        """
        if self.channels[4] < 1800:
            print("Önce ARM etmelisiniz!")
            return

        print("Yükseliyor...")
        # 1. Aşama: Motorları döndür (hover öncesi hazırlık)
        self.channels[3] = 1300 
        
        # 2. Aşama: Belirli bir süre gaz ver (Deneyerek optimize edilmeli)
        # İdeal olan: MSP_RAW_IMU veya MSP_ALTITUDE okuyarak 2 metrede durmaktır.
        for _ in range(20): # Yaklaşık 2 saniye
            self.update_rc()
            time.sleep(0.1)
            
        # 3. Aşama: Hover (Askıda kalma) seviyesine çek
        self.channels[3] = 1500 
        self.update_rc()

# Kullanım:
drone = INavControl()
drone.arm()
time.sleep(1)
drone.takeoff_to_2_meters()
# MSP V2 Message Structure
# Offset |    Usage     | CRC | Comment
#   0    |      $       | No  | Same lead-in as V1
#   1    |      X       | No  | 'X' in place of 'M' in V1
#   2    |     type     | No  | '<' = request, '>' = response, '!' = error 
#   3    |     flag     | Yes | uint8, usually 0, see https://github.com/iNavFlight/inav/wiki/MSP-V2#message-flags
#   4    |   function   | Yes | uint16, https://github.com/iNavFlight/inav/tree/master/src/main/msp (uint16 = 2 bytes)
#   6    | payload size | Yes | uint16 (little endian) payload size in bytes (uint16 = 2 bytes)
#   8    |   payload    | Yes | n (up to 65535 bytes) payload 
#  n+8   |   checksum   | No  | uint8, (n = payload size), crc8_dvb_s2 checksum

