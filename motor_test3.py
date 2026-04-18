import serial
import struct
import time

# Raspberry Pi'de uçuş kontrolcüsünün bağlı olduğu port (Genelde /dev/ttyACM0 veya /dev/ttyAMA0)
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

# MSP Komutları
MSP_SET_RAW_MOTORS = 214

def send_msp_command(ser, cmd, data):
    size = len(data)
    checksum = size ^ cmd
    for b in data:
        checksum ^= b
    
    # THE FIX: Changed '<3sBBB' to '<3sBB' to match the 3 variables perfectly
    # MSP Frame Yapısı: $ M < (yön) [size] [cmd] [payload] [checksum]
    header = struct.pack('<3sBB', b'$M<', size, cmd)
    ser.write(header + bytes(data) + struct.pack('<B', checksum))

def set_motors(ser, m1, m2, m3, m4):
    # Motor değerleri genellikle 1000-2000 arasıdır. 
    # Test için düşük değer (örn: 1100) kullanın.
    data = struct.pack('<HHHH', m1, m2, m3, m4)
    # Diğer 4 motor kanalını (toplam 8 kanal için) 1000 (kapalı) olarak dolduruyoruz
    data += struct.pack('<HHHH', 1000, 1000, 1000, 1000)
    send_msp_command(ser, MSP_SET_RAW_MOTORS, list(data))

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Bağlantı kuruldu: {SERIAL_PORT}")
    time.sleep(2) # Başlatma için bekle

    print("Motorlar test ediliyor (Düşük Devir)...")
    
    # 2 saniye boyunca tüm motorları 1100 seviyesinde çalıştır
    # DİKKAT: Pervanelerin ÇIKARILMIŞ olduğundan emin olun!
    start_time = time.time()
    while time.time() - start_time < 2:
        set_motors(ser, 1100, 1100, 1100, 1100)
        time.sleep(0.05) # MSP stabilite için periyodik gönderim gerekir

    # Motorları durdur
    set_motors(ser, 1000, 1000, 1000, 1000)
    print("Test tamamlandı, motorlar durduruldu.")

except Exception as e:
    print(f"Hata: {e}")
finally:
    if 'ser' in locals():
        ser.close()