import serial
import struct

def send_msp_set_motor(serial_connection, motor_values):
    """
    Uçuş kontrolcüsüne MSP_SET_MOTOR (214) paketini gönderir.
    
    :param serial_connection: Açık bir pyserial nesnesi
    :param motor_values: 4 adet motor değeri içeren liste (örn: [1500, 1500, 1500, 1500])
    """
    # 1. Başlık ve Yön: $ M <
    header = b'$M<'
    
    # 2. Boyut: 4 motor * 2 bayt (16-bit) = 8 bayt
    size = 8
    
    # 3. Komut ID
    command = 214
    
    # 4. Payload (Veri) Oluşturma
    # '<4H' : Little-Endian formatında 4 adet Unsigned Short (16-bit)
    payload = struct.pack('<4H', *motor_values)
    
    # 5. Checksum (Sağlama) Hesaplama
    # Checksum = Boyut XOR Komut XOR Payload_Baytları
    checksum = size ^ command
    for byte in payload:
        checksum ^= byte
        
    # 6. Paketi Birleştirme
    # Python'da tekil sayıları bayta çevirmek için bytes([değer]) kullanılır
    packet = header + bytes([size, command]) + payload + bytes([checksum])
    
    # Paketi seri porta yaz
    serial_connection.write(packet)
    
    return packet

# --- Kullanım Örneği ---
if __name__ == '__main__':
    # Bağlantı noktası donanımınıza göre değişebilir (örn: /dev/ttyUSB0, /dev/ttyACM0)
    PORT = '/dev/ttyUSB0' 
    BAUDRATE = 115200
    
    # Gönderilecek motor sinyalleri (1000 kapalı, 2000 tam güç)
    # Sıralama genellikle QuadX için: [Sağ Arka, Sağ Ön, Sol Arka, Sol Ön] şeklindedir
    motors = [1100, 1100, 1100, 1100] 

    try:
        # Seri portu aç
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"{PORT} portuna bağlanıldı.")
            
            # Fonksiyonu çağır ve paketi gönder
            sent_packet = send_msp_set_motor(ser, motors)
            
            # Gönderilen ham bayt dizilimini Hex formatında gör
            hex_packet = ' '.join([f"{b:02X}" for b in sent_packet])
            print(f"Gönderilen Paket (Hex): {hex_packet}")
            
    except serial.SerialException as e:
        print(f"Seri port hatası: {e}")