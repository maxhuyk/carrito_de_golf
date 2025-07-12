#!/usr/bin/env python3
"""
Script súper simple para probar comunicación UART
Solo lee datos del puerto serial y los muestra
"""

import serial
import time

def test_uart():
    """Prueba básica de UART"""
    
    # Lista de puertos para probar
    ports_to_try = [
        '/dev/ttyAMA0',    # GPIO UART
        '/dev/ttyS0',      # Alternativo
        '/dev/serial0',    # Enlace simbólico
        '/dev/ttyUSB0',    # USB (si usas adaptador)
    ]
    
    baudrates = [2000000, 115200, 9600]  # Velocidades a probar
    
    for port in ports_to_try:
        for baudrate in baudrates:
            try:
                print(f"\n🔍 Probando {port} a {baudrate} baudios...")
                
                # Intentar abrir puerto
                ser = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=2.0
                )
                
                print(f"✅ Puerto {port} abierto exitosamente!")
                print("📡 Escuchando datos (presiona Ctrl+C para parar)...")
                
                # Leer datos por 10 segundos
                start_time = time.time()
                data_count = 0
                
                while time.time() - start_time < 10:
                    try:
                        if ser.in_waiting > 0:
                            # Leer línea
                            line = ser.readline()
                            data_count += 1
                            
                            try:
                                # Intentar decodificar como texto
                                text = line.decode('utf-8').strip()
                                print(f"📨 [{data_count}] {text}")
                            except:
                                # Si no es texto, mostrar como bytes
                                print(f"📨 [{data_count}] BYTES: {line}")
                        
                        time.sleep(0.01)
                        
                    except KeyboardInterrupt:
                        break
                
                ser.close()
                
                if data_count > 0:
                    print(f"🎯 ÉXITO: Recibidos {data_count} mensajes en {port} a {baudrate} baudios")
                    return port, baudrate
                else:
                    print(f"⚠️ No se recibieron datos en {port} a {baudrate} baudios")
                
            except serial.SerialException as e:
                print(f"❌ No se pudo abrir {port}: {e}")
            except Exception as e:
                print(f"❌ Error con {port}: {e}")
    
    print("\n❌ No se pudo establecer comunicación en ningún puerto")
    return None, None

def list_serial_ports():
    """Lista todos los puertos serie disponibles"""
    import glob
    
    print("🔍 Puertos serie disponibles:")
    
    patterns = [
        '/dev/tty[A-Za-z]*',
        '/dev/serial*'
    ]
    
    found_ports = []
    for pattern in patterns:
        ports = glob.glob(pattern)
        found_ports.extend(ports)
    
    if found_ports:
        for port in sorted(set(found_ports)):
            print(f"   📍 {port}")
    else:
        print("   ❌ No se encontraron puertos serie")
    
    return found_ports

if __name__ == "__main__":
    print("🚗 Test UART ESP32 - Carrito de Golf")
    print("=" * 40)
    
    # Listar puertos disponibles
    list_serial_ports()
    
    # Probar comunicación
    port, baudrate = test_uart()
    
    if port:
        print(f"\n🎉 Puerto correcto: {port} a {baudrate} baudios")
        print("💡 Actualiza tu config.json con estos valores:")
        print(f'   "uart_port": "{port}",')
        print(f'   "uart_baudrate": {baudrate}')
    else:
        print("\n🔧 Revisa:")
        print("   1. ¿Está conectado el ESP32?")
        print("   2. ¿Está habilitado UART en Raspberry Pi?")
        print("   3. ¿Están correctas las conexiones GPIO?")
        print("   4. ¿Está funcionando el ESP32?")
