#!/usr/bin/env python3
"""
Test UART JSON - Script simple para verificar el formato del JSON
"""

import serial
import json
import time

def test_uart_json():
    """Test simple para verificar formato JSON desde ESP32"""
    
    try:
        # Conectar al puerto serie
        ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        print("Conectado al puerto serie...")
        
        # Leer algunos mensajes
        for i in range(10):
            print(f"\n--- Mensaje {i+1} ---")
            
            # Leer línea raw
            line = ser.readline().decode('utf-8').strip()
            print(f"RAW: {repr(line)}")
            
            if line:
                try:
                    # Intentar parsear JSON
                    data = json.loads(line)
                    print(f"JSON OK: {data}")
                    
                    # Verificar campos específicos
                    if 'uwb' in data:
                        uwb = data['uwb']
                        print(f"UWB distances: d1={uwb.get('d1')}, d2={uwb.get('d2')}, d3={uwb.get('d3')}")
                        print(f"UWB status: s1={uwb.get('s1')}, s2={uwb.get('s2')}, s3={uwb.get('s3')}")
                        print(f"UWB freq={uwb.get('freq')}, count={uwb.get('count')}")
                    
                    if 'power' in data:
                        power = data['power']
                        print(f"Power: battery={power.get('battery_v')}V, motors={power.get('motor_l_a')}A/{power.get('motor_r_a')}A")
                    
                except json.JSONDecodeError as e:
                    print(f"JSON ERROR: {e}")
                    print(f"Problematic line: {repr(line)}")
                except Exception as e:
                    print(f"OTHER ERROR: {e}")
            else:
                print("No data received")
            
            time.sleep(1)
        
        ser.close()
        print("\nTest completado")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_uart_json()
