#!/usr/bin/env python3
"""
Script simple para resetear ESP32
Uso: python reset_esp32.py [puerto]
"""

import sys
import time
import subprocess

def reset_esp32(port="COM9"):
    """Reset ESP32 usando esptool"""
    print(f"🔄 Reseteando ESP32 en {port}...")
    
    try:
        # Comando para resetear usando esptool
        cmd = [sys.executable, "-m", "esptool", "--port", port, "run"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("✅ Reset enviado correctamente")
            return True
        else:
            print(f"❌ Error: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("⏰ Timeout - El dispositivo no responde")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def check_bootloader(port="COM9"):
    """Verificar si ESP32 está en modo bootloader"""
    print(f"🔍 Verificando modo bootloader en {port}...")
    
    try:
        cmd = [sys.executable, "-m", "esptool", "--port", port, "chip_id"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        
        if result.returncode == 0 and "Chip is" in result.stdout:
            print("✅ ESP32 detectado en modo bootloader")
            print(result.stdout.split('\n')[0])  # Primera línea con info del chip
            return True
        else:
            print("❌ ESP32 no está en modo bootloader")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    
    print("=" * 50)
    print("🔧 HERRAMIENTA DE RESET ESP32")
    print("=" * 50)
    
    # Verificar estado actual
    if check_bootloader(port):
        print("💡 El ESP32 ya está en modo bootloader")
    else:
        print("💡 Intentando resetear...")
        reset_esp32(port)
        time.sleep(2)
        check_bootloader(port)
    
    print("\n📝 Comandos útiles:")
    print(f"   Reset: python {__file__} {port}")
    print(f"   Monitor: pio device monitor --port {port}")
    print(f"   Upload: pio run -t upload")
    print("\n💡 Si sigue sin funcionar:")
    print("   1. Desconecta USB completamente")
    print("   2. Mantén presionado botón BOOT")
    print("   3. Conecta USB")
    print("   4. Suelta BOOT después de 2 segundos")

if __name__ == "__main__":
    main()
