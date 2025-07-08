# Protocolo de Comunicación ESP32 ↔ Raspberry Pi

## Configuración de Hardware

### Conexiones UART
- **ESP32 UART2**: RX=GPIO22, TX=GPIO21
- **Raspberry Pi**: Conectar a pines TX/RX correspondientes
- **Velocidad**: 115200 bps, 8N1

### Sensores en ESP32
- **Corriente Motor Izquierdo**: GPIO36 (VP)
- **Corriente Motor Derecho**: GPIO39 (VN) 
- **Voltaje Batería**: GPIO34 (con divisor de voltaje 39kΩ/12kΩ para batería 12V)

## Protocolo de Comunicación

### Formato: JSON sobre UART

---

## 📤 ESP32 → Raspberry Pi

### 1. Datos del Sistema (automático cada 1 segundo)
```json
{
  "type": "system_data",
  "timestamp": 12345,
  "uwb": {
    "valid": true,
    "x": 1.25,
    "y": 2.16
  },
  "sensors": {
    "current_left": 0.85,
    "current_right": 0.92,
    "battery_voltage": 11.8
  }
}
```

### 2. Heartbeat (cada 5 segundos)
```json
{
  "type": "heartbeat",
  "timestamp": 12345,
  "uptime": 12345,
  "free_heap": 234567
}
```

### 3. Respuesta a Ping
```json
{
  "type": "pong",
  "timestamp": 12345
}
```

### 4. Confirmación de Parada de Emergencia
```json
{
  "type": "emergency_stop_ack",
  "timestamp": 12345
}
```

---

## 📥 Raspberry Pi → ESP32

### 1. Comando de Motor
```json
{
  "type": "motor_command",
  "left_speed": 150,
  "right_speed": -100,
  "emergency_stop": false
}
```
- **left_speed**: -255 a +255 (negativo = reversa)
- **right_speed**: -255 a +255 (negativo = reversa)
- **emergency_stop**: true para parada inmediata

### 2. Parada de Emergencia
```json
{
  "type": "emergency_stop"
}
```

### 3. Ping (verificar conectividad)
```json
{
  "type": "ping"
}
```

---

## Características de Seguridad

### Timeout de Comunicación
- Si no se reciben comandos por **2 segundos**, los motores se detienen automáticamente
- El ESP32 envía mensaje de timeout a la RPi

### Parada de Emergencia
- Cualquier comando con `"emergency_stop": true` detiene inmediatamente todos los motores
- Los motores permanecen deshabilitados hasta recibir un nuevo comando válido

### Límites de Velocidad
- Las velocidades se limitan automáticamente al rango válido (-255 a +255)
- Mapeo interno: -255/+255 → -100%/+100% PWM

---

## Ejemplo de Uso desde Raspberry Pi (Python)

```python
import serial
import json
import time

# Configurar conexión serial
esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Función para enviar comando de motor
def send_motor_command(left_speed, right_speed):
    command = {
        "type": "motor_command",
        "left_speed": left_speed,
        "right_speed": right_speed,
        "emergency_stop": False
    }
    esp32.write((json.dumps(command) + '\n').encode())

# Función para leer datos del sistema
def read_system_data():
    if esp32.in_waiting:
        line = esp32.readline().decode().strip()
        try:
            data = json.loads(line)
            return data
        except:
            return None
    return None

# Ejemplo de uso
while True:
    # Enviar comando de movimiento
    send_motor_command(100, 100)  # Adelante
    
    # Leer datos UWB
    data = read_system_data()
    if data and data['type'] == 'system_data':
        uwb = data['uwb']
        if uwb['valid']:
            print(f"Posición: X={uwb['x']:.2f}, Y={uwb['y']:.2f}")
    
    time.sleep(0.1)
```

---

## Notas Técnicas

### Configuración ADC
- **Resolución**: 12-bit (0-4095)
- **Atenuación**: 11dB para voltajes hasta 3.3V
- **Promediado**: Múltiples lecturas para mayor precisión

### Divisor de Voltaje para Batería
- **R1**: 39kΩ (hacia VIN)
- **R2**: 12kΩ (hacia GND)
- **Factor**: (39k + 12k) / 12k = 4.25
- **Rango**: 0V - 14V (para batería 12V nominal)

### Sensor de Corriente
- Compatible con sensores tipo ACS712
- Configuración ejemplo para ACS712-5A: 2.5V = 0A, 185mV/A
- Ajustar calibración según sensor específico utilizado
