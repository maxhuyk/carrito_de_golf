# Protocolo de Comunicación ESP32 ↔ Raspberry Pi

## Configuración Hardware
- **UART2 del ESP32**: RX=GPIO16, TX=GPIO17
- **Baudrate**: 115200 bps
- **Formato**: 8N1 (8 bits, sin paridad, 1 stop bit)

## Arquitectura del Sistema
```
┌─────────────────┐    UART2     ┌─────────────────┐
│   Raspberry Pi  │◄────────────►│      ESP32      │
│                 │              │                 │
│ - Procesamiento │              │ - UWB Anchors   │
│ - Lógica        │              │ - Motor Control │
│ - Coordenadas   │              │ - Sensores      │
│ - Algoritmos    │              │                 │
└─────────────────┘              └─────────────────┘
```

## Comandos (RPi → ESP32)

### Control de Motores
```
MOTOR:L:<speed>    # Motor izquierdo (-100 a +100)
MOTOR:R:<speed>    # Motor derecho (-100 a +100)  
MOTOR:B:<speed>    # Ambos motores (-100 a +100)
```
**Ejemplos:**
- `MOTOR:L:50`     → Motor izquierdo 50% adelante
- `MOTOR:R:-30`    → Motor derecho 30% atrás
- `MOTOR:B:0`      → Ambos motores parar

### Control del Sistema
```
ENABLE             # Activar motores
DISABLE            # Desactivar motores
STOP               # Parada de emergencia
STATUS             # Solicitar estado actual
UWB                # Solicitar datos UWB instantáneos
```

## Respuestas (ESP32 → RPi)

### Confirmaciones de Comandos
```
OK:MOTOR_L:<speed>         # Motor izquierdo configurado
OK:MOTOR_R:<speed>         # Motor derecho configurado
OK:MOTOR_BOTH:<speed>      # Ambos motores configurados
OK:MOTORS_ENABLED          # Motores activados
OK:MOTORS_DISABLED         # Motores desactivados
OK:EMERGENCY_STOP          # Parada de emergencia ejecutada
```

### Estado del Sistema
```
STATUS:ENABLED,L:<speed>,R:<speed>,UPTIME:<ms>
STATUS:DISABLED,L:<speed>,R:<speed>,UPTIME:<ms>
```
**Ejemplo:**
`STATUS:ENABLED,L:50,R:-30,UPTIME:125430`

### Datos UWB (Automáticos cada 1s)
```
UWB:VALID,X:<x>,Y:<y>,T:<timestamp>      # Posición válida
UWB:INVALID,X:0.000,Y:0.000,T:<timestamp> # Sin posición
```
**Ejemplos:**
- `UWB:VALID,X:1.250,Y:2.340,T:125430`
- `UWB:INVALID,X:0.000,Y:0.000,T:125431`

### Errores
```
ERROR:UNKNOWN_COMMAND      # Comando no reconocido
ERROR:INVALID_MOTOR_FORMAT # Formato de comando motor incorrecto
ERROR:INVALID_MOTOR_ID     # ID de motor inválido (no L/R/B)
ERROR:MOTORS_DISABLED      # Intentar mover motores desactivados
ERROR:BUFFER_OVERFLOW      # Comando demasiado largo
STATUS:TIMEOUT_STOP        # Timeout de comunicación (2s)
```

## Ejemplo de Sesión Completa

```
# RPi conecta y activa sistema
RPi: ENABLE
ESP: OK:MOTORS_ENABLED

# RPi solicita estado
RPi: STATUS  
ESP: STATUS:ENABLED,L:0,R:0,UPTIME:5240

# RPi mueve carrito adelante
RPi: MOTOR:B:50
ESP: OK:MOTOR_BOTH:50

# ESP envía datos UWB automáticamente
ESP: UWB:VALID,X:1.250,Y:2.340,T:6240

# RPi gira a la derecha
RPi: MOTOR:L:80
ESP: OK:MOTOR_L:80
RPi: MOTOR:R:20  
ESP: OK:MOTOR_R:20

# RPi para el carrito
RPi: MOTOR:B:0
ESP: OK:MOTOR_BOTH:0

# Parada de emergencia
RPi: STOP
ESP: OK:EMERGENCY_STOP
```

## Características de Seguridad

1. **Timeout**: Si no hay comunicación por 2 segundos, parada automática
2. **Estado inicial**: Motores desactivados al inicio
3. **Parada de emergencia**: Comando `STOP` detiene todo inmediatamente
4. **Validación**: Comandos mal formados generan errores
5. **Límites**: Velocidades limitadas a -100/+100

## Implementación en Raspberry Pi (Python)

```python
import serial
import time

class ESP32Controller:
    def __init__(self, port='/dev/ttyUSB0'):
        self.ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Esperar inicialización
        
    def enable_motors(self):
        self.ser.write(b'ENABLE\n')
        return self.ser.readline().decode().strip()
        
    def set_motor_speed(self, motor, speed):
        cmd = f'MOTOR:{motor}:{speed}\n'
        self.ser.write(cmd.encode())
        return self.ser.readline().decode().strip()
        
    def emergency_stop(self):
        self.ser.write(b'STOP\n')
        return self.ser.readline().decode().strip()
        
    def get_uwb_data(self):
        self.ser.write(b'UWB\n')
        response = self.ser.readline().decode().strip()
        # Parse: UWB:VALID,X:1.250,Y:2.340,T:125430
        return self.parse_uwb_response(response)
```

Este protocolo permite un control completo y seguro del carrito desde la Raspberry Pi.
