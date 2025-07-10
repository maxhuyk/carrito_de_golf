# Golf Cart Control System - Raspberry Pi
## Sistema de Control para Carrito de Golf UWB

Este directorio contiene los scripts de Python que corren en la Raspberry Pi para:

### 📡 **Funciones Principales:**
- **Trilateración UWB:** Calcular posición del carrito usando datos raw del ESP32
- **Control de Motores:** Enviar comandos de velocidad al ESP32
- **Filtrado Kalman:** Suavizar y predecir posición
- **Navegación:** Algoritmos de path planning y control
- **Monitoreo:** Logging y telemetría del sistema

### 📁 **Estructura de Directorios:**
```
RaspberryPi/
├── src/                    # Scripts principales de Python
│   ├── main.py            # Script principal del sistema
│   ├── uwb_processor.py   # Procesamiento de datos UWB
│   ├── trilateration.py   # Algoritmos de trilateración
│   ├── motor_control.py   # Control de motores ESP32
│   ├── kalman_filter.py   # Filtro Kalman para posición
│   ├── navigation.py      # Algoritmos de navegación
│   └── uart_comm.py       # Comunicación UART con ESP32
├── config/                 # Archivos de configuración
│   ├── anchors.json       # Posiciones de anchors UWB
│   ├── motor_params.json  # Parámetros de motores
│   └── system_config.json # Configuración general
├── logs/                   # Archivos de log
├── data/                   # Datos de calibración y trazas
└── requirements.txt        # Dependencias de Python
```

### 🔧 **Comunicación con ESP32:**
- **Puerto:** UART (/dev/ttyAMA0 o USB)
- **Velocidad:** 2 Mbps
- **Protocolo:** JSON sobre UART
- **Formato:** Ver documentación en Firmware/Carro/

### 🚀 **Para Empezar:**
1. Instalar dependencias: `pip install -r requirements.txt`
2. Configurar anchors en `config/anchors.json`
3. Ejecutar: `python src/main.py`

### 📊 **Datos Recibidos del ESP32:**
```json
{
  "type": "system_data",
  "timestamp": 12345,
  "uwb": {
    "d1": 125.4, "d2": 89.2, "d3": 147.1,
    "s1": true, "s2": true, "s3": false,
    "freq": 15.2, "count": 1523
  },
  "power": {
    "battery_v": 12.6,
    "motor_l_a": 2.1,
    "motor_r_a": 1.8
  }
}
```

### 📤 **Comandos Enviados al ESP32:**
```json
{
  "type": "motor_command",
  "left_speed": 100,     // -255 a 255
  "right_speed": -50,    // -255 a 255
  "emergency_stop": false
}
```

---
**Proyecto:** Sistema UWB Carrito de Golf  
**Hardware:** ESP32 + Raspberry Pi + DW3000  
**Fecha:** Julio 2025
