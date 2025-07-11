# 🏌️ Sistema UWB para Carrito de Golf

Sistema avanzado de posicionamiento y control para carrito de golf utilizando tecnología Ultra-Wideband (UWB) con ESP32 y Raspberry Pi.

## 📋 Descripción del Proyecto

Este proyecto implementa un sistema completo de navegación y control para un carrito de golf que combina:

- **🎯 Posicionamiento UWB de alta precisión** con chips DW3000
- **🧠 Procesamiento dual-core** en ESP32 (Core 0: UWB, Core 1: Control)
- **🖥️ Interfaz gráfica avanzada** en Raspberry Pi con control en tiempo real
- **🔧 Control de motores PWM** con retroalimentación de sensores
- **📡 Comunicación UART** entre ESP32 y Raspberry Pi
- **🎮 Control manual** por teclado (WASD) y interfaz táctil

## 🏗️ Arquitectura del Sistema

```
┌─────────────────┐    UART     ┌─────────────────────┐
│   Raspberry Pi  │◄──────────►│      ESP32          │
│                 │  2Mbaud     │                     │
│ • GUI Control   │             │ • Core 0: UWB       │
│ • Trilateración │             │ • Core 1: Motores   │
│ • Filtro Kalman │             │ • WiFi/OTA          │
│ • Navegación    │             │ • Sensores          │
└─────────────────┘             └─────────────────────┘
        │                                │
        ▼                                ▼
┌─────────────────┐             ┌─────────────────────┐
│   Pantalla      │             │    Anchors UWB      │
│   • Radar 5m    │             │  • 3x DW3000        │
│   • Controles   │             │  • Trilateración    │
│   • Monitoreo   │             │  • 45+ Hz           │
└─────────────────┘             └─────────────────────┘
```

## 🚀 Características Principales

### 🎯 Sistema UWB
- **Chips DW3000** para posicionamiento de alta precisión
- **3 Anchors** configurados en trilateración 2D
- **Frecuencia de medición**: 45+ Hz
- **Precisión**: ~10cm en condiciones ideales
- **Rango**: Hasta 500m en campo abierto

### 🧠 Procesamiento Dual-Core ESP32
- **Core 0 (Dedicado UWB)**: Mediciones de alta velocidad sin interrupciones
- **Core 1 (Control)**: WiFi, motores, sensores, comunicación con RPi
- **WiFi OTA**: Actualizaciones remotas del firmware
- **Gestión de energía**: Monitoreo de batería y corrientes

### 🖥️ Interfaz Gráfica Raspberry Pi
- **Radar en tiempo real** con visualización de posición (rango 5m)
- **Control de motores** con retroalimentación visual
- **Monitoreo de sistema**: batería, corrientes, frecuencia UWB
- **Control por teclado**: WASD (movimiento), U/J (velocidad)
- **Parada de emergencia** con timeout de seguridad

### 🔧 Control de Motores
- **PWM preciso** para motores izquierdo y derecho
- **Rango de velocidad**: -255 a +255 (adelante/atrás)
- **Sensores de corriente** para monitoreo en tiempo real
- **Timeout de seguridad**: Parada automática sin comunicación

### 📊 Filtrado y Procesamiento
- **Filtro Kalman estándar** para suavizar posiciones UWB
- **Trilateración 2D** con validación de errores
- **Detección de outliers** y manejo de anchors desconectados

## 📁 Estructura del Proyecto
- ✅ Control de 3 anclas DW3000 via MCP23008
- ✅ Trilateración en tiempo real
- ✅ Filtro de Kalman para suavizado
- ✅ Comunicación I2C estable
- ✅ Sistema sin resets ni watchdog issues

### Sistema TAG
- ✅ Ranging bi-direccional con anclas
- ✅ Monitoreo de batería LiPo
- ✅ Conectividad WiFi
- ✅ Actualización OTA
- ✅ Transmisión de datos de posición

## Compilación y Uso

### Para el CARRO:
```bash
cd Carro
pio run --target upload
pio device monitor --baud 115200
```

### Para el TAG:
```bash
cd TAG
pio run --target upload
pio device monitor --baud 2000000
```

## Estado del Desarrollo

### ✅ COMPLETADO:
- [x] Sistema CARRO completamente funcional
- [x] Integración MCP23008 exitosa
- [x] 3 anclas DW3000 inicializadas correctamente
- [x] Algoritmo de trilateración implementado
- [x] Sistema TAG con ranging básico
- [x] Monitoreo de batería en TAG
- [x] Conectividad WiFi en TAG

### 🔄 EN DESARROLLO:
- [ ] Comunicación WiFi entre TAG y CARRO
- [ ] Interfaz web para monitoreo
- [ ] Control de motores del carrito
- [ ] Calibración automática de anclas
- [ ] Optimización de algoritmos

### 📋 PRÓXIMOS PASOS:
1. Pruebas con TAG real para validar mediciones
2. Implementar comunicación WiFi TAG→CARRO
3. Desarrollar interfaz web de control
4. Integrar control de motores
5. Optimizar consumo de energía

## Configuración de Ramas Git

- `master`: Versión estable y funcional
- `development`: Desarrollo general
- `feature/wifi-communication`: Comunicación TAG↔CARRO
- `feature/motor-control`: Control de motores
- `feature/web-interface`: Interfaz web
- `feature/remote-control`: Control remoto

## Autor

Proyecto desarrollado para CTII - Carrito de Golf Autónomo con posicionamiento UWB
Versión: 1.0.0

```
carrito_de_golf/
├── Carro/                          # Firmware ESP32 principal
│   ├── src/
│   │   ├── main.cpp               # Loop principal dual-core
│   │   ├── UWBManager.cpp         # Gestión UWB Core 0
│   │   ├── MotorController.cpp    # Control PWM motores
│   │   ├── RPiComm.cpp           # Comunicación UART con RPi
│   │   └── WiFiOTA.cpp           # WiFi y actualizaciones OTA
│   ├── lib/DW3000/               # Librería DW3000 UWB
│   └── platformio.ini            # Configuración ESP32
│
├── TAG/                           # Firmware para TAG adicional
│   └── src/main.cpp              # Código TAG simple
│
└── RaspberryPi/                   # Sistema Raspberry Pi
    ├── src/
    │   ├── uart_comm.py          # Comunicación UART con ESP32
    │   ├── trilateration.py      # Algoritmos de trilateración
    │   └── kalman_filter.py      # Filtro Kalman para posiciones
    ├── golf_cart_gui.py          # Interfaz gráfica principal
    ├── gui_styles.py             # Estilos y temas GUI
    ├── start_gui.py              # Launcher con verificación
    ├── main.py                   # Sistema sin GUI (consola)
    ├── config.json               # Configuración del sistema
    ├── requirements.txt          # Dependencias Python
    ├── setup.sh                  # Script instalación automática
    └── testing/                  # Scripts de prueba y desarrollo
        ├── test_modules.py
        ├── test_kalman.py
        └── monitor_realtime.py
```

## 🛠️ Instalación y Configuración

### Requisitos Hardware

#### ESP32 (Carrito Principal)
- **ESP32-WROOM-32** o compatible
- **Chip DW3000** para UWB
- **Expansor I/O MCP23008** para control de motores
- **Sensores de corriente** (ej: ACS712)
- **Divisor de tensión** para monitoreo de batería
- **Motores DC** con drivers PWM

#### Anchors UWB (3 unidades)
- **ESP32** + **DW3000** cada uno
- **Posicionamiento fijo** conocido en el área

#### Raspberry Pi
- **Raspberry Pi 4** (recomendado) o 3B+
- **Pantalla táctil** o monitor + teclado/mouse
- **Conexión UART** con ESP32

### Instalación Raspberry Pi

1. **Clonar repositorio**:
```bash
git clone https://github.com/maxhuyk/carrito_de_golf.git
cd carrito_de_golf/RaspberryPi
```

2. **Instalación automática**:
```bash
chmod +x setup.sh
./setup.sh
```

3. **Configurar UART** (si no lo hizo setup.sh):
```bash
echo "enable_uart=1" | sudo tee -a /boot/config.txt
sudo reboot
```

4. **Ejecutar sistema**:
```bash
# Activar entorno virtual
source venv/bin/activate

# Opción 1: Con interfaz gráfica
python3 start_gui.py

# Opción 2: Solo consola
python3 main.py
```

### Configuración ESP32

1. **Instalar PlatformIO**:
```bash
# Con Visual Studio Code + PlatformIO extension
# O desde línea de comandos:
pip install platformio
```

2. **Compilar y subir firmware**:
```bash
cd carrito_de_golf/Carro
pio run --target upload
```

3. **Configurar anchors** (repetir para cada TAG):
```bash
cd carrito_de_golf/TAG
pio run --target upload
```

## ⚙️ Configuración del Sistema

### Archivo config.json
```json
{
  "uart_port": "/dev/serial0",
  "uart_baudrate": 2000000,
  "anchor_positions": {
    "anchor1": {"x": -280.0, "y": 0.0, "z": 0.0},
    "anchor2": {"x": 280.0, "y": 0.0, "z": 0.0},
    "anchor3": {"x": 165.0, "y": -285.0, "z": -140.0}
  },
  "kalman_filter": {
    "process_noise_pos": 50.0,
    "measurement_noise": 25.0
  }
}
```

### Conexiones Hardware

#### ESP32 ↔ Raspberry Pi
- **ESP32 GPIO17 (TX)** → **RPi GPIO15 (RX, Pin 10)**
- **ESP32 GPIO16 (RX)** → **RPi GPIO14 (TX, Pin 8)**
- **GND** → **GND**

#### ESP32 ↔ DW3000
- **SPI estándar** + **GPIO para control**
- Ver `platformio.ini` para pines específicos

## 🎮 Uso del Sistema

### Interfaz Gráfica

1. **Radar**: Muestra posición en tiempo real en área de 5m
2. **Control de motores**: 
   - **WASD**: Adelante/Atrás/Izquierda/Derecha
   - **U/J**: Aumentar/Disminuir velocidad
   - **Barra espaciadora**: Parada de emergencia
3. **Monitoreo**: Batería, corrientes, frecuencia UWB
4. **Conexión**: Botón conectar/desconectar ESP32

### Protocolo de Comunicación

#### ESP32 → Raspberry Pi (JSON)
```json
{
  "type": "system_data",
  "timestamp": 12345,
  "uwb": {
    "d1": 150.5, "d2": 200.3, "d3": 175.8,
    "s1": true, "s2": true, "s3": false,
    "freq": 45.2, "count": 1234
  },
  "power": {
    "battery_v": 12.4,
    "motor_l_a": 0.8,
    "motor_r_a": 1.2
  }
}
```

#### Raspberry Pi → ESP32 (JSON)
```json
{
  "type": "motor_command",
  "left_speed": 100,
  "right_speed": -50,
  "emergency_stop": false
}
```

## 🔧 Desarrollo y Debugging

### Scripts de Prueba
```bash
cd RaspberryPi/testing/

# Probar módulos individualmente
python3 test_modules.py

# Monitoreo en tiempo real
python3 monitor_realtime.py

# Verificar filtro Kalman
python3 test_kalman.py
```

### Logs y Debugging
- **ESP32**: Monitor serie a 2Mbaud
- **Raspberry Pi**: Logs en consola con niveles configurables
- **GUI**: Indicadores visuales de estado

## 📊 Rendimiento

### Especificaciones UWB
- **Frecuencia de medición**: 45+ Hz
- **Latencia total**: <50ms (UWB → Trilateración → GUI)
- **Precisión posición**: 10-30cm (dependiendo del entorno)
- **Alcance máximo**: 500m en campo abierto

### Recursos Sistema
- **ESP32**: ~60% CPU, 180KB RAM
- **Raspberry Pi**: ~25% CPU, 150MB RAM (con GUI)
- **Comunicación UART**: 2Mbaud, <1% pérdida de paquetes

## 🛣️ Roadmap Futuro

- [ ] **Navegación autónoma** con waypoints
- [ ] **Interfaz web** para control remoto
- [ ] **Integración GPS** para navegación outdoor
- [ ] **Machine learning** para optimización de rutas
- [ ] **App móvil** para control desde smartphone
- [ ] **Múltiples carritos** en red mesh

## 🤝 Contribución

1. Fork el repositorio
2. Crear rama feature (`git checkout -b feature/nueva-funcionalidad`)
3. Commit cambios (`git commit -am 'Add nueva funcionalidad'`)
4. Push a la rama (`git push origin feature/nueva-funcionalidad`)
5. Crear Pull Request

## 📄 Licencia

Este proyecto está bajo la Licencia MIT - ver archivo `LICENSE` para detalles.

## 👨‍💻 Autor

**Sistema UWB Carrito de Golf**
- Desarrollo: Julio 2025
- Contacto: [maxhuyk](https://github.com/maxhuyk)

## 🙏 Agradecimientos

- Librería DW3000 para ESP32
- Comunidad PlatformIO
- Documentación UWB Consortium
- Algoritmos de trilateración y filtros Kalman

---

**⚠️ Nota**: Este es un proyecto experimental. Siempre mantener supervisión humana durante las pruebas y operación del carrito.
