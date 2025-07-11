# 🚗 Sistema UWB para Carrito de Golf

Sistema de posicionamiento Ultra-Wideband (UWB) y control autónomo para carrito de golf, implementado con ESP32 y Raspberry Pi.

## 📋 Descripción del Proyecto

Este proyecto implementa un sistema completo de posicionamiento indoor y control de motores para un carrito de golf utilizando tecnología UWB (Ultra-Wideband) con chips DW3000. El sistema permite:

- **Posicionamiento en tiempo real** con precisión centimétrica
- **Control de motores** desde interfaz gráfica
- **Filtrado Kalman** para suavizar trayectorias
- **Arquitectura dual-core** optimizada en ESP32
- **Interfaz gráfica avanzada** en Raspberry Pi

## 🏗️ Arquitectura del Sistema

```
┌─────────────────┐    UART     ┌─────────────────┐
│   Raspberry Pi  │◄────────────►│      ESP32      │
│                 │   2Mbaud     │                 │
│ • GUI Control   │              │ • Core 0: UWB   │
│ • Trilateración │              │ • Core 1: WiFi  │
│ • Filtro Kalman │              │ • Motor Control │
│ • Navegación    │              │ • Sensores      │
└─────────────────┘              └─────────────────┘
                                          │
                                          │ SPI
                                          ▼
                                 ┌─────────────────┐
                                 │   DW3000 UWB    │
                                 │                 │
                                 │ • 3 Anchors     │
                                 │ • 1 Tag         │
                                 │ • Ranging 45Hz  │
                                 └─────────────────┘
```

## 📁 Estructura del Proyecto

```
Firmware/
├── Carro/                      # Firmware ESP32 del carrito
│   ├── src/
│   │   ├── main.cpp           # Programa principal dual-core
│   │   ├── UWBManager.cpp     # Gestión UWB en Core 0
│   │   ├── MotorController.cpp # Control de motores
│   │   ├── RPiComm.cpp        # Comunicación UART con RPi
│   │   └── WiFiOTA.cpp        # WiFi y actualizaciones OTA
│   ├── lib/DW3000/            # Librería DW3000
│   └── platformio.ini         # Configuración PlatformIO
├── TAG/                       # Firmware ESP32 para anchors
│   └── src/main.cpp
├── RaspberryPi/               # Sistema Python
│   ├── src/
│   │   ├── uart_comm.py       # Comunicación UART
│   │   ├── trilateration.py   # Cálculo de posición
│   │   └── kalman_filter.py   # Filtrado Kalman
│   ├── golf_cart_gui.py       # Interfaz gráfica principal
│   ├── main.py               # Programa principal
│   ├── start_gui.py          # Launcher con verificación
│   ├── setup.sh              # Script de instalación
│   ├── requirements.txt      # Dependencias Python
│   └── testing/              # Scripts de prueba
└── README.md
```

## ⚡ Características Principales

### 🎯 **Sistema UWB de Alta Precisión**
- **Frecuencia de medición**: 45Hz en Core 0 dedicado
- **Precisión**: ±10cm en condiciones ideales
- **Rango**: Hasta 100m en línea de vista
- **Anchors**: 3 puntos de referencia fijos
- **Protocolo**: IEEE 802.15.4z con DW3000

### 🚀 **Arquitectura Dual-Core ESP32**
- **Core 0**: Exclusivo para mediciones UWB de alta velocidad
- **Core 1**: WiFi, motores, comunicación, procesamiento
- **Comunicación inter-core**: FreeRTOS queues
- **OTA**: Actualizaciones remotas por WiFi

### 🖥️ **Interfaz Gráfica Avanzada**
- **Radar en tiempo real**: Visualización 5m x 5m
- **Control de motores**: WASD + U/J para velocidad
- **Monitoreo**: Batería, corrientes, estado de conexión
- **Parada de emergencia**: Botón y timeout de seguridad
- **Reconexión automática**: Sistema robusto de comunicación

### 🧮 **Algoritmos de Procesamiento**
- **Trilateración 2D**: Cálculo de posición con 3 anchors
- **Filtro Kalman**: Suavizado de trayectorias
- **Predicción de movimiento**: Estimación de velocidad
- **Filtrado de ruido**: Rechazo de mediciones erróneas

## 🔧 Instalación y Configuración

### **Prerrequisitos**
- ESP32 DevKit v1 (x2 mínimo, x4 recomendado)
- Raspberry Pi 4 con Raspberry Pi OS
- Módulos DW3000 UWB
- Motores DC con drivers
- Sensores de corriente (ACS712)

### **1. Setup Raspberry Pi**

```bash
# Clonar repositorio
git clone https://github.com/maxhuyk/carrito_de_golf.git
cd carrito_de_golf/RaspberryPi

# Instalación automática
chmod +x setup.sh
./setup.sh

# O instalación manual
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-venv python3-tk -y
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### **2. Configurar UART**

```bash
# Habilitar UART en Raspberry Pi
echo "enable_uart=1" | sudo tee -a /boot/config.txt
sudo reboot
```

### **3. Conexiones Físicas**

| ESP32        | Raspberry Pi  | Función           |
|--------------|---------------|-------------------|
| GPIO17 (TX2) | GPIO15 (RXD)  | UART TX           |
| GPIO16 (RX2) | GPIO14 (TXD)  | UART RX           |
| GND          | GND           | Tierra común      |

### **4. Programar ESP32**

```bash
# Instalar PlatformIO
pip install platformio

# Compilar y subir firmware del carrito
cd Firmware/Carro
pio run --target upload

# Compilar y subir firmware de anchors
cd ../TAG
pio run --target upload
```

## 🚀 Ejecución

### **Iniciar Sistema Completo**

```bash
# En Raspberry Pi
cd carrito_de_golf/RaspberryPi
source venv/bin/activate

# Launcher con verificación de dependencias
python3 start_gui.py

# O directamente la GUI
python3 golf_cart_gui.py
```

### **Controles de la GUI**

| Tecla | Función                    |
|-------|----------------------------|
| W     | Avanzar                    |
| S     | Retroceder                 |
| A     | Girar izquierda           |
| D     | Girar derecha             |
| U     | Aumentar velocidad (+15)   |
| J     | Disminuir velocidad (-15)  |
| Space | Parada de emergencia       |

## 📊 Monitoreo y Diagnóstico

### **Datos en Tiempo Real**
- **Posición UWB**: Coordenadas X,Y en cm
- **Distancias**: A cada anchor individual
- **Frecuencia**: Mediciones por segundo
- **Batería**: Voltaje del sistema
- **Corrientes**: Motores izquierdo y derecho
- **Estado**: Conexión ESP32, anchors activos

### **Scripts de Diagnóstico**

```bash
# Monitoreo en tiempo real
python3 testing/monitor_realtime.py

# Prueba de módulos
python3 testing/test_modules.py

# Test comunicación UART
python3 testing/test_uart.py

# Validación filtro Kalman
python3 testing/test_kalman.py
```

## ⚙️ Configuración Avanzada

### **Parámetros UWB (`config.json`)**

```json
{
  "anchor_positions": {
    "anchor1": {"x": -280.0, "y": 0.0, "z": 0.0},
    "anchor2": {"x": 280.0, "y": 0.0, "z": 0.0},
    "anchor3": {"x": 165.0, "y": -285.0, "z": -140.0}
  },
  "kalman_filter": {
    "process_noise_pos": 50.0,
    "process_noise_vel": 100.0,
    "measurement_noise": 25.0
  }
}
```

### **Parámetros ESP32**

```cpp
// UWBManager.h
#define UWB_MEASUREMENT_FREQUENCY 45  // Hz
#define NUM_ANCHORS 3
#define COMMAND_TIMEOUT 5000          // ms

// MotorController.h
#define MOTOR_MAX_SPEED 255
#define MOTOR_PWM_FREQUENCY 1000      // Hz
```

## 🔬 Tecnologías Utilizadas

### **Hardware**
- **ESP32**: Procesamiento dual-core, WiFi, control
- **DW3000**: Chips UWB IEEE 802.15.4z
- **Raspberry Pi 4**: Procesamiento, GUI, navegación
- **ACS712**: Sensores de corriente
- **Driver de motores**: Control PWM

### **Software**
- **C++/Arduino**: Firmware ESP32 optimizado
- **Python 3.8+**: Sistema de control Raspberry Pi
- **tkinter**: Interfaz gráfica nativa
- **NumPy**: Cálculos matemáticos optimizados
- **PySerial**: Comunicación UART
- **FreeRTOS**: Multitarea en ESP32
- **PlatformIO**: Desarrollo y deployment

### **Algoritmos**
- **Trilateración LSQ**: Least Squares positioning
- **Filtro Kalman**: State estimation clásico
- **Time Difference of Arrival**: Medición UWB
- **PWM Control**: Modulación de motores

## 📈 Rendimiento

### **Métricas del Sistema**
- **Latencia UWB**: <22ms (45Hz)
- **Precisión posicionamiento**: ±10-20cm
- **Frecuencia GUI**: 30 FPS
- **Tiempo respuesta motores**: <100ms
- **Autonomía**: Depende de batería del carrito

### **Optimizaciones**
- Core 0 dedicado exclusivamente a UWB
- Comunicación UART a 2Mbaud
- Filtrado adaptativo de ruido
- Buffer circular para datos históricos

## 🛡️ Seguridad

### **Características de Seguridad**
- **Timeout automático**: Para motores sin comunicación
- **Parada de emergencia**: Botón físico y software
- **Validación de comandos**: Límites de velocidad
- **Watchdog**: Reset automático en caso de fallo
- **Heartbeat**: Monitoreo de conectividad

## 🤝 Contribución

### **Cómo Contribuir**
1. Fork del repositorio
2. Crear rama para feature (`git checkout -b feature/nueva-funcionalidad`)
3. Commit cambios (`git commit -am 'Agregar nueva funcionalidad'`)
4. Push a la rama (`git push origin feature/nueva-funcionalidad`)
5. Crear Pull Request

### **Estándares de Código**
- **C++**: Google C++ Style Guide
- **Python**: PEP 8
- **Commits**: Conventional Commits
- **Documentación**: Markdown con diagramas

## 📄 Licencia

Este proyecto está bajo la Licencia MIT. Ver `LICENSE` para más detalles.

## 👨‍💻 Autor

**Sistema UWB Carrito de Golf**
- GitHub: [@maxhuyk](https://github.com/maxhuyk)
- Proyecto: [carrito_de_golf](https://github.com/maxhuyk/carrito_de_golf)

## 🙏 Agradecimientos

- Comunidad ESP32 y Arduino
- Desarrolladores de la librería DW3000
- Contribuidores de NumPy y Python
- Documentación IEEE 802.15.4z

---

**⚡ Sistema listo para implementación en hardware real ⚡**

*Desarrollado con ❤️ para navegación autónoma y control inteligente*
