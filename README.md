# Golf Cart UWB Positioning System - Complete Workspace

## Descripción del Proyecto

Sistema completo de posicionamiento UWB para carrito de golf que incluye:
- **CARRO**: Sistema receptor con 3 anclas DW3000 para trilateración
- **TAG**: Dispositivo móvil que se comunica con las anclas para determinar posición

## Estructura del Proyecto

```
Firmware/
├── Carro/              # Sistema receptor (ESP32 + 3x DW3000 + MCP23008)
│   ├── src/
│   │   ├── main.cpp           # Programa principal del carro
│   │   ├── UWBManager.cpp     # Gestión de anclas y trilateración
│   │   └── UWBManager.h
│   ├── lib/DW3000/           # Librería DW3000 modificada para MCP23008
│   └── platformio.ini        # Configuración PlatformIO del carro
└── TAG/                # Dispositivo móvil (ESP32 + DW3000 + WiFi)
    ├── src/
    │   └── main.cpp           # Programa principal del tag
    ├── lib/DW3000/           # Librería DW3000 estándar
    └── platformio.ini        # Configuración PlatformIO del tag
```

## Hardware

### CARRO (Sistema Receptor)
- **ESP32**: Microcontrolador principal
- **MCP23008**: Expansor I2C para control CS/RST de múltiples DW3000
- **3x DW3000**: Módulos UWB configurados como anclas fijas
- **Conexiones I2C**: SDA=GPIO5, SCL=GPIO4

### TAG (Dispositivo Móvil)
- **ESP32**: Microcontrolador con WiFi
- **1x DW3000**: Módulo UWB configurado como tag móvil
- **Sensor de batería**: Monitoreo de voltaje LiPo
- **OTA**: Actualización over-the-air via WiFi

## Características Técnicas

### Sistema CARRO
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
