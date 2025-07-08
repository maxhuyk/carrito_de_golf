# Sistema UWB de Trilateración para Carrito de Golf

## Descripción
Sistema de posicionamiento UWB (Ultra-Wideband) basado en ESP32 que utiliza 3 anchors DW3000 para trilateración y determinar la posición de un tag en tiempo real.

## Características
- **ESP32** como microcontrolador principal
- **3 módulos DW3000** para ranging UWB de alta precisión
- **MCP23008** como expansor I2C para control de CS/RST de los DW3000
- **Trilateración** con filtro de Kalman para suavizado de mediciones
- **Comunicación I2C** en GPIO5 (SDA) y GPIO4 (SCL)

## Hardware
- ESP32 (board: esp-wrover-kit)
- 3x módulos DW3000 UWB
- 1x MCP23008 I2C Port Expander
- Conexiones I2C: SDA=GPIO5, SCL=GPIO4

## Configuración de Pines

### Anchors (vía MCP23008):
- **Anchor 1**: CS=GP0, RST=GP4, IRQ=GPIO35
- **Anchor 2**: CS=GP1, RST=GP5, IRQ=GPIO27  
- **Anchor 3**: CS=GP2, RST=GP6, IRQ=GPIO13

### I2C:
- **SDA**: GPIO5
- **SCL**: GPIO4
- **MCP23008**: Dirección 0x20

## Estado del Desarrollo
✅ **VERSIÓN FUNCIONAL** - Sistema completamente operativo

### Logros alcanzados:
- ✅ Comunicación I2C estable
- ✅ MCP23008 funcionando correctamente
- ✅ Los 3 DW3000 inicializados y comunicando por SPI
- ✅ UWBManager implementado con trilateración
- ✅ Filtros de Kalman implementados
- ✅ Sistema sin resets ni watchdog issues

### Próximos pasos:
- [ ] Pruebas con tag real para validar mediciones
- [ ] Calibración de posiciones de anchors
- [ ] Optimización de algoritmos de trilateración
- [ ] Implementación de interfaz de usuario

## Compilación y Carga
```bash
cd "e:\CTII\Carrito de golf\Firmware\Carro"
pio run --target upload
pio device monitor --baud 115200
```

## Estructura del Código
- `src/main.cpp` - Programa principal
- `src/UWBManager.cpp/h` - Gestión de anchors y trilateración
- `lib/DW3000/` - Librería DW3000 modificada para MCP23008
- `platformio.ini` - Configuración del proyecto

## Dependencias
```ini
lib_deps = 
    adafruit/Adafruit MCP23008 library
    adafruit/Adafruit BusIO
```

## Autor
Proyecto desarrollado para CTII - Carrito de Golf con posicionamiento UWB

## Licencia
Proyecto académico/educativo
