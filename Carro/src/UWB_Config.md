# Configuración UWB Manager - Nuevo Hardware

## Resumen de Cambios

Este código ha sido actualizado para funcionar con el nuevo hardware que incluye:
- 3 anchors UWB
- Multiplexor MCP23008 para control de CS y RST
- Configuración flexible para usar 2 o 3 anchors

## Configuración de Hardware

### Multiplexor MCP23008 (Dirección I2C: 0x20)
- **GP0**: CS1 (Anchor 1)
- **GP1**: CS2 (Anchor 2)  
- **GP2**: CS3 (Anchor 3)
- **GP4**: RST1 (Reset Anchor 1)
- **GP5**: RST2 (Reset Anchor 2)
- **GP6**: RST3 (Reset Anchor 3)

### Pines IRQ (Directos en ESP32)
- **GPIO35**: IRQ1 (Anchor 1)
- **GPIO27**: IRQ2 (Anchor 2)
- **GPIO12**: IRQ3 (Anchor 3)

## Configuración de Software

### Cambiar Número de Anchors
En el archivo `UWBManager.h`, modificar la línea:
```cpp
#define NUM_ANCHORS 3  // Cambiar a 2 para usar solo 2 anchors
```

### Posicionamiento de Anchors
En la función `calculatePosition3D()`, ajustar las coordenadas de los anchors según tu configuración física:
```cpp
float x1 = 0.0, y1 = 0.0;    // Anchor 1
float x2 = 0.3, y2 = 0.0;    // Anchor 2
float x3 = 0.15, y3 = 0.3;   // Anchor 3
```

## Funcionalidades Nuevas

1. **Multiplexor MCP23008**: Control automático de CS y RST a través de I2C
2. **Configuración flexible**: Soporte para 2 o 3 anchors
3. **Trilateración mejorada**: Uso de 3 anchors para mejor precisión
4. **Fallback automático**: Si un anchor falla, usa los disponibles
5. **Mejor diagnóstico**: Mensajes de estado durante la inicialización

## Funciones Principales

- `UWBManager_setup()`: Inicializa el sistema con el nuevo hardware
- `UWBManager_update()`: Actualiza mediciones y calcula posición
- `calculatePosition2D()`: Cálculo con 2 anchors
- `calculatePosition3D()`: Cálculo con 3 anchors (nueva)

## Filtro de Kalman

Los parámetros del filtro se mantienen:
- `KALMAN_Q = 0.5f`: Ruido del proceso
- `KALMAN_R = 500.0f`: Ruido de medición
- `KALMAN_P0 = 10.0f`: Varianza inicial

## Compilación

Asegúrate de tener la librería Wire.h incluida en el proyecto para el control I2C del multiplexor.
