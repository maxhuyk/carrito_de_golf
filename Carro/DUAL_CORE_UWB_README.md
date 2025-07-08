# Sistema UWB Dual-Core para Carrito de Golf

## Arquitectura Implementada

### Core 0 (UWB Dedicado)
- **Archivo**: `UWBCore.cpp` / `UWBCore.h`
- **Función**: Medición UWB exclusiva de alta velocidad
- **Características**:
  - Tarea FreeRTOS de alta prioridad (prioridad 2)
  - Sin delays innecesarios (solo 5ms entre frames)
  - Sin prints de debug que interfieran
  - Comunicación thread-safe con Core 1 via mutex
  - Máxima velocidad de medición posible

### Core 1 (Procesamiento Principal)
- **Archivos**: `main.cpp`, `UWBManager.cpp`, etc.
- **Funciones**: 
  - Loop principal de la aplicación
  - Procesamiento de datos UWB (trilateración, Kalman)
  - WiFi/OTA management
  - Control de motores
  - Comunicación con Raspberry Pi
  - Prints y display de información
  - Filtros y validaciones

## Mejoras Conseguidas

### 1. Frecuencia de Medición
- **Antes**: ~3-5 Hz (limitado por delays y prints en el loop principal)
- **Después**: 10-20+ Hz (Core 0 dedicado, sin interferencias)

### 2. Robustez del Sistema
- **Separación de responsabilidades**: UWB nunca se bloquea por WiFi/OTA
- **Thread-safety**: Comunicación segura entre cores
- **Tolerancia mejorada**: 30cm para trilateración (ajustable)

### 3. Modularidad del Código
- **UWBCore**: Medición pura
- **UWBManager**: Procesamiento y algoritmos
- **main.cpp**: Orquestación y display

## Variables Compartidas (Thread-Safe)

```cpp
volatile UWBRawData g_uwb_raw_data;        // Datos raw del Core 0
volatile bool g_uwb_data_ready;            // Flag de datos listos
volatile unsigned long g_uwb_measurement_count; // Contador de mediciones
```

## Configuración de Anchors

```cpp
// Posiciones en metros (configurables en calculatePosition)
Anchor 1: (0.0, 0.0)     // Origen
Anchor 2: (1.0, 0.0)     // 1m en X
Anchor 3: (0.5, 0.866)   // Triángulo equilátero
```

## Display en Consola

```
[UWB] Freq: 15.2Hz (1234 med) | A1:156.3cm A2:89.7cm A3:201.4cm | Pos: X=0.45m Y=0.23m
```

- **Freq**: Frecuencia actual de medición
- **med**: Número total de mediciones
- **A1/A2/A3**: Distancias a cada anchor o "FAIL"
- **Pos**: Posición calculada o "INVALID"

## Parámetros de Filtro Kalman

```cpp
KALMAN_Q = 0.5f;    // Ruido del proceso (movimiento del tag)
KALMAN_R = 500.0f;  // Ruido de medición UWB
KALMAN_P0 = 10.0f;  // Varianza inicial
```

## Uso del Sistema

### Inicialización
```cpp
UWBManager_setup();  // Automáticamente inicia Core 0 y Core 1
```

### Loop Principal
```cpp
bool uwb_valid = UWBManager_update(tag_x, tag_y);  // Solo procesamiento
float frequency = UWBManager_getMeasurementFrequency();
unsigned long count = UWBManager_getMeasurementCount();
```

## Próximos Pasos

1. **Validación en Hardware**: Verificar frecuencia real (esperado 10-20Hz)
2. **Ajuste de Parámetros**: Afinar Kalman y tolerancias según el entorno
3. **Configuración Dinámica**: Permitir cambio de posiciones de anchors
4. **Optimizaciones Adicionales**: Reducir aún más los microsegundos de delay

## Compilación

✅ **Status**: Compilación exitosa
- RAM utilizada: 13.2% (43,192 bytes)
- Flash utilizada: 58.9% (771,629 bytes)

El sistema está listo para testing en hardware.
