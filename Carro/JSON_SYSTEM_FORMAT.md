# Formato JSON del Sistema Completo

## Datos Enviados del ESP32 a Raspberry Pi

El ESP32 envía cada 50ms (20Hz) un JSON completo con todos los datos del sistema:

```json
{
  "type": "system_data",
  "timestamp": 123456789,
  "uwb": {
    "d1": 156.3,
    "d2": 89.7,
    "d3": 201.4,
    "s1": true,
    "s2": true,
    "s3": false,
    "freq": 20.5,
    "count": 1234
  },
  "power": {
    "battery_v": 11.8,
    "motor_l_a": 2.3,
    "motor_r_a": 2.1,
    "total_a": 4.7
  }
}
```

## Descripción de Campos

### UWB (Ultra-WideBand)
- **d1, d2, d3**: Distancias a los anchors 1, 2 y 3 en centímetros (NaN si inválido)
- **s1, s2, s3**: Estado de conexión de cada anchor (true=conectado, false=fallo)
- **freq**: Frecuencia real de medición del Core 0 en Hz
- **count**: Número total de mediciones realizadas

### Power (Sistema de Energía)
- **battery_v**: Voltaje de la batería en voltios
- **motor_l_a**: Corriente del motor izquierdo en amperios
- **motor_r_a**: Corriente del motor derecho en amperios  
- **total_a**: Corriente total del sistema en amperios

## Configuración de Hardware

### Pines ADC Utilizados
```cpp
#define BATTERY_VOLTAGE_PIN 36    // ADC1_CH0 (VP) - Divisor de voltaje
#define MOTOR_L_CURRENT_PIN 39    // ADC1_CH3 (VN) - Sensor ACS712
#define MOTOR_R_CURRENT_PIN 34    // ADC1_CH6 - Sensor ACS712
#define SYSTEM_CURRENT_PIN  35    // ADC1_CH7 - Sensor ACS712
```

### Sensores Recomendados
- **Voltaje de Batería**: Divisor resistivo 4:1 (12V → 3V)
- **Corriente de Motores**: ACS712-30A (66mV/A, centro en 2.5V)
- **ADC**: 12-bit, referencia 3.3V

## Calibración

Los valores pueden ajustarse en `MotorController.cpp`:

```cpp
#define BATTERY_VOLTAGE_SCALE 0.01611  // Escalado del divisor
#define CURRENT_SCALE 0.066            // Sensibilidad del ACS712
```

## Procesamiento en Raspberry Pi

La Raspberry Pi debe:

1. **Parsear JSON** cada 50ms
2. **Trilateración** con uwb.d1, d2, d3
3. **Filtro Kalman** para suavizar posición
4. **Monitoreo de batería** para alertas
5. **Control de corriente** para protección
6. **Navegación y evasión** de obstáculos

## Ventajas

- ✅ **Datos completos** en un solo mensaje
- ✅ **Sincronización temporal** con timestamp
- ✅ **Monitoreo integral** del sistema
- ✅ **Diagnóstico remoto** de fallas
- ✅ **Optimización energética** basada en datos reales
