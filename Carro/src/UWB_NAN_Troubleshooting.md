# Diagnóstico de Distancias NAN en Sistema UWB

## Síntomas
- Las distancias UWB funcionan correctamente al inicio
- Repentinamente pasan a mostrar NAN (Not a Number)
- Puede afectar a uno, dos o los tres anchors

## Causas Principales y Soluciones

### 1. Fallos de Comunicación SPI
**Síntomas:**
- El diagnóstico muestra Device ID = 0x00000000
- Fallos intermitentes que pueden "arreglarse" solos

**Causas:**
- Cables SPI sueltos o con mal contacto
- Ruido eléctrico en las líneas SPI
- Problemas con el multiplexor MCP23008

**Soluciones:**
- Verificar todas las conexiones SPI (MOSI, MISO, SCK, CS)
- Añadir capacitores de desacoplo cerca de los módulos DW3000
- Verificar la integridad del multiplexor MCP23008

### 2. Problemas de Alimentación
**Síntomas:**
- Fallos durante transmisión (alta corriente)
- Resets esporádicos de los módulos
- Comportamiento errático

**Causas:**
- Caídas de voltaje (los DW3000 consumen ~100mA durante TX)
- Ruido en la alimentación 3.3V
- Fuente de alimentación insuficiente

**Soluciones:**
- Verificar voltaje de alimentación bajo carga
- Añadir capacitores de filtro (100µF + 10µF + 100nF)
- Usar fuente de mayor capacidad

### 3. Fallos en Double-Sided Ranging
**Síntomas:**
- Los logs muestran "Primera recepción FALLÓ" o "Segunda recepción FALLÓ"
- Distancias intermitentes en anchor específico

**Causas:**
- Timing incorrecto entre frames
- Pérdida de sincronización
- Interferencia RF durante el handshake

**Soluciones:**
- Incrementar delays entre frames (de 5ms a 10ms)
- Verificar que no hay interferencia WiFi/Bluetooth
- Cambiar canal UWB si es posible

### 4. Interferencia de RF
**Síntomas:**
- Fallos más frecuentes cerca de dispositivos WiFi
- Problemas en ciertos momentos del día
- Distancias que fluctúan wildly antes de ir a NAN

**Causas:**
- WiFi 2.4/5GHz
- Bluetooth
- Otros dispositivos UWB
- Reflexiones multipath

**Soluciones:**
- Alejar de fuentes de interferencia
- Cambiar configuración de canal UWB
- Usar antenas direccionales
- Filtros RF

### 5. Problemas de Hardware Específicos
**Síntomas:**
- Un anchor consistentemente falla más que otros
- Fallos que no se recuperan con reboot

**Causas:**
- Módulo DW3000 dañado
- Soldaduras frías
- Problemas de PCB

**Soluciones:**
- Reemplazar módulo problemático
- Verificar soldaduras con microscopio
- Testear PCB con multímetro

## Herramientas de Diagnóstico

### Comandos Serie
```
diag        - Ejecutar diagnóstico completo
help        - Mostrar comandos disponibles
```

### Logs Importantes
- `[UWB_CORE] Estado cambió` - Detecta transiciones a NAN
- `[UWB] AnchorX: Primera/Segunda recepción FALLÓ` - Fallos de comunicación
- `[UWB_STATS]` - Estadísticas de fallos cada 30 segundos
- `Device ID: 0x00000000` - Fallo de comunicación SPI

### Análisis de Estadísticas
- **Fallos < 5%**: Normal, operación estable
- **Fallos 5-20%**: Problemas intermitentes, revisar conexiones
- **Fallos > 20%**: Problema serio, revisar hardware

## Procedimiento de Diagnóstico

1. **Verificar Hardware Básico**
   ```
   - Ejecutar comando "diag"
   - Verificar Device IDs != 0x00000000
   - Revisar conexiones físicas
   ```

2. **Monitorear Patrones de Fallo**
   ```
   - Observar logs por 5-10 minutos
   - Identificar si fallos son aleatorios o específicos de anchor
   - Verificar si coinciden con actividad WiFi
   ```

3. **Test de Aislamiento**
   ```
   - Deshabilitar WiFi temporalmente
   - Alejar dispositivos Bluetooth
   - Probar en diferentes ubicaciones
   ```

4. **Verificación de Alimentación**
   ```
   - Medir voltaje 3.3V bajo carga
   - Verificar corriente total del sistema
   - Añadir capacitores si es necesario
   ```

## Configuraciones de Emergencia

### Aumentar Robustez (menor velocidad)
```cpp
// En UWBCore.cpp, cambiar delays:
delayMicroseconds(10000); // 10ms en lugar de 5ms
```

### Filtrado Más Agresivo
```cpp
// Filtrar distancias más estricto:
if (distance < 10 || distance > 5000) distance = NAN;
```

### Reinicio Automático de Anchors
```cpp
// Añadir cada X fallos consecutivos:
if (consecutive_failures > 10) {
    anchor_reset();
}
```

## Monitoreo Continuo

El sistema ahora incluye:
- **Diagnóstico automático** cada 5 minutos
- **Estadísticas de fallos** cada 30 segundos  
- **Detección de transiciones** a NAN en tiempo real
- **Diagnóstico manual** via comando serie

Esto permite identificar rápidamente cuándo y por qué ocurren los fallos.
