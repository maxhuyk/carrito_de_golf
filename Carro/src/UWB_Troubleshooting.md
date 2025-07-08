# Solución de Problemas UWB - DW3000 con MCP23008

## Problema Actual: "DEV_ID IS WRONG!"

Este error indica que los chips DW3000 no están respondiendo correctamente a las consultas SPI.

### Posibles Causas y Soluciones:

#### 1. **Problema del Multiplexor MCP23008**
**Síntomas:** MCP23008 no responde en dirección I2C 0x20
**Verificar:**
- [ ] Conexiones I2C: SDA (GPIO21), SCL (GPIO22)
- [ ] Alimentación MCP23008: VDD = 3.3V o 5V
- [ ] Pines A0, A1, A2 conectados a GND
- [ ] Pull-ups en líneas SDA/SCL (4.7kΩ)

#### 2. **Problemas de Conexión SPI**
**Verificar:**
- [ ] MISO, MOSI, SCK conectados correctamente a todos los DW3000
- [ ] Alimentación 3.3V en todos los DW3000
- [ ] Conexiones de masa (GND)

#### 3. **Problemas de CS (Chip Select)**
**Verificar:**
- [ ] Conexiones del MCP23008 a los pines CS de cada DW3000
- [ ] GP0 → CS1, GP1 → CS2, GP2 → CS3
- [ ] Los pines CS deben estar en HIGH cuando no se usan

#### 4. **Problemas de Reset**
**Verificar:**
- [ ] Conexiones del MCP23008 a los pines RST de cada DW3000
- [ ] GP4 → RST1, GP5 → RST2, GP6 → RST3
- [ ] Los pines RST deben estar en HIGH en operación normal

#### 5. **Problemas de IRQ**
**Verificar:**
- [ ] GPIO35 → IRQ1, GPIO27 → IRQ2, GPIO12 → IRQ3
- [ ] Conexiones directas (no a través del MCP23008)

### Pasos de Diagnóstico:

1. **Verificar MCP23008:**
   ```
   [UWB] === Diagnóstico MCP23008 ===
   [UWB] MCP23008 detectado correctamente en dirección 0x20
   ```

2. **Verificar SPI individual:**
   ```
   [UWB] SPI OK para Anchor1
   [UWB] SPI OK para Anchor2
   [UWB] SPI OK para Anchor3
   ```

3. **Verificar Device ID:**
   Los DW3000 deben responder con un Device ID válido

### Hardware a Verificar:

#### Conexiones I2C:
- ESP32 GPIO21 (SDA) → MCP23008 Pin 2 (SDA)
- ESP32 GPIO22 (SCL) → MCP23008 Pin 1 (SCL)
- MCP23008 Pin 18 (VDD) → 3.3V
- MCP23008 Pin 9 (VSS) → GND
- MCP23008 Pins 3,4,5 (A0,A1,A2) → GND

#### Conexiones SPI (Compartidas):
- ESP32 MISO → Todos los DW3000 MISO
- ESP32 MOSI → Todos los DW3000 MOSI  
- ESP32 SCK → Todos los DW3000 SCK

#### Conexiones CS (A través del MCP23008):
- MCP23008 GP0 → DW3000#1 CS
- MCP23008 GP1 → DW3000#2 CS
- MCP23008 GP2 → DW3000#3 CS

#### Conexiones RST (A través del MCP23008):
- MCP23008 GP4 → DW3000#1 RST
- MCP23008 GP5 → DW3000#2 RST
- MCP23008 GP6 → DW3000#3 RST

#### Conexiones IRQ (Directas):
- ESP32 GPIO35 → DW3000#1 IRQ
- ESP32 GPIO27 → DW3000#2 IRQ
- ESP32 GPIO12 → DW3000#3 IRQ

### Soluciones Temporales:

1. **Reducir número de anchors:**
   En `UWBManager.h` cambiar:
   ```cpp
   #define NUM_ANCHORS 2  // Usar solo 2 anchors
   ```

2. **Probar con delays más largos:**
   Aumentar los delays en la inicialización

3. **Verificar alimentación:**
   Medir voltajes en todos los chips

### Comandos de Diagnóstico:

El nuevo código incluye diagnósticos detallados que mostrarán:
- Estado del MCP23008
- Comunicación I2C
- Estado de cada pin
- Respuesta SPI de cada DW3000
