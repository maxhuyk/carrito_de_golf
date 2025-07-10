# Golf Cart Control GUI - Manual de Usuario

## Descripción

Interfaz gráfica avanzada para el control del carrito de golf UWB con las siguientes características:

## Características Principales

### 🎯 **Visualización de Posición (Radar)**
- **Radio de 5 metros** en todas las direcciones desde el carrito
- **Grid visual** con marcas cada metro
- **Posición del TAG** relativa al carrito mostrada en tiempo real
- **Vector de velocidad** cuando se mueve (flecha púrpura)
- **Coordenadas numéricas** X, Y en milímetros
- **Velocidad en tiempo real** vX, vY en mm/s
- **Confianza del filtro Kalman** en porcentaje

### 🎮 **Control de Motores**
- **Control de potencia** deslizable (0-255)
- **Botones direccionales**:
  - ↑ Adelante
  - ↓ Atrás  
  - ← Izquierda
  - → Derecha
  - STOP (centro)
- **Control por teclado**:
  - `W` - Adelante
  - `S` - Atrás
  - `A` - Izquierda
  - `D` - Derecha
  - `Espacio` - Parar
  - `U` - Aumentar velocidad (+15)
  - `J` - Disminuir velocidad (-15)

### ⚡ **Monitoreo del Sistema**
- **Voltaje de batería** con indicador visual de nivel
- **Corrientes de motores** izquierdo y derecho en Amperios
- **Estado de conexión** ESP32
- **Botón de parada de emergencia**

## Interfaz de Usuario

### Sección 1: Radar UWB (Izquierda Superior)
```
┌─────────────────────────────────┐
│        Posición UWB - Radio 5m │
│  ┌─────────────────────────────┐ │
│  │    N                        │ │
│  │W   +    •TAG         E      │ │
│  │    S                        │ │
│  │  1m  2.5m  5m circles       │ │
│  └─────────────────────────────┘ │
│  X: 1250.5 mm  Y: -890.2 mm     │
│  vX: 245.1 mm/s vY: -12.3 mm/s  │
│  Confianza: 97.5%               │
└─────────────────────────────────┘
```

### Sección 2: Control de Motores (Derecha Superior)
```
┌─────────────────────────┐
│   Control de Motores    │
│ Potencia: [====|    ] 128│
│                         │
│      ↑ Adelante (W)     │
│  ← Izq  STOP  Der →     │
│   (A)          (D)      │
│      ↓ Atrás (S)        │
│                         │
│ ↑Velocidad(U) ↓Vel.(J)  │
│                         │
│ Estado Motores:         │
│ Motor Izq: 0            │
│ Motor Der: 0            │
└─────────────────────────┘
```

### Sección 3: Estado del Sistema (Inferior)
```
┌─────────────────────────────────────────────────────────┐
│                    Estado del Sistema                   │
│ ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐   │
│ │   Batería   │  │ Corrientes  │  │   Conexión      │   │
│ │ Voltaje:    │  │Motor Izq:   │  │ [Conectar ESP32]│   │
│ │ 12.6 V      │  │ 2.3 A       │  │ Estado:         │   │
│ │[████████  ] │  │Motor Der:   │  │ Conectado       │   │
│ └─────────────┘  │ 1.8 A       │  │                 │   │
│                  └─────────────┘  │[PARADA EMERG.]  │   │
│                                   └─────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

## Instrucciones de Uso

### 1. **Inicio del Sistema**
```bash
# En Raspberry Pi
cd /path/to/RaspberryPi
./start_gui.sh

# O manualmente
python3 start_gui.py
```

### 2. **Conexión al ESP32**
1. Presionar **"Conectar ESP32"**
2. Verificar que el estado cambie a **"Conectado"** (verde)
3. Los datos UWB comenzarán a aparecer automáticamente

### 3. **Control Manual**
- **Con mouse**: Usar los botones direccionales
- **Con teclado**: Usar teclas W,A,S,D para movimiento
- **Velocidad**: U para aumentar, J para disminuir
- **Parada**: Botón STOP o tecla Espacio

### 4. **Monitoreo**
- **Posición**: Observar el punto amarillo en el radar
- **Velocidad**: Vector púrpura indica dirección y velocidad
- **Batería**: Barra de progreso con colores:
  - Verde: >11.5V (bueno)
  - Amarillo: 10.5-11.5V (medio)  
  - Rojo: <10.5V (bajo)
- **Corrientes**: Valores en tiempo real de ambos motores

### 5. **Emergencia**
- **Parada de emergencia**: Botón rojo grande
- **Desconexión automática**: Si se pierde comunicación
- **Stop inmediato**: Tecla Espacio o botón STOP

## Configuración

### Parámetros en `config.json`:
```json
{
  "uart_port": "/dev/serial0",
  "uart_baudrate": 2000000,
  "kalman_filter": {
    "process_noise_pos": 50.0,
    "process_noise_vel": 100.0,
    "measurement_noise": 100.0,
    "initial_uncertainty": 1000.0
  }
}
```

### Personalización:
- **Tamaño de ventana**: Modificar `geometry("1400x900")` en `golf_cart_gui.py`
- **Frecuencia de actualización**: Cambiar valores en `_schedule_updates()` 
- **Colores**: Editar paleta en `gui_styles.py`
- **Rango del radar**: Modificar los 5 metros por defecto en `_setup_radar_display()`

## Resolución de Problemas

### GUI no inicia:
```bash
# Verificar dependencias
python3 start_gui.py

# Instalar tkinter si falta
sudo apt-get install python3-tk

# Verificar display (para SSH)
echo $DISPLAY
# Si está vacío: ssh -X usuario@raspberry_pi
```

### No se conecta al ESP32:
- Verificar puerto serie: `ls /dev/tty*`
- Revisar permisos: `sudo usermod -a -G dialout $USER`
- Comprobar baudrate en config.json
- Verificar que ESP32 esté ejecutando el firmware correcto

### Posición errática:
- Calibrar filtro Kalman con `test_kalman.py`
- Ajustar parámetros de ruido en config.json
- Verificar posición de anchors UWB
- Comprobar interferencias en 6.5 GHz

### Controles no responden:
- Hacer clic en la ventana para enfocar
- Verificar conexión ESP32
- Comprobar que no haya parada de emergencia activada

## Funcionalidades Avanzadas

### Filtro Kalman Integrado:
- **Estado**: [x, y, vx, vy] posición y velocidad
- **Auto-calibración**: Se ajusta automáticamente
- **Predicción**: Estima posición incluso con mediciones perdidas
- **Suavizado**: Reduce ruido de mediciones UWB

### Interfaz Responsive:
- **Actualización 10Hz**: GUI fluida
- **Threading seguro**: Datos en segundo plano
- **Manejo de errores**: Reconexión automática
- **Escalado**: Adaptable a diferentes resoluciones

## Desarrollo Futuro

### Características Planeadas:
- **Grabación de trayectorias**
- **Reproducción automática de rutas**
- **Configuración visual de anchors**
- **Gráficos históricos de rendimiento**
- **Control remoto vía web**
- **Integración con cámara**
- **Mapeo del entorno**

## Seguridad

### Características de Seguridad:
- **Parada de emergencia** siempre disponible
- **Timeout de comunicación** (2 segundos)
- **Límites de velocidad** configurables
- **Monitoreo de batería** con alertas
- **Detección de desconexión** automática

### Mejores Prácticas:
- Siempre mantener visibilidad del carrito
- Verificar estado de batería antes de operar
- Usar parada de emergencia ante cualquier problema
- Mantener área despejada de obstáculos
- No operar con voltaje bajo (<10.5V)

---

**Autor**: Sistema UWB Carrito de Golf  
**Fecha**: Julio 2025  
**Versión**: 1.0.0
