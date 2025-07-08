#include "DW3000.h"
#include <Arduino.h>
#include "UWBManager.h"
#include <math.h>
#include <algorithm>

// Configuración de pines del multiplexor MCP23008
#define ANCHOR1_CS 0     // GP0 del MCP23008
#define ANCHOR1_RST 4    // GP4 del MCP23008
#define ANCHOR1_IRQ 35   // GPIO35 del ESP32 (directo)

#define ANCHOR2_CS 1     // GP1 del MCP23008
#define ANCHOR2_RST 5    // GP5 del MCP23008
#define ANCHOR2_IRQ 27   // GPIO27 del ESP32 (directo)

#define ANCHOR3_CS 2     // GP2 del MCP23008
#define ANCHOR3_RST 6    // GP6 del MCP23008
#define ANCHOR3_IRQ 13   // GPIO13 del ESP32 (directo)

// Parámetros del filtro de Kalman (ajusta aquí)
static float KALMAN_Q = 0.5f; // Ruido del proceso (mayor si el tag se mueve rápido)
static float KALMAN_R = 500.0f; // Ruido de medición (mayor si el UWB es ruidoso) era 8
static float KALMAN_P0 = 10.0f; // Varianza inicial

static DW3000Class anchor1(ANCHOR1_CS, ANCHOR1_RST, ANCHOR1_IRQ);
static DW3000Class anchor2(ANCHOR2_CS, ANCHOR2_RST, ANCHOR2_IRQ);
static DW3000Class anchor3(ANCHOR3_CS, ANCHOR3_RST, ANCHOR3_IRQ);

// Instancias de filtro de Kalman para cada distancia
static KalmanFilter1D kf_d1(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);
static KalmanFilter1D kf_d2(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);
static KalmanFilter1D kf_d3(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);

void UWBManager_setup() {
    // Inicializar Anchor 1
    anchor1.begin();
    anchor1.hardReset();
    delay(200);
    while (!anchor1.checkForIDLE()) delay(100);
    anchor1.softReset();
    delay(200);
    anchor1.init();
    anchor1.setupGPIO();
    anchor1.configureAsTX();
    anchor1.clearSystemStatus();

    // Inicializar Anchor 2
    anchor2.begin();
    anchor2.hardReset();
    delay(200);
    while (!anchor2.checkForIDLE()) delay(100);
    anchor2.softReset();
    delay(200);
    anchor2.init();
    anchor2.setupGPIO();
    anchor2.configureAsTX();
    anchor2.clearSystemStatus();

    // Inicializar Anchor 3
    anchor3.begin();
    anchor3.hardReset();
    delay(200);
    while (!anchor3.checkForIDLE()) delay(100);
    anchor3.softReset();
    delay(200);
    anchor3.init();
    anchor3.setupGPIO();
    anchor3.configureAsTX();
    anchor3.clearSystemStatus();
}

static void calculatePosition(float d1, float d2, float d3, float &x, float &y) {
    // Posiciones de los anchors (en metros)
    float x1 = 0.0, y1 = 0.0;    // Anchor 1
    float x2 = 0.3, y2 = 0.0;    // Anchor 2  
    float x3 = 0.15, y3 = 0.3;   // Anchor 3 (formar triángulo)
    
    static float last_x = NAN, last_y = NAN;
    
    // Método de mínimos cuadrados para trilateración
    // Sistema de ecuaciones: (x-xi)² + (y-yi)² = di²
    // Linearizamos: 2(x1-x3)x + 2(y1-y3)y = d3²-d1²+x1²+y1²-x3²-y3²
    //               2(x2-x3)x + 2(y2-y3)y = d3²-d2²+x2²+y2²-x3²-y3²
    
    float A11 = 2 * (x1 - x3);
    float A12 = 2 * (y1 - y3);
    float A21 = 2 * (x2 - x3);
    float A22 = 2 * (y2 - y3);
    
    float b1 = d3*d3 - d1*d1 + x1*x1 + y1*y1 - x3*x3 - y3*y3;
    float b2 = d3*d3 - d2*d2 + x2*x2 + y2*y2 - x3*x3 - y3*y3;
    
    // Resolver sistema 2x2: A * [x y]' = b
    float det = A11 * A22 - A12 * A21;
    
    if (abs(det) < 1e-6) {
        // Sistema singular, usar última posición conocida
        x = last_x;
        y = last_y;
        return;
    }
    
    x = (A22 * b1 - A12 * b2) / det;
    y = (A11 * b2 - A21 * b1) / det;
    
    // Validación básica de la solución
    float dist_check1 = sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1));
    float dist_check2 = sqrt((x-x2)*(x-x2) + (y-y2)*(y-y2));
    float dist_check3 = sqrt((x-x3)*(x-x3) + (y-y3)*(y-y3));
    
    const float TOLERANCE = 0.1; // 10cm de tolerancia
    if (abs(dist_check1 - d1) > TOLERANCE || 
        abs(dist_check2 - d2) > TOLERANCE || 
        abs(dist_check3 - d3) > TOLERANCE) {
        // Solución no válida, usar última posición conocida
        x = last_x;
        y = last_y;
        return;
    }
    
    last_x = x;
    last_y = y;
}

bool UWBManager_update(float &tag_x, float &tag_y) {
    // Medición Anchor 1
    float distance1 = NAN, distance2 = NAN, distance3 = NAN;
    int t_roundA = 0, t_replyA = 0, clock_offset = 0, ranging_time = 0;
    long long rx = 0, tx = 0;
    
    // Medición Anchor 1
    anchor1.ds_sendFrame(1);
    tx = anchor1.readTXTimestamp();
    delay(10);
    if (anchor1.receivedFrameSucc() == 1) {
        anchor1.clearSystemStatus();
        rx = anchor1.readRXTimestamp();
        anchor1.ds_sendFrame(3);
        t_roundA = rx - tx;
        tx = anchor1.readTXTimestamp();
        t_replyA = tx - rx;
        delay(10);
        if (anchor1.receivedFrameSucc() == 1) {
            anchor1.clearSystemStatus();
            clock_offset = anchor1.getRawClockOffset();
            ranging_time = anchor1.ds_processRTInfo(t_roundA, t_replyA, anchor1.read(0x12, 0x04), anchor1.read(0x12, 0x08), clock_offset);
            distance1 = anchor1.convertToCM(ranging_time);
        }
    }
    
    // Medición Anchor 2
    anchor2.ds_sendFrame(1);
    tx = anchor2.readTXTimestamp();
    delay(10);
    if (anchor2.receivedFrameSucc() == 1) {
        anchor2.clearSystemStatus();
        rx = anchor2.readRXTimestamp();
        anchor2.ds_sendFrame(3);
        t_roundA = rx - tx;
        tx = anchor2.readTXTimestamp();
        t_replyA = tx - rx;
        delay(10);
        if (anchor2.receivedFrameSucc() == 1) {
            anchor2.clearSystemStatus();
            clock_offset = anchor2.getRawClockOffset();
            ranging_time = anchor2.ds_processRTInfo(t_roundA, t_replyA, anchor2.read(0x12, 0x04), anchor2.read(0x12, 0x08), clock_offset);
            distance2 = anchor2.convertToCM(ranging_time);
        }
    }
    
    // Medición Anchor 3
    anchor3.ds_sendFrame(1);
    tx = anchor3.readTXTimestamp();
    delay(10);
    if (anchor3.receivedFrameSucc() == 1) {
        anchor3.clearSystemStatus();
        rx = anchor3.readRXTimestamp();
        anchor3.ds_sendFrame(3);
        t_roundA = rx - tx;
        tx = anchor3.readTXTimestamp();
        t_replyA = tx - rx;
        delay(10);
        if (anchor3.receivedFrameSucc() == 1) {
            anchor3.clearSystemStatus();
            clock_offset = anchor3.getRawClockOffset();
            ranging_time = anchor3.ds_processRTInfo(t_roundA, t_replyA, anchor3.read(0x12, 0x04), anchor3.read(0x12, 0x08), clock_offset);
            distance3 = anchor3.convertToCM(ranging_time);
        }
    }
    
    // Filtro Kalman para cada distancia (en cm)
    static bool first = true;
    float d1_f = kf_d1.x, d2_f = kf_d2.x, d3_f = kf_d3.x;
    
    if (first) {
        if (!isnan(distance1)) { kf_d1.x = distance1; d1_f = kf_d1.x; }
        if (!isnan(distance2)) { kf_d2.x = distance2; d2_f = kf_d2.x; }
        if (!isnan(distance3)) { kf_d3.x = distance3; d3_f = kf_d3.x; }
        if (!isnan(distance1) && !isnan(distance2) && !isnan(distance3)) first = false;
    } else {
        if (!isnan(distance1)) d1_f = kf_d1.update(distance1);
        if (!isnan(distance2)) d2_f = kf_d2.update(distance2);
        if (!isnan(distance3)) d3_f = kf_d3.update(distance3);
    }
    
    // Calcular posición con trilateración de 3 anchors (en metros)
    if (!isnan(d1_f) && !isnan(d2_f) && !isnan(d3_f)) {
        calculatePosition(d1_f / 100.0, d2_f / 100.0, d3_f / 100.0, tag_x, tag_y);
        return (!isnan(tag_x) && !isnan(tag_y));
    }
    
    return false;
}
