#include <Arduino.h>
#include "UWBManager.h"
#include "UWBCore.h"
#include <math.h>
#include <algorithm>

// Parámetros del filtro de Kalman (ajusta aquí)
static float KALMAN_Q = 0.5f; // Ruido del proceso (mayor si el tag se mueve rápido)
static float KALMAN_R = 500.0f; // Ruido de medición (mayor si el UWB es ruidoso)
static float KALMAN_P0 = 10.0f; // Varianza inicial

// Instancias de filtro de Kalman para cada distancia
static KalmanFilter1D kf_d1(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);
static KalmanFilter1D kf_d2(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);
static KalmanFilter1D kf_d3(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);

// Variables globales para almacenar las últimas distancias medidas
static float g_distances[NUM_ANCHORS] = {NAN, NAN, NAN};
static bool g_anchor_status[NUM_ANCHORS] = {false, false, false};

void UWBManager_setup() {
    // Inicializar el core UWB (Core 0)
    UWBCore_setup();
    
    // Iniciar la tarea UWB en Core 0
    UWBCore_startTask();
    
    Serial.println("[UWBManager] Dual-core UWB system initialized");
}

static void calculatePosition(float d1, float d2, float d3, float &x, float &y) {
    // Posiciones de los anchors (en metros) - Configuración de prueba
    float x1 = 0.0, y1 = 0.0;    // Anchor 1 en origen
    float x2 = 1.0, y2 = 0.0;    // Anchor 2 a 1m en X
    float x3 = 0.5, y3 = 0.866;  // Anchor 3 formando triángulo equilátero
    
    static float last_x = NAN, last_y = NAN;
    
    // Método de mínimos cuadrados para trilateración
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
    
    // Validación más permisiva de la solución
    float dist_check1 = sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1));
    float dist_check2 = sqrt((x-x2)*(x-x2) + (y-y2)*(y-y2));
    float dist_check3 = sqrt((x-x3)*(x-x3) + (y-y3)*(y-y3));
    
    float error1 = abs(dist_check1 - d1);
    float error2 = abs(dist_check2 - d2);
    float error3 = abs(dist_check3 - d3);
    
    const float TOLERANCE = 0.3; // 30cm de tolerancia - más permisivo
    
    if (error1 > TOLERANCE || error2 > TOLERANCE || error3 > TOLERANCE) {
        // Si no hay posición previa válida, usar la calculada de todas formas
        if (isnan(last_x) || isnan(last_y)) {
            last_x = x;
            last_y = y;
        } else {
            x = last_x;
            y = last_y;
        }
        return;
    }
    
    last_x = x;
    last_y = y;
}

bool UWBManager_update(float &tag_x, float &tag_y) {
    // Obtener datos raw del Core 0 (UWB) - siempre disponibles
    UWBRawData rawData;
    UWBCore_getRawData(rawData);
    
    // Extraer distancias del Core 0
    float distance1 = rawData.distance1;
    float distance2 = rawData.distance2;
    float distance3 = rawData.distance3;
    
    // Almacenar distancias y estado de los anchors
    g_distances[0] = distance1;
    g_distances[1] = distance2;
    g_distances[2] = distance3;
    g_anchor_status[0] = rawData.valid1;
    g_anchor_status[1] = rawData.valid2;
    g_anchor_status[2] = rawData.valid3;
    
    // Filtro Kalman para cada distancia (en cm)
    static bool first = true;
    float d1_f = kf_d1.x, d2_f = kf_d2.x, d3_f = kf_d3.x;
    
    if (first) {
        if (rawData.valid1) { kf_d1.x = distance1; d1_f = kf_d1.x; }
        if (rawData.valid2) { kf_d2.x = distance2; d2_f = kf_d2.x; }
        if (rawData.valid3) { kf_d3.x = distance3; d3_f = kf_d3.x; }
        if (rawData.valid1 && rawData.valid2 && rawData.valid3) first = false;
    } else {
        if (rawData.valid1) d1_f = kf_d1.update(distance1);
        if (rawData.valid2) d2_f = kf_d2.update(distance2);
        if (rawData.valid3) d3_f = kf_d3.update(distance3);
    }
    
    // Calcular posición con trilateración de 3 anchors (en metros)
    bool result = false;
    if (rawData.valid1 && rawData.valid2 && rawData.valid3) {
        calculatePosition(d1_f / 100.0, d2_f / 100.0, d3_f / 100.0, tag_x, tag_y);
        result = (!isnan(tag_x) && !isnan(tag_y));
    }
    
    return result;
}

void UWBManager_getDistances(float distances[NUM_ANCHORS]) {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        distances[i] = g_distances[i];
    }
}

void UWBManager_getAnchorStatus(bool status[NUM_ANCHORS]) {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        status[i] = g_anchor_status[i];
    }
}

float UWBManager_getMeasurementFrequency() {
    return UWBCore_getMeasurementFrequency(); // Usar la frecuencia del Core 0
}

unsigned long UWBManager_getMeasurementCount() {
    return g_uwb_measurement_count;
}
