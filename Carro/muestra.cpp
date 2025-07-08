#include "DW3000.h"
#include <Arduino.h>
#include "UWBManager.h"
#include <math.h>
#include <algorithm>

#define ANCHOR1_CS 4
#define ANCHOR1_RST 14
#define ANCHOR1_IRQ 35
#define ANCHOR2_CS 5
#define ANCHOR2_RST 27
#define ANCHOR2_IRQ 34

// Parámetros del filtro de Kalman (ajusta aquí)
static float KALMAN_Q = 0.5f; // Ruido del proceso (mayor si el tag se mueve rápido)
static float KALMAN_R = 500.0f; // Ruido de medición (mayor si el UWB es ruidoso) era 8
static float KALMAN_P0 = 10.0f; // Varianza inicial

static DW3000Class anchor1(ANCHOR1_CS, ANCHOR1_RST, ANCHOR1_IRQ);
static DW3000Class anchor2(ANCHOR2_CS, ANCHOR2_RST, ANCHOR2_IRQ);

// Instancias de filtro de Kalman para cada distancia
static KalmanFilter1D kf_d1(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);
static KalmanFilter1D kf_d2(KALMAN_Q, KALMAN_R, 0, KALMAN_P0);

void UWBManager_setup() {
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
}

static void calculatePosition(float d1, float d2, float &x, float &y) {
    float x1 = 0.0, y1 = 0.0;
    float x2 = 0.3, y2 = 0.0;
    float D = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    const float TOL = 0.01;
    static float last_x = NAN, last_y = NAN;
    if (D > d1 + d2 + TOL || D < fabs(d1 - d2) - TOL) {
        x = last_x;
        y = last_y;
        return;
    }
    float a = (d1*d1 - d2*d2 + D*D) / (2*D);
    float h2 = d1*d1 - a*a;
    if (h2 < -TOL) {
        x = last_x;
        y = last_y;
        return;
    }
    float h = (h2 < 0) ? 0 : sqrt(h2);
    float ex = (x2 - x1) / D;
    float ey = (y2 - y1) / D;
    x = x1 + a * ex - h * ey;
    y = y1 + a * ey + h * ex;
    last_x = x;
    last_y = y;
}

bool UWBManager_update(float &tag_x, float &tag_y) {
    // Medición Anchor 1
    float distance1 = NAN, distance2 = NAN;
    int t_roundA = 0, t_replyA = 0, clock_offset = 0, ranging_time = 0;
    long long rx = 0, tx = 0;
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
    // Filtro Kalman para cada distancia (en cm) SIN mediana ni outlier
    static bool first = true;
    float d1_f = kf_d1.x, d2_f = kf_d2.x;
    if (first) {
        if (!isnan(distance1)) { kf_d1.x = distance1; d1_f = kf_d1.x; }
        if (!isnan(distance2)) { kf_d2.x = distance2; d2_f = kf_d2.x; }
        if (!isnan(distance1) && !isnan(distance2)) first = false;
    } else {
        if (!isnan(distance1)) d1_f = kf_d1.update(distance1);
        if (!isnan(distance2)) d2_f = kf_d2.update(distance2);
    }
    // Calcular posición (en metros)
    calculatePosition(d1_f / 100.0, d2_f / 100.0, tag_x, tag_y);
    return (!isnan(tag_x) && !isnan(tag_y));
}
