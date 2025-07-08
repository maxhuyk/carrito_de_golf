#pragma once

// Configuración del número de anchors (2 o 3)
#define NUM_ANCHORS 3  // Cambiar a 2 para usar solo 2 anchors

// Filtro de Kalman 1D simple
class KalmanFilter1D {
  public:
    float q; // Ruido del proceso
    float r; // Ruido de medición
    float x; // Estado estimado
    float p; // Varianza estimada
    KalmanFilter1D(float q_, float r_, float x0 = 0, float p0 = 1) : q(q_), r(r_), x(x0), p(p0) {}
    float update(float measurement) {
        // Predicción
        p += q;
        // Ganancia de Kalman
        float k = p / (p + r);
        // Actualización
        x = x + k * (measurement - x);
        p = (1 - k) * p;
        return x;
    }
};

// Inicializa el sistema UWB (debe llamarse en setup)
void UWBManager_setup();

// Realiza una medición, filtra y calcula la posición (debe llamarse periódicamente)
// Devuelve true si la posición es válida, false si no.
bool UWBManager_update(float &tag_x, float &tag_y);
