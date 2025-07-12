#pragma once

class KalmanFilter1D {
public:
    float Q; // Ruido del proceso
    float R; // Ruido de medición
    float x; // Estado estimado
    float P; // Varianza estimada

    KalmanFilter1D(float q, float r, float x0, float p0) : Q(q), R(r), x(x0), P(p0) {}

    float update(float measurement) {
        // Predicción
        P += Q;
        // Ganancia de Kalman
        float K = P / (P + R);
        // Actualización
        x = x + K * (measurement - x);
        P = (1 - K) * P;
        return x;
    }
};
