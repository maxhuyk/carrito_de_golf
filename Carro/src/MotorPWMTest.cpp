#include <Arduino.h>

// Pines para los DRV8701 (ambos motores)
#define ENABLE_MOTORS 14  // Pin único de enable para ambos motores
#define PWML1 26
#define PWML2 25
#define CURRL 36  // SENSOR_VP
// Pines para el DRV8701 (motor derecho)
#define PWMR1 33
#define PWMR2 32
#define CURRR 39  // SENSOR_VN

// --- PWM ESP32 ---
#define PWM_FREQ 20000
#define PWM_RES 8
#define CH_L1 0
#define CH_L2 1
#define CH_R1 2
#define CH_R2 3

static bool pwm_initialized = false;
typedef enum {TEST_IDLE, TEST_L_FWD, TEST_L_REV, TEST_R_FWD, TEST_R_REV} TestState;
static TestState testState = TEST_IDLE;
static unsigned long lastChange = 0;

// Función para desactivar completamente los motores
void disableMotors() {
    digitalWrite(ENABLE_MOTORS, LOW);
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
    Serial.println("[PWM] Motores desactivados completamente");
}

// Función para activar los motores
void enableMotors() {
    digitalWrite(ENABLE_MOTORS, HIGH);
    Serial.println("[PWM] Motores activados");
}

// pwm: 0 a 100, dir: true=adelante, false=atrás
void setMotorL(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 100);
    int duty = map(pwm, 0, 100, 0, 255);
    
    // Solo activar si hay PWM, sino mantener el estado actual
    if (pwm > 0) {
        digitalWrite(ENABLE_MOTORS, HIGH);
    }
    
    Serial.printf("[PWM] Izq: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)\n", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
    
    if (dir) {
        ledcWrite(CH_L1, duty);
        ledcWrite(CH_L2, 0);
    } else {
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, duty);
    }
}

void setMotorR(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 100);
    int duty = map(pwm, 0, 100, 0, 255);
    
    // Solo activar si hay PWM, sino mantener el estado actual
    if (pwm > 0) {
        digitalWrite(ENABLE_MOTORS, HIGH);
    }
    
    Serial.printf("[PWM] Der: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)\n", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
    
    if (dir) {
        ledcWrite(CH_R1, duty);
        ledcWrite(CH_R2, 0);
    } else {
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, duty);
    }
}

// Inicializa los pines de los drivers
void setupMotorPWMTest() {
    Serial.println("[PWM] Inicializando pines y PWM...");
    pinMode(ENABLE_MOTORS, OUTPUT);
    pinMode(PWML1, OUTPUT);
    pinMode(PWML2, OUTPUT);
    pinMode(PWMR1, OUTPUT);
    pinMode(PWMR2, OUTPUT);
    digitalWrite(ENABLE_MOTORS, LOW);
    digitalWrite(PWML1, LOW);
    digitalWrite(PWML2, LOW);
    digitalWrite(PWMR1, LOW);
    digitalWrite(PWMR2, LOW);
    if (!pwm_initialized) {
        uint32_t freqL1 = ledcSetup(CH_L1, PWM_FREQ, PWM_RES);
        uint32_t freqL2 = ledcSetup(CH_L2, PWM_FREQ, PWM_RES);
        uint32_t freqR1 = ledcSetup(CH_R1, PWM_FREQ, PWM_RES);
        uint32_t freqR2 = ledcSetup(CH_R2, PWM_FREQ, PWM_RES);
        ledcAttachPin(PWML1, CH_L1);
        ledcAttachPin(PWML2, CH_L2);
        ledcAttachPin(PWMR1, CH_R1);
        ledcAttachPin(PWMR2, CH_R2);
        pwm_initialized = true;
        Serial.printf("[PWM] ledcSetup L1: %lu Hz\n", freqL1);
        Serial.printf("[PWM] ledcSetup L2: %lu Hz\n", freqL2);
        Serial.printf("[PWM] ledcSetup R1: %lu Hz\n", freqR1);
        Serial.printf("[PWM] ledcSetup R2: %lu Hz\n", freqR2);
        if (freqL1 == PWM_FREQ && freqL2 == PWM_FREQ && freqR1 == PWM_FREQ && freqR2 == PWM_FREQ)
            Serial.println("[PWM] PWM inicializado correctamente");
        else
            Serial.println("[PWM] ADVERTENCIA: alguna frecuencia PWM no coincide");
    }
    // --- Test manual inmediato: ambos motores adelante 2s, atrás 2s ---
    Serial.println("[TEST] Motores adelante 2s");
    setMotorL(100, true);
    setMotorR(100, true);
    delay(2000);
    Serial.println("[TEST] Motores atrás 2s");
    setMotorL(100, false);
    setMotorR(100, false);
    delay(2000);
    Serial.println("[TEST] Motores en reposo");
    setMotorL(0, true);
    setMotorR(0, true);
}

// Prueba: alterna adelante y atrás en ambos motores
void motorPWMTestLoop() {
    unsigned long now = millis();
    static TestState lastState = TEST_IDLE;
    if (testState != lastState) {
        switch (testState) {
            case TEST_IDLE:
                Serial.println("[MOTOR] Ambos motores en reposo");
                disableMotors();  // Desactivar completamente en reposo
                break;
            case TEST_L_FWD:
                Serial.println("[MOTOR] Izquierdo ADELANTE (80%), Derecho REPOSO");
                enableMotors();   // Activar antes de usar
                break;
            case TEST_L_REV:
                Serial.println("[MOTOR] Izquierdo ATRÁS (80%), Derecho REPOSO");
                break;
            case TEST_R_FWD:
                Serial.println("[MOTOR] Derecho ADELANTE (80%), Izquierdo REPOSO");
                break;
            case TEST_R_REV:
                Serial.println("[MOTOR] Derecho ATRÁS (80%), Izquierdo REPOSO");
                break;
        }
        lastState = testState;
    }
    switch (testState) {
        case TEST_IDLE:
            // Ya se desactivaron arriba, no hacer nada más
            if (now - lastChange > 1000) { testState = TEST_L_FWD; lastChange = now; }
            break;
        case TEST_L_FWD:
            setMotorL(80, true);
            setMotorR(0, true);
            if (now - lastChange > 2000) { testState = TEST_L_REV; lastChange = now; }
            break;
        case TEST_L_REV:
            setMotorL(80, false);
            setMotorR(0, true);
            if (now - lastChange > 2000) { testState = TEST_R_FWD; lastChange = now; }
            break;
        case TEST_R_FWD:
            setMotorL(0, true);
            setMotorR(80, true);
            if (now - lastChange > 2000) { testState = TEST_R_REV; lastChange = now; }
            break;
        case TEST_R_REV:
            setMotorL(0, true);
            setMotorR(80, false);
            if (now - lastChange > 2000) { testState = TEST_IDLE; lastChange = now; }
            break;
    }
}
