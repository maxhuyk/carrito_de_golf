#include "MotorController.h"
#include "RPiComm.h"
#include <ArduinoJson.h>

// ========== MOTOR PWM DEFINITIONS ==========
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
#define PWM_FREQ 20000  // 20kHz - frecuencia original que funcionaba bien
#define PWM_RES 8       // 8 bits = 0-255
#define CH_L1 0
#define CH_L2 1
#define CH_R1 2
#define CH_R2 3

// Variables de estado
static bool motorsEnabled = false;
static bool pwm_initialized = false;
static unsigned long lastCommandTime = 0;
static const unsigned long COMMAND_TIMEOUT = 2000; // 2 segundos

// Declaración de función
void executeMotorCommand(const MotorCommand& cmd);

// ========== MOTOR PWM BASE FUNCTIONS ==========
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

// pwm: 0 a 100, dir: true=adelante, false=atrás (COMO EL TEST ORIGINAL)
void setMotorL(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 100);
    int duty = map(pwm, 0, 100, 0, 255);
    digitalWrite(ENABLE_MOTORS, HIGH);  // Enable SIEMPRE activo como en el test
    Serial.printf("[PWM] Izq: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)\n", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
    if (dir) {
        ledcWrite(CH_L1, duty);
        ledcWrite(CH_L2, 0);
    } else {
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, duty);
    }
}

// NUEVA: Función directa sin mapeo - pwm: 0 a 255 (SIN UMBRALES)
void setMotorL_direct(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(ENABLE_MOTORS, HIGH);  // Enable SIEMPRE activo
    Serial.printf("[MOTOR_L_DIRECT] PWM=%d, Dir=%s\n", pwm, dir ? "FWD" : "REV");
    
    if (dir) {
        ledcWrite(CH_L1, pwm);
        ledcWrite(CH_L2, 0);
    } else {
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, pwm);
    }
}

void setMotorR(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 100);
    int duty = map(pwm, 0, 100, 0, 255);
    digitalWrite(ENABLE_MOTORS, HIGH);  // Enable SIEMPRE activo como en el test
    Serial.printf("[PWM] Der: ENABLE=%d, DIR=%s, PWM=%d (duty=%d)\n", digitalRead(ENABLE_MOTORS), dir ? "FWD" : "REV", pwm, duty);
    if (dir) {
        ledcWrite(CH_R1, duty);
        ledcWrite(CH_R2, 0);
    } else {
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, duty);
    }
}

// NUEVA: Función directa sin mapeo - pwm: 0 a 255 (SIN UMBRALES)
void setMotorR_direct(int pwm, bool dir) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(ENABLE_MOTORS, HIGH);  // Enable SIEMPRE activo
    Serial.printf("[MOTOR_R_DIRECT] PWM=%d, Dir=%s\n", pwm, dir ? "FWD" : "REV");
    
    if (dir) {
        ledcWrite(CH_R1, pwm);
        ledcWrite(CH_R2, 0);
    } else {
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, pwm);
    }
}

// Inicializa los pines de los drivers (sin test)
void setupMotorPWM() {
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
        
        if (freqL1 == PWM_FREQ && freqL2 == PWM_FREQ && freqR1 == PWM_FREQ && freqR2 == PWM_FREQ) {
            Serial.println("[PWM] PWM inicializado correctamente");
        } else {
            Serial.println("[PWM] ADVERTENCIA: alguna frecuencia PWM no coincide");
        }
    }
}

// ========== MOTOR CONTROLLER FUNCTIONS ==========

void setupMotorController() {
    RPiComm_setup();
    setupMotorPWM();
    disableMotors();
    
    motorsEnabled = false;
    lastCommandTime = millis();
    
    Serial.println("[MotorController] Initialized");
}

void processSerialCommands() {
    MotorCommand cmd;
    
    // Verificar si hay comandos de la Raspberry Pi
    if (RPiComm_receiveCommand(cmd)) {
        lastCommandTime = millis();
        
        if (cmd.command_type == 'M') {
            // Comando de motor
            executeMotorCommand(cmd);
        }
        else if (cmd.command_type == 'S') {
            // Parada de emergencia
            emergencyStop();
        }
    }
    
    // Timeout de seguridad: si no hay comunicación, parar motores
    if (motorsEnabled && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
        emergencyStop();
        Serial.println("[MotorController] TIMEOUT: Sin comunicación con RPi, parando motores");
    }
    
    // Enviar heartbeat periódico
    RPiComm_sendHeartbeat();
}

void sendUWBData(float x, float y, bool valid) {
    // Enviar datos UWB junto con sensores a la Raspberry Pi
    RPiComm_sendSystemData(x, y, valid);
}

void executeMotorCommand(const MotorCommand& cmd) {
    if (cmd.emergency_stop) {
        emergencyStop();
        return;
    }
    
    // CONVERTIR valores de GUI (-255 a +255) con zona muerta real
    int left_speed = constrain(cmd.motor_left_speed, -255, 255);
    int right_speed = constrain(cmd.motor_right_speed, -255, 255);
    
    // DEBUG
    Serial.printf("[MOTOR_CMD] GUI values: L=%d, R=%d\n", left_speed, right_speed);
    
    // Definir zona muerta real (PWM duty mínimo para moverse)
    const int DEADZONE_PWM = 60; // duty mínimo real (0-255)
    // Mapeo: GUI 0 -> 0, GUI 1-255 -> 60-255 (duty)
    int left_pwm, right_pwm;
    if (abs(left_speed) == 0) {
        left_pwm = 0;
    } else {
        left_pwm = map(abs(left_speed), 1, 255, DEADZONE_PWM, 100); // 100 en escala 0-100
        if (left_pwm < DEADZONE_PWM) left_pwm = DEADZONE_PWM;
    }
    if (abs(right_speed) == 0) {
        right_pwm = 0;
    } else {
        right_pwm = map(abs(right_speed), 1, 255, DEADZONE_PWM, 100);
        if (right_pwm < DEADZONE_PWM) right_pwm = DEADZONE_PWM;
    }
    bool left_dir = (left_speed >= 0);
    bool right_dir = (right_speed >= 0);
    Serial.printf("[MOTOR_CMD] Converted: L_PWM=%d(dir=%s), R_PWM=%d(dir=%s)\n", 
                 left_pwm, left_dir ? "FWD" : "REV", 
                 right_pwm, right_dir ? "FWD" : "REV");
    setMotorL(left_pwm, left_dir);
    setMotorR(right_pwm, right_dir);
    motorsEnabled = true;
}

void setMotorSpeed(uint8_t motor_id, int8_t speed) {
    // Limitar velocidad
    speed = constrain(speed, -100, 100);
    
    if (motor_id == 0) {
        // Motor izquierdo
        if (speed >= 0) {
            setMotorL(speed, true);  // true = adelante
        } else {
            setMotorL(abs(speed), false); // false = atrás
        }
    }
    else if (motor_id == 1) {
        // Motor derecho  
        if (speed >= 0) {
            setMotorR(speed, true);  // true = adelante
        } else {
            setMotorR(abs(speed), false); // false = atrás
        }
    }
}

void emergencyStop() {
    disableMotors();
    motorsEnabled = false;
    
    // Notificar a la Raspberry Pi
    JsonDocument doc;
    doc["type"] = "emergency_stop_ack";
    doc["timestamp"] = millis();
    String output;
    serializeJson(doc, output);
    RPiComm_sendJSON(output.c_str());
}

// ========== FUNCIONES DE MONITOREO ==========

// Pines ADC para sensores (ajustar según tu hardware)
#define BATTERY_VOLTAGE_PIN 36    // ADC1_CH0 (VP)
#define MOTOR_L_CURRENT_PIN 39    // ADC1_CH3 (VN) 
#define MOTOR_R_CURRENT_PIN 34    // ADC1_CH6

// Constantes de calibración (ajustar según tu hardware)
#define BATTERY_VOLTAGE_SCALE 0.01611  // Escalado para divisor de voltaje (12V -> 3.3V)
#define CURRENT_SCALE 0.066            // mV/A del sensor de corriente (ej: ACS712-30A)
#define ADC_RESOLUTION 4095.0          // 12-bit ADC
#define ADC_REFERENCE 3.3              // Voltaje de referencia

float getBatteryVoltage() {
    // Leer voltaje de batería a través de divisor de voltaje
    int rawValue = analogRead(BATTERY_VOLTAGE_PIN);
    float voltage = (rawValue / ADC_RESOLUTION) * ADC_REFERENCE;
    
    // Aplicar escalado del divisor de voltaje (ej: 12V -> 3.3V con divisor 4:1)
    voltage = voltage / BATTERY_VOLTAGE_SCALE;
    
    return voltage;
}

float getMotorCurrent(uint8_t motor_id) {
    int pin = (motor_id == 0) ? MOTOR_L_CURRENT_PIN : MOTOR_R_CURRENT_PIN;
    
    // Leer valor ADC del sensor de corriente
    int rawValue = analogRead(pin);
    float voltage = (rawValue / ADC_RESOLUTION) * ADC_REFERENCE;
    
    // Convertir voltaje a corriente usando la sensibilidad del sensor
    // ACS712: 2.5V = 0A, cada 66mV = 1A (para ACS712-30A)
    float current = (voltage - 2.5) / CURRENT_SCALE;
    
    // Valor absoluto para corriente (dirección no importa aquí)
    return abs(current);
}
