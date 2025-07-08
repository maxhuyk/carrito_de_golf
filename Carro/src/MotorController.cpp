#include "MotorController.h"
#include "MotorPWMTest.h"
#include "RPiComm.h"
#include <ArduinoJson.h>

// Variables de estado
static bool motorsEnabled = false;
static unsigned long lastCommandTime = 0;
static const unsigned long COMMAND_TIMEOUT = 2000; // 2 segundos

// Declaración de función
void executeMotorCommand(const MotorCommand& cmd);

void setupMotorController() {
    RPiComm_setup();
    setupMotorPWMTest();
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
    
    if (!motorsEnabled) {
        enableMotors();
        motorsEnabled = true;
    }
    
    // Mapear velocidades de -255/+255 a -100/+100
    int left_speed = map(cmd.motor_left_speed, -255, 255, -100, 100);
    int right_speed = map(cmd.motor_right_speed, -255, 255, -100, 100);
    
    // Aplicar velocidades a los motores
    if (left_speed >= 0) {
        setMotorL(abs(left_speed), true);
    } else {
        setMotorL(abs(left_speed), false);
    }
    
    if (right_speed >= 0) {
        setMotorR(abs(right_speed), true);
    } else {
        setMotorR(abs(right_speed), false);
    }
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
#define SYSTEM_CURRENT_PIN  35    // ADC1_CH7

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

float getTotalCurrent() {
    // Leer corriente total del sistema
    int rawValue = analogRead(SYSTEM_CURRENT_PIN);
    float voltage = (rawValue / ADC_RESOLUTION) * ADC_REFERENCE;
    
    // Convertir a corriente
    float current = (voltage - 2.5) / CURRENT_SCALE;
    
    return abs(current);
}
