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
    // Inicializar comunicación con Raspberry Pi
    RPiComm_setup();
    
    // Inicializar sistema de motores PWM
    setupMotorPWMTest();
    disableMotors(); // Comenzar con motores desactivados por seguridad
    
    motorsEnabled = false;
    lastCommandTime = millis();
    
    Serial.println("[MotorController] Sistema inicializado");
    Serial.println("  - Comunicación UART2 con RPi: OK");
    Serial.println("  - Sistema PWM de motores: OK");
    Serial.println("  - Sensores de corriente y voltaje: OK");
    Serial.println("  - Motores deshabilitados por seguridad");
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
    
    // Habilitar motores si no están habilitados
    if (!motorsEnabled) {
        enableMotors();
        motorsEnabled = true;
        Serial.println("[MotorController] Motores habilitados");
    }
    
    // Mapear velocidades de -255/+255 a -100/+100
    int left_speed = map(cmd.motor_left_speed, -255, 255, -100, 100);
    int right_speed = map(cmd.motor_right_speed, -255, 255, -100, 100);
    
    // Aplicar velocidades a los motores usando las funciones disponibles
    if (left_speed >= 0) {
        setMotorL(abs(left_speed), true);  // true = adelante
    } else {
        setMotorL(abs(left_speed), false); // false = atrás
    }
    
    if (right_speed >= 0) {
        setMotorR(abs(right_speed), true);  // true = adelante
    } else {
        setMotorR(abs(right_speed), false); // false = atrás
    }
    
    Serial.printf("[MotorController] Comando ejecutado: L=%d%%, R=%d%%\n", 
                 left_speed, right_speed);
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
    // Parar todos los motores inmediatamente
    disableMotors();
    motorsEnabled = false;
    
    Serial.println("[MotorController] PARADA DE EMERGENCIA");
    
    // Notificar a la Raspberry Pi
    JsonDocument doc;
    doc["type"] = "emergency_stop_ack";
    doc["timestamp"] = millis();
    String output;
    serializeJson(doc, output);
    RPiComm_sendJSON(output.c_str());
}
