#include "RPiComm.h"
#include <ArduinoJson.h>

// UART2 para comunicación con Raspberry Pi
HardwareSerial RPiSerial(2);

// Variables internas
static unsigned long lastHeartbeat = 0;
static const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 segundos

void RPiComm_setup() {
    // Configurar UART2 para Raspberry Pi
    RPiSerial.begin(RPI_UART_SPEED, SERIAL_8N1, RPI_UART_RX_PIN, RPI_UART_TX_PIN);
    
    // Configurar pines de sensores como entradas analógicas
    pinMode(CURRENT_L_PIN, INPUT);
    pinMode(CURRENT_R_PIN, INPUT);
    pinMode(BATTERY_V_PIN, INPUT);
    
    // Configurar ADC para mejor precisión
    analogReadResolution(12); // 12-bit resolution (0-4095)
    analogSetAttenuation(ADC_11db); // Para voltajes hasta 3.3V
    
    Serial.println("[RPiComm] UART2 configurado:");
    Serial.printf("  RX: GPIO%d, TX: GPIO%d\n", RPI_UART_RX_PIN, RPI_UART_TX_PIN);
    Serial.printf("  Velocidad: %d bps\n", RPI_UART_SPEED);
    Serial.println("  Sensores configurados:");
    Serial.printf("    Corriente L: GPIO%d\n", CURRENT_L_PIN);
    Serial.printf("    Corriente R: GPIO%d\n", CURRENT_R_PIN);
    Serial.printf("    Batería: GPIO%d\n", BATTERY_V_PIN);
}

void RPiComm_sendSystemData(float tag_x, float tag_y, bool uwb_valid) {
    // Leer sensores
    float current_left = RPiComm_readCurrentSensor(CURRENT_L_PIN);
    float current_right = RPiComm_readCurrentSensor(CURRENT_R_PIN);
    float battery_voltage = RPiComm_readBatteryVoltage();
    
    // Crear JSON con datos del sistema
    JsonDocument doc;
    doc["type"] = "system_data";
    doc["timestamp"] = millis();
    
    // Datos UWB
    doc["uwb"]["valid"] = uwb_valid;
    if (uwb_valid) {
        doc["uwb"]["x"] = tag_x;
        doc["uwb"]["y"] = tag_y;
    } else {
        doc["uwb"]["x"] = nullptr;
        doc["uwb"]["y"] = nullptr;
    }
    
    // Datos de sensores
    doc["sensors"]["current_left"] = current_left;
    doc["sensors"]["current_right"] = current_right;
    doc["sensors"]["battery_voltage"] = battery_voltage;
    
    // Enviar JSON
    String output;
    serializeJson(doc, output);
    RPiSerial.println(output);
    
    // Debug opcional (menos frecuente)
    static unsigned long lastDebugSensor = 0;
    if (millis() - lastDebugSensor > 10000) { // Cada 10 segundos
        Serial.printf("[RPiComm] Sensores - I_L:%.2fA, I_R:%.2fA, V_bat:%.1fV\n", 
                     current_left, current_right, battery_voltage);
        lastDebugSensor = millis();
    }
}

bool RPiComm_receiveCommand(MotorCommand &cmd) {
    if (!RPiSerial.available()) {
        return false;
    }
    
    String line = RPiSerial.readStringUntil('\n');
    line.trim();
    
    if (line.length() == 0) {
        return false;
    }
    
    // Parsear JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, line);
    
    if (error) {
        Serial.printf("[RPiComm] Error parsing JSON: %s\n", error.c_str());
        return false;
    }
    
    // Verificar tipo de comando
    const char* type = doc["type"];
    if (!type) {
        Serial.println("[RPiComm] Missing command type");
        return false;
    }
    
    if (strcmp(type, "motor_command") == 0) {
        cmd.command_type = 'M';
        cmd.motor_left_speed = doc["left_speed"] | 0;
        cmd.motor_right_speed = doc["right_speed"] | 0;
        cmd.emergency_stop = doc["emergency_stop"] | false;
        
        // Limitar velocidades
        cmd.motor_left_speed = constrain(cmd.motor_left_speed, -255, 255);
        cmd.motor_right_speed = constrain(cmd.motor_right_speed, -255, 255);
        
        Serial.printf("[RPiComm] Motor command: L=%d, R=%d, STOP=%d\n", 
                     cmd.motor_left_speed, cmd.motor_right_speed, cmd.emergency_stop);
        return true;
    }
    else if (strcmp(type, "emergency_stop") == 0) {
        cmd.command_type = 'S';
        cmd.motor_left_speed = 0;
        cmd.motor_right_speed = 0;
        cmd.emergency_stop = true;
        
        Serial.println("[RPiComm] EMERGENCY STOP received!");
        return true;
    }
    else if (strcmp(type, "ping") == 0) {
        cmd.command_type = 'P';
        
        // Responder con pong
        JsonDocument response;
        response["type"] = "pong";
        response["timestamp"] = millis();
        String output;
        serializeJson(response, output);
        RPiSerial.println(output);
        
        return false; // No es un comando de motor
    }
    
    return false;
}

float RPiComm_readBatteryVoltage() {
    // Leer ADC múltiples veces para mayor precisión
    long sum = 0;
    const int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        sum += analogRead(BATTERY_V_PIN);
        delay(1);
    }
    
    float adc_average = sum / (float)samples;
    
    // Convertir ADC a voltaje del divisor
    float voltage_divider = (adc_average / ADC_RESOLUTION) * ADC_VREF;
    
    // Calcular voltaje real de la batería
    // V_battery = V_divider * (R1 + R2) / R2
    float battery_voltage = voltage_divider * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    
    return battery_voltage;
}

float RPiComm_readCurrentSensor(int pin) {
    // Leer ADC múltiples veces para mayor precisión
    long sum = 0;
    const int samples = 5;
    
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delay(1);
    }
    
    float adc_average = sum / (float)samples;
    
    // Convertir ADC a voltaje
    float voltage = (adc_average / ADC_RESOLUTION) * ADC_VREF;
    
    // Convertir voltaje a corriente (depende del sensor)
    // Ejemplo para sensor ACS712 (5A): 2.5V = 0A, 185mV/A
    // Ajustar según tu sensor específico
    float current = (voltage - 2.5) / 0.185; // Para ACS712-5A
    
    return abs(current); // Retornar valor absoluto
}

void RPiComm_sendHeartbeat() {
    unsigned long now = millis();
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        JsonDocument doc;
        doc["type"] = "heartbeat";
        doc["timestamp"] = now;
        doc["uptime"] = now;
        doc["free_heap"] = ESP.getFreeHeap();
        
        String output;
        serializeJson(doc, output);
        RPiSerial.println(output);
        
        lastHeartbeat = now;
    }
}

void RPiComm_sendJSON(const char* json) {
    RPiSerial.println(json);
}

bool RPiComm_available() {
    return RPiSerial.available() > 0;
}
