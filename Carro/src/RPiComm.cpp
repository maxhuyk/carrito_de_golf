#include "RPiComm.h"
#include "MotorController.h"
#include <ArduinoJson.h>

// UART2 para comunicación con Raspberry Pi
HardwareSerial RPiSerial(2);

// Variables internas
static unsigned long lastHeartbeat = 0;
static const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 segundos

void RPiComm_setup() {
    RPiSerial.begin(RPI_UART_SPEED, SERIAL_8N1, RPI_UART_RX_PIN, RPI_UART_TX_PIN);
    
    // Configurar pines ADC específicamente
    pinMode(CURRENT_L_PIN, INPUT);
    pinMode(CURRENT_R_PIN, INPUT);
    pinMode(BATTERY_V_PIN, INPUT);
    
    // Configuración ADC más específica
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Verificar que los pines ADC son válidos
    Serial.printf("[RPiComm] Configurando ADC - Pin corriente L: %d, R: %d, Batería: %d\n", 
                 CURRENT_L_PIN, CURRENT_R_PIN, BATTERY_V_PIN);
    
    // Test inicial de lecturas ADC
    delay(100);
    int test_adc_l = analogRead(CURRENT_L_PIN);
    int test_adc_r = analogRead(CURRENT_R_PIN);
    int test_adc_bat = analogRead(BATTERY_V_PIN);
    
    Serial.printf("[RPiComm] Test ADC inicial - L:%d, R:%d, Bat:%d (rango esperado: 100-4000)\n", 
                 test_adc_l, test_adc_r, test_adc_bat);
    
    if (test_adc_l == 0 && test_adc_r == 0) {
        Serial.println("[RPiComm] WARNING: Sensores de corriente parecen desconectados (ADC=0)");
    }
    
    Serial.println("[RPiComm] UART2 ready");
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
}

void RPiComm_sendUWBData(float distances[3], bool anchor_status[3], 
                        float tag_x, float tag_y, bool pos_valid, 
                        float frequency, unsigned long count) {
    // Crear JSON con datos UWB completos
    JsonDocument doc;
    doc["type"] = "uwb_data";
    doc["timestamp"] = millis();
    
    // Datos de distancias raw de cada anchor
    if (anchor_status[0] && !isnan(distances[0])) {
        doc["distances"]["a1"] = distances[0];
    } else {
        doc["distances"]["a1"] = nullptr;
    }
    
    if (anchor_status[1] && !isnan(distances[1])) {
        doc["distances"]["a2"] = distances[1];
    } else {
        doc["distances"]["a2"] = nullptr;
    }
    
    if (anchor_status[2] && !isnan(distances[2])) {
        doc["distances"]["a3"] = distances[2];
    } else {
        doc["distances"]["a3"] = nullptr;
    }
    
    // Estado de cada anchor
    doc["anchor_status"]["a1"] = anchor_status[0];
    doc["anchor_status"]["a2"] = anchor_status[1];
    doc["anchor_status"]["a3"] = anchor_status[2];
    
    // Posición calculada por ESP32
    doc["position"]["valid"] = pos_valid;
    if (pos_valid) {
        doc["position"]["x"] = tag_x;
        doc["position"]["y"] = tag_y;
    } else {
        doc["position"]["x"] = nullptr;
        doc["position"]["y"] = nullptr;
    }
    
    // Estadísticas de rendimiento - asegurar que frequency no sea NAN
    doc["stats"]["frequency"] = isnan(frequency) ? 0.0 : frequency;
    doc["stats"]["count"] = count;
    
    // Enviar JSON
    String output;
    serializeJson(doc, output);
    RPiSerial.println(output);
}

void RPiComm_sendRawUWBData(float distances[3], bool anchor_status[3], 
                           float frequency, unsigned long count) {
    // Crear JSON completo con UWB + datos del sistema
    JsonDocument doc;
    doc["type"] = "system_data";
    doc["timestamp"] = millis();
    
    // === DATOS UWB ===
    // Distancias raw de cada anchor (en cm) - convertir NAN a null
    if (anchor_status[0] && !isnan(distances[0])) {
        doc["uwb"]["d1"] = distances[0];
    } else {
        doc["uwb"]["d1"] = nullptr;
    }
    
    if (anchor_status[1] && !isnan(distances[1])) {
        doc["uwb"]["d2"] = distances[1];
    } else {
        doc["uwb"]["d2"] = nullptr;
    }
    
    if (anchor_status[2] && !isnan(distances[2])) {
        doc["uwb"]["d3"] = distances[2];
    } else {
        doc["uwb"]["d3"] = nullptr;
    }
    
    // Estado de conexión de cada anchor
    doc["uwb"]["s1"] = anchor_status[0];
    doc["uwb"]["s2"] = anchor_status[1];
    doc["uwb"]["s3"] = anchor_status[2];
    
    // Frecuencia de medición del Core 0 (eliminado por rendimiento)
    doc["uwb"]["freq"] = 0.0;  // Siempre 0.0, no NAN
    doc["uwb"]["count"] = count;
    
    // === DATOS DEL SISTEMA ===
    // Voltaje de batería - verificar que no sea NAN
    float battery_v = RPiComm_readBatteryVoltage();
    doc["power"]["battery_v"] = isnan(battery_v) ? 0.0 : battery_v;
    
    // Corrientes de motores - verificar que no sean NAN
    float motor_l_current = RPiComm_readCurrentSensor(CURRENT_L_PIN);
    float motor_r_current = RPiComm_readCurrentSensor(CURRENT_R_PIN);
    doc["power"]["motor_l_a"] = isnan(motor_l_current) ? 0.0 : motor_l_current;  // Motor izquierdo
    doc["power"]["motor_r_a"] = isnan(motor_r_current) ? 0.0 : motor_r_current;  // Motor derecho
    
    // Enviar JSON completo
    String output;
    serializeJson(doc, output);
    RPiSerial.println(output);
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
        return false;
    }
    
    const char* type = doc["type"];
    if (!type) {
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
        
        return true;
    }
    else if (strcmp(type, "emergency_stop") == 0) {
        cmd.command_type = 'S';
        cmd.motor_left_speed = 0;
        cmd.motor_right_speed = 0;
        cmd.emergency_stop = true;
        
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
    
    // DEBUG: Imprimir valores ADC
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {  // Debug cada 5 segundos
        Serial.printf("[BATTERY] ADC raw: %.1f, ", adc_average);
        lastDebug = millis();
    }
    
    // Convertir ADC a voltaje del divisor
    float voltage_divider = (adc_average / ADC_RESOLUTION) * ADC_VREF;
    
    // Calcular voltaje real de la batería
    // V_battery = V_divider * (R1 + R2) / R2
    float battery_voltage = voltage_divider * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    
    // DEBUG: Continuar debug
    if (millis() - lastDebug < 100) {  // Solo si acabamos de hacer debug arriba
        Serial.printf("V_div: %.3fV, V_bat: %.2fV\n", voltage_divider, battery_voltage);
    }
    
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
    
    // Convertir voltaje a corriente para DRV8701
    const float AV = 20.0;          // Ganancia del amplificador de shunt
    const float V_OFF = 1.65;       // Voltaje de offset teórico
    const float R_SENSE = 0.005;    // Resistencia de sensado en Ohms (5 miliohms)
    
    // Calcular corriente con fórmula original
    float current = (voltage - V_OFF) / (AV * R_SENSE);
    
    // SIMPLE: Sumar 16.5A para corregir offset (como pediste)
    current = current + 16.5;
    
    // DEBUG: Imprimir valores cada 5 segundos
    static unsigned long lastCurrentDebug = 0;
    if (millis() - lastCurrentDebug > 5000) {
        Serial.printf("[CURRENT] Pin=%d, ADC=%.1f, V=%.3f -> I=%.3fA\n", 
                     pin, adc_average, voltage, current);
        lastCurrentDebug = millis();
    }
    
    // Filtrar valores negativos y limitar máximo
    if (current < 0.0) current = 0.0;
    if (current > 25.0) current = 25.0;
    
    return current;
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
