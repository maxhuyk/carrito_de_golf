#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include "DW3000.h"
#include "UWBManager.h"
#include "MotorController.h"
#include "RPiComm.h"
#include "WiFiOTA.h"

void setup() {
    Serial.begin(2000000);
    delay(1000);
    
    Serial.println("=== Golf Cart UWB System Starting ===");
    
    // Inicializar I2C
    Wire.begin(5, 4); // SDA=GPIO5, SCL=GPIO4
    Wire.setClock(500000);
    
    // Inicializar WiFi y OTA
    WiFiOTA_setup();
    
    // Inicializar sistemas UWB (ahora dual-core)
    if (!DW3000Class::initMCP23008()) {
        Serial.println("ERROR: MCP23008 initialization failed");
        return;
    }
    
    // UWBManager_setup ahora inicializa el Core 0 y arranca la tarea UWB
    UWBManager_setup();
    
    // Ejecutar diagnóstico inicial después de un breve delay para estabilización
    delay(2000);
    Serial.println("Ejecutando diagnóstico inicial...");
    UWBManager_runDiagnostics();
    
    setupMotorController();
    
    Serial.println("=== Dual-Core UWB System Ready ===");
    Serial.println("Core 0: UWB measurements (high-speed)");
    Serial.println("Core 1: Processing, WiFi, motors, display");
}

void loop() {
    // Manejar WiFi y OTA
    WiFiOTA_loop();
    
    // Procesar comandos de la Raspberry Pi
    processSerialCommands();
    
    // Procesar comandos de diagnóstico desde Serial Monitor
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command == "diag" || command == "diagnostic") {
            UWBManager_runDiagnostics();
        } else if (command == "sensors") {
            // Diagnóstico de sensores
            Serial.println("\n=== DIAGNÓSTICO DE SENSORES ===");
            
            // Test múltiples lecturas ADC
            Serial.println("Realizando 10 lecturas ADC...");
            for (int i = 0; i < 10; i++) {
                int adc_l = analogRead(CURRENT_L_PIN);
                int adc_r = analogRead(CURRENT_R_PIN);
                int adc_bat = analogRead(BATTERY_V_PIN);
                
                float v_l = (adc_l / 4095.0) * 3.3;
                float v_r = (adc_r / 4095.0) * 3.3;
                float v_bat = (adc_bat / 4095.0) * 3.3;
                
                Serial.printf("Lectura %d: L=%d(%.3fV), R=%d(%.3fV), Bat=%d(%.3fV)\n", 
                             i+1, adc_l, v_l, adc_r, v_r, adc_bat, v_bat);
                delay(100);
            }
            
            Serial.println("Valores esperados:");
            Serial.println("- Sensores DRV8701 SIN corriente: ~1650 ADC (~1.65V)");
            Serial.println("- Sensores DRV8701 CON corriente: 1650 ± corriente×(20×0.005)");
            Serial.println("- Batería 12V: ~2400 ADC (~1.95V con divisor 39k/12k)");
            Serial.println("================================\n");
        } else if (command == "cal" || command == "calibrate") {
            // Forzar recalibración de sensores de corriente
            Serial.println("\n=== RECALIBRACIÓN DE CORRIENTE ===");
            Serial.println("ASEGÚRATE de que los motores estén PARADOS");
            Serial.println("Recalibrando en 3 segundos...");
            delay(3000);
            
            // TODO: Añadir función de recalibración
            Serial.println("Para recalibrar, reinicia el ESP32 con motores parados");
            Serial.println("=====================================\n");
        } else if (command.startsWith("motor")) {
            // Test de motores: motor L,R,PWM (ej: "motor 50,80,2000" = L=50, R=80, 2seg)
            int comma1 = command.indexOf(',');
            int comma2 = command.indexOf(',', comma1 + 1);
            
            if (comma1 > 0 && comma2 > 0) {
                int left_pwm = command.substring(6, comma1).toInt();
                int right_pwm = command.substring(comma1 + 1, comma2).toInt();
                int duration = command.substring(comma2 + 1).toInt();
                
                left_pwm = constrain(left_pwm, -255, 255);
                right_pwm = constrain(right_pwm, -255, 255);
                duration = constrain(duration, 100, 10000);
                
                Serial.printf("TEST MOTOR: L=%d, R=%d por %dms\n", left_pwm, right_pwm, duration);
                
                // Activar motores
                if (left_pwm >= 0) {
                    setMotorL_direct(left_pwm, true);
                } else {
                    setMotorL_direct(abs(left_pwm), false);
                }
                
                if (right_pwm >= 0) {
                    setMotorR_direct(right_pwm, true);
                } else {
                    setMotorR_direct(abs(right_pwm), false);
                }
                
                delay(duration);
                
                // Parar motores
                setMotorL_direct(0, true);
                setMotorR_direct(0, true);
                disableMotors();
                
                Serial.println("Test completado");
            } else {
                Serial.println("Uso: motor L,R,duration (ej: motor 50,80,2000)");
                Serial.println("L/R: -255 a +255, duration: 100-10000ms");
            }
        } else if (command == "help") {
            Serial.println("Comandos disponibles:");
            Serial.println("- diag: Ejecutar diagnóstico UWB");
            Serial.println("- sensors: Diagnóstico de sensores ADC");
            Serial.println("- cal: Recalibrar sensores de corriente");
            Serial.println("- motor L,R,tiempo: Test motores (ej: motor 50,80,2000)");
            Serial.println("- help: Mostrar esta ayuda");
        }
    }
    
    // Actualizar datos UWB (esto actualiza las variables globales)
    float dummy_x, dummy_y;
    UWBManager_update(dummy_x, dummy_y);
    
    // Solo obtener distancias raw para enviar a la RPi
    float distances[NUM_ANCHORS];
    bool anchor_status[NUM_ANCHORS];
    UWBManager_getDistances(distances);
    UWBManager_getAnchorStatus(anchor_status);
    
    // Mostrar información UWB en consola cada 3 segundos (menos frecuente)
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay > 3000) {
        unsigned long count = UWBManager_getMeasurementCount();
        
        Serial.printf("[UWB] Count: %lu | ", count);
        
        for (int i = 0; i < NUM_ANCHORS; i++) {
            Serial.printf("A%d:", i+1);
            if (anchor_status[i]) {
                Serial.printf("%.1fcm ", distances[i]);
            } else {
                Serial.print("FAIL ");
            }
        }
        Serial.println();
        lastDisplay = millis();
    }
    
    // Diagnóstico automático cada 5 minutos para detectar degradación
    static unsigned long lastAutoDiag = 0;
    if (millis() - lastAutoDiag > 300000) { // 5 minutos
        Serial.println("=== DIAGNÓSTICO AUTOMÁTICO ===");
        UWBManager_runDiagnostics();
        lastAutoDiag = millis();
    }
    
    // Enviar datos raw a la RPi solo cuando hay nuevas mediciones
    static unsigned long lastMeasurementCount = 0;
    unsigned long currentCount = UWBManager_getMeasurementCount();
    
    if (currentCount > lastMeasurementCount) {
        // Hay nuevas mediciones, enviar a RPi (sin cálculo de frecuencia)
        RPiComm_sendRawUWBData(distances, anchor_status, 0.0, currentCount);
        lastMeasurementCount = currentCount;
    }
}
