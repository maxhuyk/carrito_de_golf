#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include "DW3000.h"
#include "UWBManager.h"
#include "UWBCore.h"
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
    
    // Obtener y enviar datos UWB
    float tag_x = 0, tag_y = 0;
    bool uwb_valid = UWBManager_update(tag_x, tag_y);
    
    // Obtener distancias individuales
    float distances[NUM_ANCHORS];
    bool anchor_status[NUM_ANCHORS];
    UWBManager_getDistances(distances);
    UWBManager_getAnchorStatus(anchor_status);
    
    // Mostrar información UWB en consola cada 3 segundos (menos frecuente)
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay > 3000) {
        float frequency = UWBManager_getMeasurementFrequency();
        unsigned long count = UWBManager_getMeasurementCount();
        
        Serial.printf("[UWB] Freq: %.1fHz (%lu med) | ", frequency, count);
        
        for (int i = 0; i < NUM_ANCHORS; i++) {
            Serial.printf("A%d:", i+1);
            if (anchor_status[i]) {
                Serial.printf("%.1fcm ", distances[i]);
            } else {
                Serial.print("FAIL ");
            }
        }
        
        if (uwb_valid) {
            Serial.printf("| Pos: X=%.2fm Y=%.2fm", tag_x, tag_y);
        } else {
            Serial.print("| Pos: INVALID");
        }
        Serial.println();
        lastDisplay = millis();
    }
    
    static unsigned long lastUWBSend = 0;
    if (millis() - lastUWBSend > 1000) {
        sendUWBData(tag_x, tag_y, uwb_valid);
        lastUWBSend = millis();
    }
}
