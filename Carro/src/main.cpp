#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include "DW3000.h"
#include "UWBManager.h"

void setup() {
    Serial.begin(115200);
    delay(2000); // Esperar más tiempo a que se establezca la conexión serie
    
    Serial.println("=== INICIANDO SISTEMA UWB COMPLETO ===");
    Serial.println("1. Serial funcionando OK");
    Serial.flush(); // Asegurar que se envíe
    
    // Test básico de GPIO
    Serial.println("2. Test básico de GPIO...");
    pinMode(2, OUTPUT); // LED interno del ESP32
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    Serial.println("   ✅ GPIO funcionando");
    Serial.flush();
    
    // Inicializar I2C
    Serial.println("3. Iniciando I2C en GPIO5 (SDA) y GPIO4 (SCL)...");
    Serial.flush();
    
    Wire.begin(5, 4); // SDA=GPIO5, SCL=GPIO4
    Wire.setClock(100000); // 100kHz para mayor compatibilidad
    delay(100);
    
    Serial.println("   ✅ I2C iniciado");
    Serial.flush();
    
    // Escanear dispositivos I2C
    Serial.println("4. Escaneando dispositivos I2C...");
    Serial.flush();
    
    int deviceCount = 0;
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte result = Wire.endTransmission();
        if (result == 0) {
            Serial.printf("   Dispositivo encontrado en 0x%02X\n", address);
            deviceCount++;
            Serial.flush();
        }
        delay(10); // Pequeña pausa entre escaneos
    }
    
    if (deviceCount == 0) {
        Serial.println("   ❌ No se encontraron dispositivos I2C");
    } else {
        Serial.printf("   ✅ %d dispositivo(s) I2C encontrado(s)\n", deviceCount);
    }
    Serial.flush();
    
    // Inicializar DW3000 MCP23008 support
    Serial.println("5. Inicializando DW3000 MCP23008 support...");
    Serial.flush();
    
    if (DW3000Class::initMCP23008()) {
        Serial.println("   ✅ DW3000 MCP23008 support inicializado");
    } else {
        Serial.println("   ❌ Error en DW3000 MCP23008 support");
        Serial.flush();
        return;
    }
    
    // Inicializar UWBManager
    Serial.println("6. Inicializando UWBManager con 3 anchors...");
    Serial.flush();
    
    UWBManager_setup();
    
    Serial.println("   ✅ UWBManager inicializado correctamente");
    Serial.flush();
    
    Serial.println("=== SISTEMA UWB LISTO ===");
    Serial.println("Iniciando mediciones de posición...");
    Serial.flush();
}

void loop() {
    float tag_x = 0, tag_y = 0;
    
    if (UWBManager_update(tag_x, tag_y)) {
        Serial.printf("Posición: X=%.3f m, Y=%.3f m\n", tag_x, tag_y);
    } else {
        Serial.println("Esperando mediciones válidas...");
    }
    
    // LED de actividad
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(400);
}
