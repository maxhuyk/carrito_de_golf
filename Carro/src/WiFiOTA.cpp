#include "WiFiOTA.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

// Variables de estado
static WiFiStatus wifiStatus = WIFI_DISCONNECTED;
static unsigned long lastConnectionAttempt = 0;
static bool otaInitialized = false;

// Declaración de función privada
void setupOTA();

void WiFiOTA_setup() {
    // Configurar modo WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    
    // Intentar conexión inicial
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiStatus = WIFI_CONNECTING;
    lastConnectionAttempt = millis();
    
    Serial.printf("[WiFi] Conectando a %s...\n", WIFI_SSID);
}

void WiFiOTA_loop() {
    // Verificar estado de WiFi
    if (WiFi.status() == WL_CONNECTED && wifiStatus != WIFI_CONNECTED) {
        wifiStatus = WIFI_CONNECTED;
        Serial.printf("[WiFi] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
        
        // Inicializar OTA solo cuando WiFi esté conectado
        if (!otaInitialized) {
            setupOTA();
            otaInitialized = true;
        }
    }
    else if (WiFi.status() != WL_CONNECTED && wifiStatus == WIFI_CONNECTED) {
        wifiStatus = WIFI_DISCONNECTED;
        otaInitialized = false;
        Serial.println("[WiFi] Desconectado");
    }
    
    // Reconectar si es necesario
    if (wifiStatus == WIFI_CONNECTING && millis() - lastConnectionAttempt > WIFI_TIMEOUT) {
        if (WiFi.status() != WL_CONNECTED) {
            wifiStatus = WIFI_ERROR;
            Serial.println("[WiFi] Error de conexión, reintentando en 30s");
            // Reintentar en 30 segundos
            static unsigned long lastRetry = 0;
            if (millis() - lastRetry > 30000) {
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
                wifiStatus = WIFI_CONNECTING;
                lastConnectionAttempt = millis();
                lastRetry = millis();
            }
        }
    }
    
    // Manejar OTA si está inicializado
    if (otaInitialized && WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }
}

void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.setPort(OTA_PORT);
    
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("[OTA] Iniciando actualización: " + type);
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Actualización completada");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned long lastPrint = 0;
        // Imprimir progreso cada 10%
        if (millis() - lastPrint > 1000) {
            Serial.printf("[OTA] Progreso: %u%% (%u/%u)\n", (progress / (total / 100)), progress, total);
            lastPrint = millis();
        }
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        switch (error) {
            case OTA_AUTH_ERROR:    Serial.println("Error de autenticación"); break;
            case OTA_BEGIN_ERROR:   Serial.println("Error al iniciar"); break;
            case OTA_CONNECT_ERROR: Serial.println("Error de conexión"); break;
            case OTA_RECEIVE_ERROR: Serial.println("Error de recepción"); break;
            case OTA_END_ERROR:     Serial.println("Error al finalizar"); break;
            default:                Serial.println("Error desconocido"); break;
        }
    });
    
    ArduinoOTA.begin();
    Serial.printf("[OTA] Listo en %s:%d\n", WiFi.localIP().toString().c_str(), OTA_PORT);
}

WiFiStatus getWiFiStatus() {
    return wifiStatus;
}

String getLocalIP() {
    if (wifiStatus == WIFI_CONNECTED) {
        return WiFi.localIP().toString();
    }
    return "0.0.0.0";
}

bool isOTAActive() {
    return otaInitialized && (wifiStatus == WIFI_CONNECTED);
}
