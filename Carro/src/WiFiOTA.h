#ifndef WIFIOTA_H
#define WIFIOTA_H

#include <Arduino.h>

// Configuración WiFi
#define WIFI_SSID "CLAROWIFI"
#define WIFI_PASSWORD "11557788"
#define WIFI_TIMEOUT 10000  // 10 segundos

// Configuración OTA
#define OTA_HOSTNAME "GolfCart-ESP32"
#define OTA_PASSWORD "golf2024"  // Contraseña para OTA
#define OTA_PORT 3232

// Estados de conexión
enum WiFiStatus {
    WIFI_DISCONNECTED,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_ERROR
};

// Funciones públicas
void WiFiOTA_setup();
void WiFiOTA_loop();
WiFiStatus getWiFiStatus();
String getLocalIP();
bool isOTAActive();

#endif // WIFIOTA_H
