#include "UWBCore.h"
#include "DW3000.h"
#include <Arduino.h>

// Configuración de pines del multiplexor MCP23008
#define ANCHOR1_CS 0     // GP0 del MCP23008
#define ANCHOR1_RST 4    // GP4 del MCP23008
#define ANCHOR1_IRQ 35   // GPIO35 del ESP32 (directo)

#define ANCHOR2_CS 1     // GP1 del MCP23008
#define ANCHOR2_RST 5    // GP5 del MCP23008
#define ANCHOR2_IRQ 27   // GPIO27 del ESP32 (directo)

#define ANCHOR3_CS 2     // GP2 del MCP23008
#define ANCHOR3_RST 6    // GP6 del MCP23008
#define ANCHOR3_IRQ 13   // GPIO13 del ESP32 (directo)

// Instancias DW3000
static DW3000Class anchor1(ANCHOR1_CS, ANCHOR1_RST, ANCHOR1_IRQ);
static DW3000Class anchor2(ANCHOR2_CS, ANCHOR2_RST, ANCHOR2_IRQ);
static DW3000Class anchor3(ANCHOR3_CS, ANCHOR3_RST, ANCHOR3_IRQ);

// Variables compartidas entre cores
volatile UWBRawData g_uwb_raw_data = {NAN, NAN, NAN, false, false, false, 0};
volatile bool g_uwb_data_ready = false;
volatile unsigned long g_uwb_measurement_count = 0;
volatile unsigned long g_uwb_last_measurement_time = 0;

// Mutex para acceso thread-safe
static portMUX_TYPE uwbMutex = portMUX_INITIALIZER_UNLOCKED;

// Handle de la tarea
static TaskHandle_t uwbTaskHandle = NULL;

void UWBCore_setup() {
    // Inicializar Anchor 1
    anchor1.begin();
    anchor1.hardReset();
    delay(200);
    while (!anchor1.checkForIDLE()) delay(100);
    anchor1.softReset();
    delay(200);
    anchor1.init();
    anchor1.setupGPIO();
    anchor1.configureAsTX();
    anchor1.clearSystemStatus();

    // Inicializar Anchor 2
    anchor2.begin();
    anchor2.hardReset();
    delay(200);
    while (!anchor2.checkForIDLE()) delay(100);
    anchor2.softReset();
    delay(200);
    anchor2.init();
    anchor2.setupGPIO();
    anchor2.configureAsTX();
    anchor2.clearSystemStatus();

    // Inicializar Anchor 3
    anchor3.begin();
    anchor3.hardReset();
    delay(200);
    while (!anchor3.checkForIDLE()) delay(100);
    anchor3.softReset();
    delay(200);
    anchor3.init();
    anchor3.setupGPIO();
    anchor3.configureAsTX();
    anchor3.clearSystemStatus();
}

void UWBCore_task(void* parameter) {
    // Tarea dedicada para mediciones UWB en Core 0
    while (true) {
        UWBRawData rawData;
        rawData.timestamp = millis();
        
        // Medición Anchor 1 - Sin delays
        float distance1 = NAN;
        int t_roundA = 0, t_replyA = 0, clock_offset = 0, ranging_time = 0;
        long long rx = 0, tx = 0;
        
        anchor1.ds_sendFrame(1);
        tx = anchor1.readTXTimestamp();
        delayMicroseconds(5000); // 5ms en lugar de 10ms
        if (anchor1.receivedFrameSucc() == 1) {
            anchor1.clearSystemStatus();
            rx = anchor1.readRXTimestamp();
            anchor1.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor1.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(5000); // 5ms en lugar de 10ms
            if (anchor1.receivedFrameSucc() == 1) {
                anchor1.clearSystemStatus();
                clock_offset = anchor1.getRawClockOffset();
                ranging_time = anchor1.ds_processRTInfo(t_roundA, t_replyA, anchor1.read(0x12, 0x04), anchor1.read(0x12, 0x08), clock_offset);
                distance1 = anchor1.convertToCM(ranging_time);
                // Filtrar distancias inválidas
                if (distance1 < 0 || distance1 > 10000) distance1 = NAN;
            }
        }
        
        // Medición Anchor 2
        float distance2 = NAN;
        anchor2.ds_sendFrame(1);
        tx = anchor2.readTXTimestamp();
        delayMicroseconds(5000);
        if (anchor2.receivedFrameSucc() == 1) {
            anchor2.clearSystemStatus();
            rx = anchor2.readRXTimestamp();
            anchor2.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor2.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(5000);
            if (anchor2.receivedFrameSucc() == 1) {
                anchor2.clearSystemStatus();
                clock_offset = anchor2.getRawClockOffset();
                ranging_time = anchor2.ds_processRTInfo(t_roundA, t_replyA, anchor2.read(0x12, 0x04), anchor2.read(0x12, 0x08), clock_offset);
                distance2 = anchor2.convertToCM(ranging_time);
                // Filtrar distancias inválidas
                if (distance2 < 0 || distance2 > 10000) distance2 = NAN;
            }
        }
        
        // Medición Anchor 3
        float distance3 = NAN;
        anchor3.ds_sendFrame(1);
        tx = anchor3.readTXTimestamp();
        delayMicroseconds(5000);
        if (anchor3.receivedFrameSucc() == 1) {
            anchor3.clearSystemStatus();
            rx = anchor3.readRXTimestamp();
            anchor3.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor3.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(5000);
            if (anchor3.receivedFrameSucc() == 1) {
                anchor3.clearSystemStatus();
                clock_offset = anchor3.getRawClockOffset();
                ranging_time = anchor3.ds_processRTInfo(t_roundA, t_replyA, anchor3.read(0x12, 0x04), anchor3.read(0x12, 0x08), clock_offset);
                distance3 = anchor3.convertToCM(ranging_time);
                // Filtrar distancias inválidas
                if (distance3 < 0 || distance3 > 10000) distance3 = NAN;
            }
        }
        
        // Actualizar datos compartidos de forma atómica
        rawData.distance1 = distance1;
        rawData.distance2 = distance2;
        rawData.distance3 = distance3;
        rawData.valid1 = !isnan(distance1);
        rawData.valid2 = !isnan(distance2);
        rawData.valid3 = !isnan(distance3);
        
        // Escribir datos de forma thread-safe
        portENTER_CRITICAL(&uwbMutex);
        g_uwb_raw_data.distance1 = rawData.distance1;
        g_uwb_raw_data.distance2 = rawData.distance2;
        g_uwb_raw_data.distance3 = rawData.distance3;
        g_uwb_raw_data.valid1 = rawData.valid1;
        g_uwb_raw_data.valid2 = rawData.valid2;
        g_uwb_raw_data.valid3 = rawData.valid3;
        g_uwb_raw_data.timestamp = rawData.timestamp;
        g_uwb_data_ready = true;
        g_uwb_measurement_count++;
        g_uwb_last_measurement_time = millis();
        portEXIT_CRITICAL(&uwbMutex);
        
        // Sin delay - máxima velocidad
        taskYIELD(); // Permitir que otras tareas del mismo core se ejecuten
    }
}

void UWBCore_startTask() {
    Serial.println("[UWBCore] Iniciando tarea UWB en Core 0...");
    
    // Crear tarea en Core 0 con alta prioridad
    BaseType_t result = xTaskCreatePinnedToCore(
        UWBCore_task,        // Función de la tarea
        "UWB_Core_Task",     // Nombre de la tarea
        4096,                // Tamaño del stack
        NULL,                // Parámetros
        2,                   // Prioridad (alta)
        &uwbTaskHandle,      // Handle de la tarea
        0                    // Core 0
    );
    
    if (result == pdPASS) {
        Serial.println("[UWBCore] Tarea UWB creada exitosamente en Core 0");
    } else {
        Serial.printf("[UWBCore] ERROR: No se pudo crear la tarea UWB (error %d)\n", result);
    }
}

bool UWBCore_getRawData(UWBRawData& data) {
    // Leer datos de forma thread-safe (siempre devolver los más recientes)
    portENTER_CRITICAL(&uwbMutex);
    data.distance1 = g_uwb_raw_data.distance1;
    data.distance2 = g_uwb_raw_data.distance2;
    data.distance3 = g_uwb_raw_data.distance3;
    data.valid1 = g_uwb_raw_data.valid1;
    data.valid2 = g_uwb_raw_data.valid2;
    data.valid3 = g_uwb_raw_data.valid3;
    data.timestamp = g_uwb_raw_data.timestamp;
    bool dataReady = g_uwb_data_ready;
    if (g_uwb_data_ready) {
        g_uwb_data_ready = false; // Marcar como leído solo si había datos nuevos
    }
    portEXIT_CRITICAL(&uwbMutex);
    
    return true; // Siempre devolver datos (aunque sean viejos)
}

float UWBCore_getMeasurementFrequency() {
    static unsigned long lastMeasurementCount = 0;
    static unsigned long lastCheckTime = 0;
    static float frequency = 0.0;
    
    unsigned long currentTime = millis();
    unsigned long currentCount = g_uwb_measurement_count;
    
    if (lastCheckTime > 0 && currentTime > lastCheckTime) {
        unsigned long timeInterval = currentTime - lastCheckTime;
        unsigned long countInterval = currentCount - lastMeasurementCount;
        
        if (timeInterval > 0) {
            frequency = (countInterval * 1000.0) / timeInterval;
        }
    }
    
    lastCheckTime = currentTime;
    lastMeasurementCount = currentCount;
    
    return frequency;
}
