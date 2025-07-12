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

// Semáforo binario para acceso thread-safe entre cores
static SemaphoreHandle_t uwbSemaphore = NULL;

// Handle de la tarea
static TaskHandle_t uwbTaskHandle = NULL;

void UWBCore_setup() {
    // Crear semáforo binario para sincronización entre cores
    uwbSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(uwbSemaphore); // Inicializar como disponible
    
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
    // Esperar 3 segundos antes de empezar mediciones para estabilización
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("[UWBCore] Iniciando bucle de mediciones...");
    
    // Tarea dedicada para mediciones UWB en Core 0
    while (true) {
        UWBRawData rawData;
        rawData.timestamp = millis();
        
        // Medición Anchor 1 - Con debug detallado
        float distance1 = NAN;
        int t_roundA = 0, t_replyA = 0, clock_offset = 0, ranging_time = 0;
        long long rx = 0, tx = 0;
        
        anchor1.ds_sendFrame(1);
        tx = anchor1.readTXTimestamp();
        delayMicroseconds(10000); // Incrementar a 10ms para mayor robustez
        if (anchor1.receivedFrameSucc() == 1) {
            anchor1.clearSystemStatus();
            rx = anchor1.readRXTimestamp();
            anchor1.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor1.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(10000); // Incrementar a 10ms para mayor robustez
            if (anchor1.receivedFrameSucc() == 1) {
                anchor1.clearSystemStatus();
                clock_offset = anchor1.getRawClockOffset();
                ranging_time = anchor1.ds_processRTInfo(t_roundA, t_replyA, anchor1.read(0x12, 0x04), anchor1.read(0x12, 0x08), clock_offset);
                distance1 = anchor1.convertToCM(ranging_time);
                // Filtrar distancias inválidas
                if (distance1 < 0 || distance1 > 10000) {
                    static unsigned long lastDebugA1_invalid = 0;
                    if (millis() - lastDebugA1_invalid > 3000) {
                        Serial.printf("[UWB] Anchor1: Distancia inválida: %.2f cm (ranging_time=%d)\n", distance1, ranging_time);
                        lastDebugA1_invalid = millis();
                    }
                    distance1 = NAN;
                }
            } else {
                // DEBUG: Segunda recepción falló
                static unsigned long lastDebugA1_2 = 0;
                if (millis() - lastDebugA1_2 > 3000) {
                    Serial.println("[UWB] Anchor1: Segunda recepción FALLÓ");
                    lastDebugA1_2 = millis();
                }
            }
        } else {
            // DEBUG: Primera recepción falló
            static unsigned long lastDebugA1_1 = 0;
            if (millis() - lastDebugA1_1 > 3000) {
                Serial.println("[UWB] Anchor1: Primera recepción FALLÓ");
                lastDebugA1_1 = millis();
            }
        }
        
        // Medición Anchor 2
        float distance2 = NAN;
        anchor2.ds_sendFrame(1);
        tx = anchor2.readTXTimestamp();
        delayMicroseconds(10000); // Incrementar a 10ms para mayor robustez
        if (anchor2.receivedFrameSucc() == 1) {
            anchor2.clearSystemStatus();
            rx = anchor2.readRXTimestamp();
            anchor2.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor2.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(10000); // Incrementar a 10ms para mayor robustez
            if (anchor2.receivedFrameSucc() == 1) {
                anchor2.clearSystemStatus();
                clock_offset = anchor2.getRawClockOffset();
                ranging_time = anchor2.ds_processRTInfo(t_roundA, t_replyA, anchor2.read(0x12, 0x04), anchor2.read(0x12, 0x08), clock_offset);
                distance2 = anchor2.convertToCM(ranging_time);
                // Filtrar distancias inválidas
                if (distance2 < 0 || distance2 > 10000) distance2 = NAN;
            } else {
                // DEBUG: Segunda recepción falló
                static unsigned long lastDebugA2_2 = 0;
                if (millis() - lastDebugA2_2 > 2000) {
                    Serial.println("[UWB] Anchor2: Segunda recepción FALLÓ");
                    lastDebugA2_2 = millis();
                }
            }
        } else {
            // DEBUG: Primera recepción falló
            static unsigned long lastDebugA2_1 = 0;
            if (millis() - lastDebugA2_1 > 2000) {
                Serial.println("[UWB] Anchor2: Primera recepción FALLÓ");
                lastDebugA2_1 = millis();
            }
        }
        
        // Medición Anchor 3 - Con debug detallado
        float distance3 = NAN;
        anchor3.ds_sendFrame(1);
        tx = anchor3.readTXTimestamp();
        delayMicroseconds(10000); // Incrementar a 10ms para mayor robustez
        if (anchor3.receivedFrameSucc() == 1) {
            anchor3.clearSystemStatus();
            rx = anchor3.readRXTimestamp();
            anchor3.ds_sendFrame(3);
            t_roundA = rx - tx;
            tx = anchor3.readTXTimestamp();
            t_replyA = tx - rx;
            delayMicroseconds(10000); // Incrementar a 10ms para mayor robustez
            if (anchor3.receivedFrameSucc() == 1) {
                anchor3.clearSystemStatus();
                clock_offset = anchor3.getRawClockOffset();
                ranging_time = anchor3.ds_processRTInfo(t_roundA, t_replyA, anchor3.read(0x12, 0x04), anchor3.read(0x12, 0x08), clock_offset);
                distance3 = anchor3.convertToCM(ranging_time);
                // Filtrar distancias inválidas
                if (distance3 < 0 || distance3 > 10000) {
                    static unsigned long lastDebugA3_invalid = 0;
                    if (millis() - lastDebugA3_invalid > 3000) {
                        Serial.printf("[UWB] Anchor3: Distancia inválida: %.2f cm (ranging_time=%d)\n", distance3, ranging_time);
                        lastDebugA3_invalid = millis();
                    }
                    distance3 = NAN;
                }
            } else {
                // DEBUG: Segunda recepción falló
                static unsigned long lastDebugA3_2 = 0;
                if (millis() - lastDebugA3_2 > 3000) {
                    Serial.println("[UWB] Anchor3: Segunda recepción FALLÓ");
                    lastDebugA3_2 = millis();
                }
            }
        } else {
            // DEBUG: Primera recepción falló
            static unsigned long lastDebugA3_1 = 0;
            if (millis() - lastDebugA3_1 > 3000) {
                Serial.println("[UWB] Anchor3: Primera recepción FALLÓ");
                lastDebugA3_1 = millis();
            }
        }
        
        // Actualizar datos compartidos de forma atómica
        rawData.distance1 = distance1;
        rawData.distance2 = distance2;
        rawData.distance3 = distance3;
        rawData.valid1 = !isnan(distance1);
        rawData.valid2 = !isnan(distance2);
        rawData.valid3 = !isnan(distance3);
        
        // Escribir datos de forma thread-safe usando semáforo
        if (xSemaphoreTake(uwbSemaphore, portMAX_DELAY) == pdTRUE) {
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
            xSemaphoreGive(uwbSemaphore);
        }
        
        // DEBUG: Detectar cambios súbitos a NAN y estadísticas
        static bool prev_valid1 = false, prev_valid2 = false, prev_valid3 = false;
        static unsigned long lastStateChange = 0;
        static unsigned long total_measurements = 0;
        static unsigned long failed_measurements[3] = {0, 0, 0};
        
        total_measurements++;
        if (!rawData.valid1) failed_measurements[0]++;
        if (!rawData.valid2) failed_measurements[1]++;
        if (!rawData.valid3) failed_measurements[2]++;
        
        if ((prev_valid1 && !rawData.valid1) || (prev_valid2 && !rawData.valid2) || (prev_valid3 && !rawData.valid3)) {
            if (millis() - lastStateChange > 1000) { // Solo reportar cambios cada segundo
                Serial.printf("[UWB_CORE] Estado cambió: A1:%d->%d, A2:%d->%d, A3:%d->%d\n", 
                             prev_valid1, rawData.valid1, prev_valid2, rawData.valid2, prev_valid3, rawData.valid3);
                lastStateChange = millis();
            }
        }
        
        // Estadísticas cada 30 segundos
        static unsigned long lastStats = 0;
        if (millis() - lastStats > 30000) {
            Serial.printf("[UWB_STATS] Total:%lu, Fallos A1:%lu(%.1f%%), A2:%lu(%.1f%%), A3:%lu(%.1f%%)\n",
                         total_measurements,
                         failed_measurements[0], (failed_measurements[0] * 100.0) / total_measurements,
                         failed_measurements[1], (failed_measurements[1] * 100.0) / total_measurements,
                         failed_measurements[2], (failed_measurements[2] * 100.0) / total_measurements);
            lastStats = millis();
        }
        
        prev_valid1 = rawData.valid1;
        prev_valid2 = rawData.valid2;
        prev_valid3 = rawData.valid3;
        
        // Delay más largo entre ciclos completos para estabilidad
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms = 20 Hz máximo
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
    // Leer datos de forma thread-safe usando semáforo
    if (xSemaphoreTake(uwbSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
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
        xSemaphoreGive(uwbSemaphore);
        return true;
    }
    
    // Si no se puede obtener el semáforo, devolver datos por defecto
    data.distance1 = NAN;
    data.distance2 = NAN;
    data.distance3 = NAN;
    data.valid1 = false;
    data.valid2 = false;
    data.valid3 = false;
    data.timestamp = millis();
    
    return false;
}

void UWBCore_diagnostics() {
    Serial.println("\n=== DIAGNÓSTICO UWB ===");
    
    // Verificar conectividad SPI con cada módulo
    Serial.println("Verificando conectividad SPI...");
    
    // Anchor 1 - Leer registro de identificación
    uint32_t dev_id1 = anchor1.read(0x00, 0x04); // Registro DEV_ID
    Serial.printf("Anchor 1 - Device ID: 0x%08X %s\n", dev_id1, 
                 (dev_id1 == 0xDECA0302 || dev_id1 != 0x00000000) ? "(OK)" : "(ERROR - Sin respuesta SPI)");
    
    // Anchor 2  
    uint32_t dev_id2 = anchor2.read(0x00, 0x04);
    Serial.printf("Anchor 2 - Device ID: 0x%08X %s\n", dev_id2,
                 (dev_id2 == 0xDECA0302 || dev_id2 != 0x00000000) ? "(OK)" : "(ERROR - Sin respuesta SPI)");
    
    // Anchor 3
    uint32_t dev_id3 = anchor3.read(0x00, 0x04);
    Serial.printf("Anchor 3 - Device ID: 0x%08X %s\n", dev_id3,
                 (dev_id3 == 0xDECA0302 || dev_id3 != 0x00000000) ? "(OK)" : "(ERROR - Sin respuesta SPI)");
    
    // Estado actual de las mediciones - thread-safe
    UWBRawData current_data;
    bool data_available = UWBCore_getRawData(current_data);
    
    Serial.printf("\nEstado actual:\n");
    Serial.printf("- Total mediciones: %lu\n", g_uwb_measurement_count);
    Serial.printf("- Última medición: %lu ms ago\n", millis() - g_uwb_last_measurement_time);
    Serial.printf("- Datos disponibles: %s\n", data_available ? "SÍ" : "NO");
    Serial.printf("- Datos válidos: A1=%d, A2=%d, A3=%d\n", 
                 current_data.valid1, current_data.valid2, current_data.valid3);
    Serial.printf("- Distancias: A1=%.1f, A2=%.1f, A3=%.1f cm\n",
                 current_data.distance1, current_data.distance2, current_data.distance3);
    
    Serial.println("======================\n");
}

unsigned long UWBCore_getMeasurementCount() {
    // Thread-safe access al contador
    unsigned long count = 0;
    if (xSemaphoreTake(uwbSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = g_uwb_measurement_count;
        xSemaphoreGive(uwbSemaphore);
    }
    return count;
}
