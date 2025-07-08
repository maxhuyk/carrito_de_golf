#include <Arduino.h>
#include "DW3000.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

// Configuración del divisor de voltaje de la batería LiPo
#define BATTERY_PIN 36        // GPIO36 (SENSOR_VP) para lectura de voltaje
#define R1 10000.0           // Resistencia superior (10k ohms)
#define R2 33000.0           // Resistencia inferior (33k ohms)
#define VOLTAGE_DIVIDER_FACTOR ((R1 + R2) / R2)  // Factor de conversión: 1.303
#define ADC_RESOLUTION 4095.0 // Resolución del ADC (12 bits)
#define ADC_REFERENCE_VOLTAGE 3.3 // Voltaje de referencia del ADC
#define BATTERY_SAMPLES 10    // Número de muestras para promediar

// Example: create two DW3000 instances with different pins
// DW3000Class dw1(csPin1, rstPin1, irqPin1);
// DW3000Class dw2(csPin2, rstPin2, irqPin2);
// Use dw1.begin(), dw2.begin(), etc.

// For demonstration, use one instance:
DW3000Class dw(4, 32, 34); // Example pins: CS=4, RST=32, IRQ=34

/*
   BE AWARE: Baud Rate got changed to 2.000.000!

   Approach based on the application note APS011 ("SOURCES OF ERROR IN DW1000 BASED
   TWO-WAY RANGING (TWR) SCHEMES")

   see chapter 2.4 figure 6 and the corresponding description for more information

   This approach tackles the problem of a big clock offset between the ping and pong side
   by reducing the clock offset to a minimum.

   This approach is a more advanced version of the classical ping and pong with timestamp examples.
*/

static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

/*
   valid stages:
   0 - default stage; await ranging
   1 - ranging received; sending response
   2 - response sent; await second response
   3 - second response received; sending information frame
   4 - information frame sent
*/
static int curr_stage = 0;

static int t_roundB = 0;
static int t_replyB = 0;

static long long rx = 0;
static long long tx = 0;

static unsigned long last_dw3000_rx = 40000;
static const unsigned long WIFI_RECONNECT_TIMEOUT = 30000; // 30 segundos

// Función para leer el voltaje de la batería LiPo
float readBatteryVoltage() {
    long sum = 0;
    
    // Tomar múltiples muestras para reducir ruido
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
        sum += analogRead(BATTERY_PIN);
        delayMicroseconds(100); // Pequeño delay entre muestras
    }
    
    // Calcular promedio
    float average = (float)sum / BATTERY_SAMPLES;
    
    // Convertir a voltaje
    float voltage_at_pin = (average / ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
    
    // Aplicar factor de divisor de voltaje
    float battery_voltage = voltage_at_pin * VOLTAGE_DIVIDER_FACTOR;
    
    return battery_voltage;
}

// Función para obtener estado de la batería LiPo
String getBatteryStatus(float voltage) {
    if (voltage > 4.1) return "FULL";      // >4.1V por celda
    else if (voltage > 3.8) return "GOOD"; // 3.8-4.1V por celda
    else if (voltage > 3.6) return "MID";  // 3.6-3.8V por celda
    else if (voltage > 3.3) return "LOW";  // 3.3-3.6V por celda
    else return "CRITICAL";                // <3.3V por celda
}

// Función para obtener porcentaje aproximado de batería LiPo
int getBatteryPercentage(float voltage) {
    if (voltage >= 4.1) return 100;
    else if (voltage >= 3.9) return 80;
    else if (voltage >= 3.8) return 60;
    else if (voltage >= 3.7) return 40;
    else if (voltage >= 3.6) return 20;
    else if (voltage >= 3.3) return 10;
    else return 0;
}

void setup()
{
  pinMode(5, OUTPUT); // LED en GPIO 5
  digitalWrite(5, LOW); // Apagado por defecto
  Serial.begin(2000000); // Init Serial
  
  // Configurar el pin de lectura de batería
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12); // Configurar resolución del ADC a 12 bits
  analogSetAttenuation(ADC_11db); // Configurar atenuación para rango completo
  
  // Leer voltaje inicial de la batería
  float initial_voltage = readBatteryVoltage();
  int battery_percentage = getBatteryPercentage(initial_voltage);
  Serial.printf("[BATTERY] Voltaje inicial: %.2fV (%s - %d%%)\n", 
                initial_voltage, getBatteryStatus(initial_voltage).c_str(), battery_percentage);
  
  // --- OTA: Conexión WiFi y setup OTA (no bloqueante) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin("CLAROWIFI", "11557788");
  unsigned long wifi_start = millis();
  Serial.print("[OTA] Conectando a WiFi");
  bool led_state = false;
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 20000) {
    delay(100);
    Serial.print(".");
    led_state = !led_state;
    digitalWrite(5, led_state ? HIGH : LOW); // Titila LED
  }
  digitalWrite(5, LOW); // Apaga LED al terminar intento
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[OTA] WiFi conectado, IP: ");
    Serial.println(WiFi.localIP());
    ArduinoOTA.setHostname("tag-ota");
    ArduinoOTA.onStart([]() { Serial.println("[OTA] Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("[OTA] End"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progreso: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("[OTA] Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
  } else {
    Serial.println();
    Serial.println("[OTA] WiFi NO DISPONIBLE, el sistema funcionará sin OTA");
  }
  last_dw3000_rx = millis(); // <-- Inicializa con millis() para permitir reintentos aunque nunca haya frame

  dw.begin(); // Init SPI
  dw.hardReset(); // hard reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  // --- ASIGNAR ID UNICO AL TAG Y ACEPTAR SOLO DE ANCHORS ---
  // const uint16_t TAG_ID = 10;
  // const uint16_t ANCHOR1_ID = 1;
  // const uint16_t ANCHOR2_ID = 2;
  // dw.setSenderID(TAG_ID); // El TAG tiene ID 10
  // Si quieres que el tag solo responda a un anchor específico, usa:
  // dw.setDestinationID(ANCHOR1_ID); // o ANCHOR2_ID
  // Si quieres aceptar de ambos, puedes cambiar dinámicamente en el loop o aceptar ambos en la lógica

  if(!dw.checkSPI())
  {
    Serial.println("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly.");
    while(100);
  }
  while (!dw.checkForIDLE()) // Make sure that chip is in IDLE before continuing
  {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    delay(1000);
  }
  dw.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if (!dw.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }

  dw.init(); // Initialize chip (write default values, calibration, etc.)
  dw.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs

  Serial.println("> double-sided PONG with timestamp example <\n");

  Serial.println("[INFO] Setup finished.");

  dw.configureAsTX(); // Configure basic settings for frame transmitting

  dw.clearSystemStatus();

  dw.standardRX();
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }
  
  // Leer voltaje de la batería cada 15 segundos
  static unsigned long lastBatteryRead = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastBatteryRead >= 15000) { // 15 segundos
    float battery_voltage = readBatteryVoltage();
    String battery_status = getBatteryStatus(battery_voltage);
    int battery_percentage = getBatteryPercentage(battery_voltage);
    Serial.printf("[BATTERY] %.2fV (%s - %d%%)\n", battery_voltage, battery_status.c_str(), battery_percentage);
    
    // Advertencia si la batería está baja
    if (battery_voltage < 3.6) {
      Serial.println("[WARNING] Batería baja! Considere recargar.");
      digitalWrite(5, HIGH); // Enciende LED como advertencia
      delay(100);
      digitalWrite(5, LOW);
    }
    
    lastBatteryRead = currentTime;
  }
  
  // --- Reintento de WiFi si no hay frames DW3000 por más de 30s ---
  if (millis() - last_dw3000_rx > WIFI_RECONNECT_TIMEOUT && WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] Intentando reconectar WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    unsigned long wifi_reconnect_start = millis();
    bool led_state = false;
    while (WiFi.status() != WL_CONNECTED && millis() - wifi_reconnect_start < 5000) {
      delay(100);
      Serial.print(".");
      led_state = !led_state;
      digitalWrite(5, led_state ? HIGH : LOW); // Titila LED
    }
    digitalWrite(5, LOW); // Apaga LED al terminar intento
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("[OTA] WiFi reconectado, IP: ");
      Serial.println(WiFi.localIP());
      ArduinoOTA.begin();
    } else {
      Serial.println();
      Serial.println("[OTA] No se pudo reconectar WiFi");
    }
    last_dw3000_rx = millis(); // Evita reintentos continuos si sigue sin frames
  }
  switch (curr_stage) {
    case 0:  // Await ranging.
      t_roundB = 0;
      t_replyB = 0;

      if (rx_status = dw.receivedFrameSucc()) {
        last_dw3000_rx = millis(); // <-- Marca el último frame recibido
        dw.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (dw.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
            curr_stage = 0;
            dw.standardRX();
          } else if (dw.ds_getStage() != 1) {
            dw.ds_sendErrorFrame();
            dw.standardRX();
            curr_stage = 0;
          } else {
            curr_stage = 1;
          }
        } else // if rx_status returns error (2)
        {
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          dw.clearSystemStatus();
        }
      }
      break;
    case 1:  // Ranging received. Sending response.
      dw.ds_sendFrame(2);

      rx = dw.readRXTimestamp();
      tx = dw.readTXTimestamp();

      t_replyB = tx - rx;
      curr_stage = 2;
      break;
    case 2:  // Awaiting response.
      if (rx_status = dw.receivedFrameSucc()) {
        dw.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (dw.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
            curr_stage = 0;
            dw.standardRX();
          } else if (dw.ds_getStage() != 3) {
            dw.ds_sendErrorFrame();
            dw.standardRX();
            curr_stage = 0;
          } else {
            curr_stage = 3;
          }
        } else // if rx_status returns error (2)
        {
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          dw.clearSystemStatus();
        }
      }
      break;
    case 3:  // Second response received. Sending information frame.
      rx = dw.readRXTimestamp();
      t_roundB = rx - tx;
      dw.ds_sendRTInfo(t_roundB, t_replyB);

      curr_stage = 0;
      break;
    default:
      Serial.print("[ERROR] Entered unknown stage (");
      Serial.print(curr_stage);
      Serial.println("). Reverting back to stage 0");

      curr_stage = 0;
      dw.standardRX();
      break;
  }
  // Validar que el frame recibido venga de un anchor válido
  // (opcional, si quieres aceptar solo de ANCHOR1_ID o ANCHOR2_ID)
  // if (dw.getSenderID() != ANCHOR1_ID && dw.getSenderID() != ANCHOR2_ID) {
  //   // Ignorar frame
  //   dw.standardRX();
  //   curr_stage = 0;
  //   return;
  // }
}