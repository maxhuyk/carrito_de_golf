#ifndef RPICOMM_H
#define RPICOMM_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Configuración de pines para UART2 (comunicación con Raspberry Pi)
#define RPI_UART_RX_PIN 22
#define RPI_UART_TX_PIN 21
#define RPI_UART_SPEED 115200

// Pines de sensores
#define CURRENT_L_PIN 36  // GPIO36 (VP)  - Corriente motor izquierdo
#define CURRENT_R_PIN 39  // GPIO39 (VN)  - Corriente motor derecho
#define BATTERY_V_PIN 34  // GPIO34       - Voltaje de batería

// Configuración del divisor de voltaje para batería 12V
// VIN---39k--GPIO34---12k---GND
#define BATTERY_R1 39000.0  // 39kΩ
#define BATTERY_R2 12000.0  // 12kΩ
#define BATTERY_MAX_VOLTAGE 12.0  // Voltaje máximo de batería de gel
#define ADC_RESOLUTION 4095.0  // ESP32 ADC 12-bit
#define ADC_VREF 3.3  // Voltaje de referencia del ADC

// Estructura para datos del sistema
struct SystemData {
    float tag_x;
    float tag_y;
    bool uwb_valid;
    float current_left;
    float current_right;
    float battery_voltage;
    unsigned long timestamp;
};

// Estructura para comandos de motor
struct MotorCommand {
    int motor_left_speed;   // -255 a +255
    int motor_right_speed;  // -255 a +255
    bool emergency_stop;
    char command_type;      // 'M' = motor, 'S' = stop, 'P' = ping
};

// Funciones públicas
void RPiComm_setup();
void RPiComm_sendSystemData(float tag_x, float tag_y, bool uwb_valid);
bool RPiComm_receiveCommand(MotorCommand &cmd);
float RPiComm_readBatteryVoltage();
float RPiComm_readCurrentSensor(int pin);
void RPiComm_sendHeartbeat();

// Funciones de utilidad
void RPiComm_sendJSON(const char* json);
bool RPiComm_available();

#endif // RPICOMM_H
