#pragma once

#include <Arduino.h>
#include "RPiComm.h"

// Funciones públicas
void setupMotorController();
void processSerialCommands();
void sendUWBData(float x, float y, bool valid);
void setMotorSpeed(uint8_t motor_id, int8_t speed);
void emergencyStop();

// Protocolo de comunicación:
// ESP32 <- RPi: JSON con comandos de motor
// ESP32 -> RPi: JSON con datos UWB + sensores
