#pragma once

void setupMotorPWMTest();
void setMotorL(int pwm, bool dir);
void setMotorR(int pwm, bool dir);
void motorPWMTestLoop();
void disableMotors();
void enableMotors();
