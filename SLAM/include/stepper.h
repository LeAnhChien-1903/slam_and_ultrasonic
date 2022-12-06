#ifndef __STEPPER_NEW_H_
#define __STEPPER_NEW_H_

#include "Arduino.h"
#define PI 3.1415926535897932384626433832795
// Using a 200-step motor
#define MOTOR_STEPS 200
// Defining the Motor Pins for Left Motor
#define EnPinLeft 52
#define StepPinLeft 44
#define DirPinLeft 42
// Defining the Motor Pins for Right Motor
#define EnPinRight 32
#define StepPinRight 24
#define DirPinRight 22
// uint64_t prevTimeLeft = 0;
// uint64_t prevTimeRight = 0;
// uint64_t prevTimeTurnLeft = 0;
// uint64_t prevTimeTurnRight = 0;
// uint64_t prevTimeForward = 0;
// bool togglePulseLeft = HIGH;
// bool togglePulseRight = HIGH;

uint16_t stepsPerRevolution = 200;
// Prototype 
void motorSetup();
uint16_t convertRadToTimes(float omega);
void turnLeft(float omega, uint16_t steps);
void turnRight(float omega, uint16_t steps);
void goStraight(float omega, uint16_t steps);
void motorSetup()
{
    pinMode(EnPinLeft, OUTPUT);
    pinMode(StepPinLeft, OUTPUT);
    pinMode(DirPinLeft, OUTPUT);
    pinMode(EnPinRight, OUTPUT);
    pinMode(StepPinRight, OUTPUT);
    pinMode(DirPinRight, OUTPUT);
    digitalWrite(EnPinLeft, LOW);
    digitalWrite(EnPinRight, LOW);
    // prevTimeLeft = micros();
    // prevTimeRight = micros();
    // prevTimeForward = millis();
    // prevTimeTurnLeft = millis();
    // prevTimeTurnRight = millis();
}
void controlMotorLeft(uint16_t times, bool direction)
{
    digitalWrite(DirPinLeft, direction);
    digitalWrite(StepPinLeft, HIGH);
    delayMicroseconds(times);
    digitalWrite(StepPinLeft, LOW);
    delayMicroseconds(times);
}
void controlMotorRight(uint16_t times, bool direction)
{
    digitalWrite(DirPinRight, direction);
    digitalWrite(StepPinRight, HIGH);
    delayMicroseconds(times);
    digitalWrite(StepPinRight, LOW);
    delayMicroseconds(times);
}
void controlTwoMotor(uint16_t times, bool direction)
{
    digitalWrite(DirPinRight, direction);
    digitalWrite(DirPinLeft, direction);
    digitalWrite(StepPinRight, HIGH);
    digitalWrite(StepPinLeft, HIGH);
    delayMicroseconds(times);
    digitalWrite(StepPinRight, LOW);
    digitalWrite(StepPinLeft, LOW);
    delayMicroseconds(times);
}
void turnLeft(float omega, uint16_t steps)
{
    uint16_t times = convertRadToTimes(omega);
    for (uint16_t i = 0; i < steps; i++)
    {
        controlMotorRight(times, HIGH);
    }
}
void turnRight(float omega, uint16_t steps)
{
    uint16_t times = convertRadToTimes(omega);
    for (uint16_t i = 0; i < steps; i++)
    {
        controlMotorLeft(times, HIGH);
    }
}
void goStraight(float omega, uint16_t steps)
{
    uint16_t times = convertRadToTimes(omega);
    if (omega > 0)
    {
        for (uint16_t i = 0; i < steps; i++)
        {
            controlTwoMotor(times, HIGH);
        }
    }
    else
    {
        for (uint16_t i = 0; i < steps; i++)
        {
            controlTwoMotor(times, LOW);
        }
    }
}
uint16_t convertRadToTimes(float omega)
{
    /*
        omega: angular velocity of wheel (rad/s)
    */
    uint16_t times = uint16_t(1000000*PI/(abs(omega)*stepsPerRevolution));
    return times;
}
#endif