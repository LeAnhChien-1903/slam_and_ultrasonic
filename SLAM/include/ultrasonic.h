#ifndef __ULTRASONIC_H_
#define __ULTRASONIC_H_

#include "Arduino.h"
#include <SimpleKalmanFilter.h>
#define MAX_RANGE 2.0
#define MIN_RANGE 0.02
#define MAX_DISTANCE 200 // Max distance to detect obstacles
#define SONAR_NUM 6
#define trigPin0 4           
#define echoPin0 5
#define trigPin45 6
#define echoPin45 7
#define trigPin90 12
#define echoPin90 13
#define trigPin135 10
#define echoPin135 11
#define trigPin180 8
#define echoPin180 9
#define trigPin270 2
#define echoPin270 3

// Prototype 
void ultrasonicInit();
void ultrasonicCycle();
void applyKF();
uint8_t PingSonar(int trigPin, int echoPin);

uint8_t sensor0;  //Store raw sensor's value.
uint8_t sensor45;
uint8_t sensor90;
uint8_t sensor135;
uint8_t sensor180;
uint8_t sensor270;

uint8_t sensor0Kalman; //Store kalman sensor's value.
uint8_t sensor45Kalman;
uint8_t sensor90Kalman;
uint8_t sensor135Kalman;
uint8_t sensor180Kalman;
uint8_t sensor270Kalman;

SimpleKalmanFilter KF_0(2, 2, 0.01);
SimpleKalmanFilter KF_45(2, 2, 0.01);
SimpleKalmanFilter KF_90(2, 2, 0.01);
SimpleKalmanFilter KF_135(2, 2, 0.01);
SimpleKalmanFilter KF_180(2, 2, 0.01);
SimpleKalmanFilter KF_270(2, 2, 0.01);

void ultrasonicInit()
{
    pinMode(trigPin0, OUTPUT);
    pinMode(echoPin0, INPUT);
    pinMode(trigPin45, OUTPUT);
    pinMode(echoPin45, INPUT);
    pinMode(trigPin90, OUTPUT);
    pinMode(echoPin90, INPUT);
    pinMode(trigPin135, OUTPUT);
    pinMode(echoPin135, INPUT);
    pinMode(trigPin180, OUTPUT);
    pinMode(echoPin180, INPUT);
    pinMode(trigPin270, OUTPUT);
    pinMode(echoPin270, INPUT);
}
void ultrasonicCycle()
{
    sensor0 = PingSonar(trigPin0, echoPin0);
    sensor45 = PingSonar(trigPin45, echoPin45);
    sensor90 = PingSonar(trigPin90, echoPin90);
    sensor135 = PingSonar(trigPin135, echoPin135);
    sensor180 = PingSonar(trigPin180, echoPin180);
    sensor270 = PingSonar(trigPin270, echoPin270);
}
void applyKF()
{
    sensor0Kalman = KF_0.updateEstimate(sensor0);
    sensor45Kalman = KF_45.updateEstimate(sensor45);
    sensor90Kalman = KF_90.updateEstimate(sensor90);
    sensor135Kalman = KF_135.updateEstimate(sensor135);
    sensor180Kalman = KF_180.updateEstimate(sensor180);
    sensor270Kalman = KF_270.updateEstimate(sensor270);
}
uint8_t PingSonar(int trigPin, int echoPin)
{
    long duration;
    int distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    return distance;
}
#endif