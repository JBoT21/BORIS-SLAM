#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"

// Hardware Pin Definitions
const int trigPin = 5;
const int echoPin = 18;

const int motorLeftA  = 26;
const int motorLeftB  = 27;
const int motorRightA = 32;
const int motorRightB = 33;

const int servoPin = 14;

//Objects

Servo scanServo;
MPU6050 imu;

void setup() {

    Serial.println("Initializing...");
    Serial.begin(115200);
    delay(200);
    Serial.println("Serial has been set up.");

    //Ultrasonic Sensor Pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println("Ultrasonic sensor pins have been set up.");

    //Motor Pins
    pinMode(motorLeftA, OUTPUT);
    pinMode(motorLeftB, OUTPUT);
    pinMode(motorRightA, OUTPUT);
    pinMode(motorRightB, OUTPUT);
    Serial.println("Motor pins have been set up.");

    //Servo Initialization
    scanServo.attach(servoPin);
    Serial.println("Servo has been set up.");

    //IMU
    Wire.begin();
    Serial.println("Wire (I2C) has been initialized.");
    imu.initialize();
    Serial.println("MPU6050 IMU has been initialized.");
    Serial.println("Setup Complete - ESP32 Ready!");
}

