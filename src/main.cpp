#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"

//Global Variables
float yaw = 0;
unsigned long lastTime = 0;

// Hardware Pin Definitions (Adjust ESP32 pins as needed)
//Note: GPIO 6-11 are reserved for flash memory on most ESP32 boards
//(So dont use those)

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"

//Pin definitions
const int trigPin = 4;
const int echoPin = 15;

// TB6612 Motor Driver Pins
#define PWMA 18
#define AIN1 23
#define AIN2 22
#define PWMB 19
#define BIN1 21
#define BIN2 5
#define STBY 2

// Servo
const int servoPin = 14;

//Global objecrs
Servo scanServo;
MPU6050 imu;

float yaw = 0;
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);

    // Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Motor pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    // PWM channels
    ledcAttachPin(PWMA, 0);
    ledcAttachPin(PWMB, 1);
    ledcSetup(0, 20000, 8);
    ledcSetup(1, 20000, 8);

    // Servo
    scanServo.attach(servoPin);

    // IMU
    Wire.begin();
    imu.initialize();

    lastTime = micros();
}

//Ultrasonic 
long readUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000);
    return duration * 0.034 / 2;
}

//IMU
void sendIMU() {
    float gyroZ = imu.getRotationZ() / 131.0;
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    yaw += gyroZ * dt;
    if (yaw < 0) yaw += 360;
    if (yaw >= 360) yaw -= 360;

    Serial.printf("IMU:%0.2f,0.00,0.00\n", yaw);
}

//Motor Control
void leftMotor(int speed) {
    if (speed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        speed = -speed;
    }
    ledcWrite(0, speed);
}

void rightMotor(int speed) {
    if (speed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        speed = -speed;
    }
    ledcWrite(1, speed);
}


void loop() {

    // Jetson commands get handled here
    if (Serial2.available()) {
        char cmd = Serial2.read();

        if (cmd == 'F') { leftMotor(200); rightMotor(200); }
        if (cmd == 'B') { leftMotor(-200); rightMotor(-200); }
        if (cmd == 'L') { leftMotor(-150); rightMotor(150); }
        if (cmd == 'R') { leftMotor(150); rightMotor(-150); }
        if (cmd == 'S') { leftMotor(0); rightMotor(0); }
    }

    // Send ultrasonic
    long distance = readUltrasonic();
    Serial.printf("ULTRASONIC:%ld\n", distance);

    // Send IMU
    sendIMU();

    delay(50);
}