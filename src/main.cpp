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

//Ultrasonic Sound Sensor Function

long readUltrasonic(){
    //Clear the trigPin by setting it LOW
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echoPin and calculate distance
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2; // "☝️🤓 The speed of sound is 0.034 cm/us"

    return distance;
}

//IMU Data Function
void sendIMU(){

    float yaw = imu.getRotationZ() / 131.0; // Convert raw data to degrees/s
    float pitch = imu.getRotationX() / 131.0;
    float roll = imu.getRotationY() / 131.0;

    Serial.printf("IMU:%0.2f,%0.2f,%0.2f\n", yaw, pitch, roll);
}

//Motor Control Functions
void moveForward(int speed) {
    analogWrite(motorLeftA, speed);
    analogWrite(motorLeftB, 0);
    analogWrite(motorRightA, speed);
    analogWrite(motorRightB, 0);
}
void moveBackward(int speed) {
    analogWrite(motorLeftA, 0);
    analogWrite(motorLeftB, speed);
    analogWrite(motorRightA, 0);
    analogWrite(motorRightB, speed);
}
void turnLeft(int speed) {
    analogWrite(motorLeftA, 0);
    analogWrite(motorLeftB, speed);
    analogWrite(motorRightA, speed);
    analogWrite(motorRightB, 0);
}
void turnRight(int speed) {
    analogWrite(motorLeftA, speed);
    analogWrite(motorLeftB, 0);
    analogWrite(motorRightA, 0);
    analogWrite(motorRightB, speed);
}
void stopMotors() {
    analogWrite(motorLeftA, 0);
    analogWrite(motorLeftB, 0);
    analogWrite(motorRightA, 0);
    analogWrite(motorRightB, 0);
}

//Command parser

void parseCommand(String cmd){
    cmd.trim();
    if(cmd.startsWith("MOVE FWD")){
        int speed = cmd.substring(9).toInt();
        moveForward(speed);
    } else if(cmd.startsWith("MOVE BACK")){
        int speed = cmd.substring(10).toInt();
        moveBackward(speed);
    } else if(cmd.startsWith("TURN LEFT")){
        int speed = cmd.substring(10).toInt();
        turnLeft(speed);
    } else if(cmd.startsWith("TURN RIGHT")){
        int speed = cmd.substring(10).toInt();
        turnRight(speed);
    } else if(cmd == "STOP"){
        stopMotors();
    } else if(cmd.startsWith("SERVO")){
        int angle = cmd.substring(6).toInt();
        angle = constrain(angle, 0, 180);
        scanServo.write(angle);
    }
}
