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

const int chLA = 0;
const int chLB = 1;
const int chRA = 2;
const int chRB = 3;

void setup() {

    Serial.begin(115200);
        Serial.println("Initializing...");
    delay(200);
    Serial.println("Serial has been set up.");

    //Ultrasonic Sensor Pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println("Ultrasonic sensor pins have been set up.");

    //Motor Pins
    ledcAttachPin(motorLeftA,  chLA);
    ledcAttachPin(motorLeftB,  chLB);
    ledcAttachPin(motorRightA, chRA);
    ledcAttachPin(motorRightB, chRB);

    ledcSetup(chLA, 1000, 8);
    ledcSetup(chLB, 1000, 8);
    ledcSetup(chRA, 1000, 8);
    ledcSetup(chRB, 1000, 8);
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
    long duration = pulseIn(echoPin, HIGH, 20000);
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
    ledcWrite(chLA, speed);
    ledcWrite(chLB, 0);
    ledcWrite(chRA, speed);
    ledcWrite(chRB, 0);
}
void moveBackward(int speed) {
    ledcWrite(chLA, 0);
    ledcWrite(chLB, speed);
    ledcWrite(chRA, 0);
    ledcWrite(chRB, speed);
}

void turnLeft(int speed) {
    ledcWrite(chLA, 0);
    ledcWrite(chLB, speed);
    ledcWrite(chRA, speed);
    ledcWrite(chRB, 0);
}
void turnRight(int speed) {
    ledcWrite(chLA, speed);
    ledcWrite(chLB, 0);
    ledcWrite(chRA, 0);
    ledcWrite(chRB, speed);
}
void stopMotors() {
    ledcWrite(chLA, 0);
    ledcWrite(chLB, 0);
    ledcWrite(chRA, 0);
    ledcWrite(chRB, 0);
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
        int speed = cmd.substring(11).toInt();
        turnRight(speed);
    } else if(cmd == "STOP"){
        stopMotors();
    } else if(cmd.startsWith("SERVO")){
        int angle = cmd.substring(6).toInt();
        angle = constrain(angle, 0, 180);
        scanServo.write(angle);
    }
}

//Main Loop

void loop(){
// Handle incoming commands
if(Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
}
//Send U.S.S
long distance = readUltrasonic();
Serial.printf("ULTRASONIC:%ld\n", distance);

//Send IMU
sendIMU();
delay(50); // May need to adjust delay to handle update rates/ avoid flooding serial


}