#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "esp32-hal-ledc.h"
#include "MPU6050.h"
#include "esp32-hal-gpio.h"
//#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define NUM_STATIONARY_SAMPLES 10
#define ACC_THRESHOLD 0.1  // Threshold in mps^2 to consider the robot as moving or stationary
#define GYRO_THRESHOLD 3   // Threshold in deg/sec to consider the robot as moving or stationary

// Hardware Pin Definitions (Adjust ESP32 pins as needed)
// Note: GPIO 6-11 are reserved for flash memory on most ESP32 boards
// (So don't use those)

// Pin definitions
const int trigPin = 4;
const int echoPin = 15;

// TB6612 Motor Driver Pins
#define PWMA 5 
#define AIN1 19
#define AIN2 18 
#define PWMB 23
#define BIN1 21
#define BIN2 22
//#define STBY 2

//MMA8451 Accelerometer Pins (I2C)
#define SDA_PIN 33
#define SCL_PIN 32

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68  // Default address (0x69 if AD0 pin is high)
// Global objects
MPU6050 mpu(MPU6050_ADDR, &Wire);
//Adafruit_MMA8451 mma = Adafruit_MMA8451();

float pitch=0, roll=0, yaw = 0; //gyroscope offsets from start
float x=0, y=0, z=0; //accelerometer calculated position from start
float x_vel=0, y_vel=0, z_vel=0; //linear velocity in m/s from mpu
int stationary_counter = 0; // Counter for how many consecutive samples indicate stationary
unsigned long lastTime = 0;

// MPU6050 raw data storage
int16_t ax, ay, az;
int16_t gx, gy, gz;

// MPU 6050 calibrated offsets
long ax_offset = 0, ay_offset = 0, az_offset = 0;
long gx_offset = 0, gy_offset = 0, gz_offset = 0;

void initIMU() {
    int i;
    long sumX=0, sumY=0, sumZ=0, sumGX=0, sumGY=0, sumGZ=0;

    Serial.println("Initializing IMU...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("I2C Initialized");
    Wire.beginTransmission(MPU6050_ADDR);
    int error = Wire.endTransmission();
    
    if (error != 0) {
        Serial.printf("Failed to find MPU6050 at address 0x%02X\n", MPU6050_ADDR);
        Serial.println("Checking alternative address 0x69...");
        
        Wire.beginTransmission(0x69);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.println("Found MPU6050 at 0x69, updating address");
            // Update the mpu address
            mpu.setSlaveAddress(0,0x69);
        } else {
            Serial.println("MPU6050 not found at either address!");
            while (1);  // Stops here if IMU not found
        }
    }

    mpu.initialize();
    mpu.setDLPFMode(4);
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection test failed!");
        while (1);
    }
    Serial.println("MPU6050 Initialized!");
    
    // Set up the accelerometer and gyro
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // 250 deg/sec
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // 2g

    /*Calibrate Accelerometer*/
    /*Device must be sitting still during this time*/
    for (i=0; i<200; i++)
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += ax;
        sumY += ay;
        sumZ += az - 16384; // Subtract 1g from Z-axis to account for gravity
        sumGX += gx;
        sumGY += gy;
        sumGZ += gz;
        delay(10);
    }

    ax_offset = sumX / 200;
    ay_offset = sumY / 200;
    az_offset = sumZ / 200;
    gx_offset = sumGX / 200;
    gy_offset = sumGY / 200;
    gz_offset = sumGZ / 200;


    lastTime = micros();
    Serial.println("IMU Ready");
}

void sendIMU() {
    float x_mps2, y_mps2, z_mps2; //linear accelerations in mps
    float xOff, yOff, zOff; //linear offsets from last position.
    float gyroX_dps, gyroY_dps, gyroZ_dps;
    float dt;
    float acc_magnitude;
    float gyro_magnitude;
    int i;

    /* Timekeeping */
    unsigned long now = micros();
    dt = (now - lastTime) / 1e6;  // Converts to seconds
    lastTime = now;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /* Apply calibrated offsets*/
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;
    
    /* Convert 3-axis acceleration to m/s^2*/
    x_mps2 = ax / 16384.0 * 9.8;
    y_mps2 = ay / 16384.0 * 9.8;
    z_mps2 = az / 16384.0 * 9.8;

    // Convert raw gyro values to degrees per second (250 deg/sec range)
    gyroX_dps = gx / 131.0;
    gyroY_dps = gy / 131.0;
    gyroZ_dps = gz / 131.0;

    /* Compute total acceleration magnitude in m/s^2*/
    acc_magnitude = sqrt(x_mps2*x_mps2 + y_mps2*y_mps2 + z_mps2*z_mps2) - 9.8; // Subtract gravity to get net acceleration
    gyro_magnitude = sqrt(gyroX_dps*gyroX_dps + gyroY_dps*gyroY_dps + gyroZ_dps*gyroZ_dps);

    /* Detect stationary state */
    if (gyro_magnitude < GYRO_THRESHOLD && acc_magnitude < ACC_THRESHOLD)
        stationary_counter++;
    else
        stationary_counter = 0; // Reset counter if we detect movement

    
    if (stationary_counter >= NUM_STATIONARY_SAMPLES)
    {
        x_vel = 0;
        y_vel = 0;
        z_vel = 0;
        gyroX_dps = 0;
        gyroY_dps = 0;
        gyroZ_dps = 0;
    }
    else
    {
        /*Only integrate velocity if moving*/
        x_vel += x_mps2 * dt;
        y_vel += y_mps2 * dt;
        z_vel += z_mps2 * dt;
    }
    
    xOff = x_vel*dt;
    yOff = y_vel*dt;
    zOff = z_vel*dt;

    /* Compute total X,Y,Z positions*/
    x += xOff;
    y += yOff;
    z += zOff;
    
    // Integrate gyro to get angles
    pitch += gyroX_dps*dt;
    roll += gyroY_dps*dt;
    yaw += gyroZ_dps * dt;
    
    // Normalize yaw to 0-360
    while (pitch < 0) pitch += 360;
    while (pitch >= 360) pitch -= 360;
    while (roll < 0) roll += 360;
    while (roll >= 360) roll -= 360;
    while (yaw < 0) yaw += 360;
    while (yaw >= 360) yaw -= 360;

    //Serial.printf("IMU:%0.2f,0.00,0.00\n", yaw);
    Serial.printf("IMU\n");
    Serial.printf("X: %0.2fm, Y: %0.2fm, Z: %0.2fm\n", x, y, z);
    Serial.printf("Pitch: %0.2f, Roll: %0.2f, Yaw: %0.2f\n", pitch, roll, yaw);
    // Serial.printf("RAWX: %d, RAWY: %d, RAWZ: %d\n", ax, ay, az);
    // Serial.printf("RawPitch: %d, RawRoll: %d, RawYaw: %d\n", gx, gy, gz);
}

void setup() {
    Serial.begin(115200);  // Used to communicate w/ ESP32 (Sensor data output)
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // Used to communicate w/ Jetson (Command input)
    Serial.println("Serial Initialized");

    // Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println("Ultrasonic Initialized");

    // Motor pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    //pinMode(STBY, OUTPUT);
    //digitalWrite(STBY, HIGH);
    
    // PWM channels for motors
    ledcSetup(0, 1000, 8);
    ledcAttachPin(PWMA, 0);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(PWMB, 1);
    Serial.println("Motors Initialized");

    // IMU
    initIMU();

    /* MMA8451 Accelerometer
    if (!mma.begin()) {
        Serial.println("Couldn't start MMA8451");
        while (1);
    }
    Serial.println("MMA8451 found!");
    
    mma.setRange(MMA8451_RANGE_2_G);
    
    Serial.print("Range = ");
    Serial.print(2 << mma.getRange());
    Serial.println("G");
}
*/
}


// Ultrasonic distance reading
long readUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 60000);
    Serial.printf("ULTRA_RAW:%ld\n", duration);
    return duration * 0.034 / 2;  // Convert to cm
}

// Motor Control Functions
void leftMotor(int speed) {
    // Clamp speed to valid range
    speed = constrain(speed, -255, 255);
    
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
    // Clamp speed to valid range
    speed = constrain(speed, -255, 255);
    
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

// Stop all motors
void stopMotors() {
    leftMotor(0);
    rightMotor(0);
}

void loop() {
    // Jetson commands handling
    if (Serial2.available()) {
        char cmd = Serial2.read();

        if (cmd == 'F') {
            leftMotor(200);
            rightMotor(200);
        }
        else if (cmd == 'B') {
            leftMotor(-200);
            rightMotor(-200);
        }
        else if (cmd == 'L') {
            leftMotor(-150);
            rightMotor(150);
        }
        else if (cmd == 'R') {
            leftMotor(150);
            rightMotor(-150);
        }
        else if (cmd == 'S') {
            stopMotors();
        }
    }

    // Send ultrasonic data
    long distance = readUltrasonic();
    Serial.printf("ULTRASONIC:%ld\n", distance);
    Serial.printf("\n");// Blank line for better readability

    // Send IMU (gyro-based yaw)
    sendIMU();

    // Send accelerometer data
   // mma.read();
    //Serial.printf("MMA8451:%0.2f,%0.2f,%0.2f\n", mma.x_g, mma.y_g, mma.z_g);
    
    delay(50);
}