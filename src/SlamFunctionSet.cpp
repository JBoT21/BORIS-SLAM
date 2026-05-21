#include "SlamFunctionSet.h"
#include "DeviceDriverSet_xxx0.h"
//#include <cstddef>

SLAMFunctionSet SLAM_FunctionSet;

// Original SLAM calibration
unsigned long unit_TH = 20000;  // Horizontal movement time per block
unsigned long unit_TV = 15000;  // Vertical movement time per block
// Test by making the robot move one "robot length" and timing it

//NOTE: CHANGE BYTE TO UNSIGNED CHAR UNTIL I LEARN WHICH C++ VERSION ARDUINO IS USING.
int mapSize = SLAMFunctionSet::MAX_MAP_SIZE;
bool slamInitialized;

Servo slamServo;
DeviceDriverSet_Motor appMotor; // Create instance of motor control for calibration (duh)
#define FORWARD true
#define BACKWARD false

//MOVEMENT FUNCTIONS
void SLAMFunctionSet::slam_moveForward(){
  //Call Conqueror's motor control instead of AFMotor's motor control
    appMotor.DeviceDriverSet_Motor_control(/*directionA*/ FORWARD, /*speedA*/ 250, /*directionB*/ FORWARD, /*speedB*/ 250, /*controlED*/ true);
}
void SLAMFunctionSet::slam_moveForwardContinuous(){
    appMotor.DeviceDriverSet_Motor_control(/*directionA*/ FORWARD, /*speedA*/ 250, /*directionB*/ FORWARD, /*speedB*/ 250, /*controlED*/ true);
    delay(5000);  // Short delay to allow movement - adjust as needed
}

void SLAMFunctionSet::slam_moveStop() {
    appMotor.DeviceDriverSet_Motor_control(FORWARD,  0,  FORWARD, 0, false);
}

void SLAMFunctionSet::slam_moveOneBlock() {
    slam_moveForward();
    delay(unit_TH/1000);  // Move for the calibrated time per block
    slam_moveStop();
}

void SLAMFunctionSet::slam_turnRight() {
    // Implement tank turn using Conqueror's motor control
     appMotor.DeviceDriverSet_Motor_control(BACKWARD,  200,  FORWARD, 200, true);
     delay(500); //Will need to adjust this delay to achieve a 90 degree turn
     slam_moveStop(); 
}

void SLAMFunctionSet::slam_turnLeft() {
     appMotor.DeviceDriverSet_Motor_control(FORWARD,  200,  BACKWARD, 200, true);
     delay(500); //Will need to adjust this delay to achieve a 90 degree turn
     slam_moveStop(); 
}


//INITIALIZATION FUNCTIONS
void SLAMFunctionSet::initializeSLAM() {
    Serial.print("Initializing SLAM... (1/9)");
    Serial.print("Initializing flags...");
    slamMappingActive = false;
    slamNavigationActive = false;
    slamInitialized = false;
    

    Serial.print("Initializing map storage...(2/9)");
    //int mapSize = MAX_MAP_SIZE;
    currentIteration = 1;
    for (int i = 0; i < mapSize; i++) {
        readings[i] = 0;
    }
    Serial.print("Initializing reading variables...(3/9)");
    l = 0;
    b = 0;
    r = 0;
    Serial.print("Initializing timing variables...(4/9)");
    prev_t = 0;
    curr_t = 0;
    Serial.print("Initializing default calibration values...(5/9)");
    unit_TH = 20000;  // 20 seconds per horizontal block - CALIBRATE THIS!
    unit_TV = 15000;  // 15 seconds per vertical block - CALIBRATE THIS!
    Serial.print("Initializing hardware configuration...(6/9)");

    #if defined(ESP32)
    // ESP32 uses GPIO numbers
    ultrasonicTrigPin = 13;  // Use actual GPIO pins
    ultrasonicEchoPin = 12;
    slamServoPin = 14;
#else
    // Arduino Uno/Mega
    ultrasonicTrigPin = A0;
    ultrasonicEchoPin = A1;
    slamServoPin = 9;
#endif
    slamServoPin = 9;        // May need to use a different pin

    Serial.print("Setting up ultrasonic sensor pins...(7/9)");
    pinMode(ultrasonicTrigPin, OUTPUT);
    pinMode(ultrasonicEchoPin, INPUT);

    Serial.print("Setting up servo...(8/9)");
    slamServo.attach(slamServoPin);
    slamServo.write(90);  // Start with servo facing forward
    delay(500);  // Allow servo to move to position

    slamInitialized = true;
    Serial.print("SLAM initialization complete!(9/9)");
}

void SLAMFunctionSet:: calibrateSLAM(){
    Serial.println("Calibrating SLAM... (1/3)");
    Serial.println("Starting horizontal calibration...");
    // Move forward one block and time it
    unsigned long startTime = micros();
    slam_moveForward();
    delay(20000);  // Move for 20 seconds - adjust as needed
    unsigned long endTime = micros();
    unit_TH = endTime - startTime;
    Serial.print("Horizontal calibration complete! Time per block: " + String(unit_TH) + " microseconds");
    slamInitialized = true;
    }

void SLAMFunctionSet::slam_setCalibration(unsigned long horizontal_time, unsigned long vertical_time) {
    unit_TH = horizontal_time;
    unit_TV = vertical_time;
    Serial.print("Calibration updated: TH=");
    Serial.print(unit_TH);
    Serial.print(" TV=");
    Serial.print(unit_TV);
    }




//SENSOR FUNCTIONS

int SLAMFunctionSet::slam_readUltrasonic() {
    // Use Conqueror's ultrasonic reading if available
    // Or implement direct reading:
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(200);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    return duration / 29 / 2;  // Convert to cm
}

//OBSERVATION FUNCTIONS
int SLAMFunctionSet::slam_lookForward() {
    return slam_readUltrasonic();
}
int SLAMFunctionSet::slam_lookLeft() {
    slamServo.write(90);  // Turn servo to left position
    delay(500);  
    return slam_readUltrasonic();
}
int SLAMFunctionSet::slam_lookRight() {
    slamServo.write(0);  // Turn servo to right position
    delay(500);  
    return slam_readUltrasonic();
}

bool SLAMFunctionSet::slam_checkObstacle() {
    int distance = slam_lookForward();
    return (distance > 0 && distance < 15);  // 15cm threshold should be good
}

//MAPPING FUNCTIONS

void SLAMFunctionSet::executeMapping() {
    if (!slamInitialized) {
        Serial.print("Error: SLAM not initialized!");
        return;
    }
    if (!slamMappingActive) {
        Serial.print("Starting SLAM Mapping...");
        slamMappingActive = true;
        currentIteration = 1;
        slam_clearMap();
    }
    // Execute one mapping iteration
    slam_mappingIteration();
}

void SLAMFunctionSet::slam_mappingIteration() {
  // This is the core SLAM mapping algorithm
  // Ported from the original UltraSLAM.ino (From the UltraSLAM github)
    
    Serial.print("Iteration: ");
    Serial.print(currentIteration);  
    // Reset current readings
    l = 0;
    b = 0;
    r = 0;
    
    int frontDistance = slam_lookForward();
    if (frontDistance > 15) {  // Path is clear
        if (currentIteration % 2 != 0) {  // Odd iterations - move forward
            prev_t = millis();
            
            // Move forward until obstacle detected
            while (slam_lookForward() > 15) {
                Serial.print("Moving forward...");
                slam_moveForwardContinuous();
            }
            slam_moveStop();
            
        } else { // On even iterations, do side movements
            if (currentIteration % 4 != 0) {  // Turn right pattern
                slam_turnRight();
                slam_moveOneBlock();
                
                if (slam_lookLeft() > 20) {
                    slam_turnLeft();
                    prev_t = millis();
                    
                    while (slam_lookForward() > 10) {
                        Serial.print("Moving in left corridor...");
                        slam_moveForwardContinuous();
                    }
                    slam_moveStop();
                    
                    curr_t = millis();
                    l = (curr_t - prev_t) / unit_TH;
                    
                    slam_turnRight();
                    slam_moveForward();
                    delay(curr_t - prev_t);
                    slam_moveStop();
                }
                slam_turnRight();
                currentIteration++;
                
            } else {  // Turn left pattern (every 4th iteration)
                slam_turnLeft();
                slam_moveOneBlock();
                
                if (slam_lookRight() > 20) {
                    slam_turnRight();
                    prev_t = millis();
                    
                    while (slam_lookForward() > 10) {
                        Serial.print("Moving in right corridor...");
                        slam_moveForwardContinuous();
                    }
                    slam_moveStop();
                    curr_t = millis();
                    r = (curr_t - prev_t) / unit_TH;
                    slam_turnLeft();
                    slam_moveForward();
                    delay(curr_t - prev_t);
                    slam_moveStop();
                }
                slam_turnLeft();
                currentIteration++;
            }
        }
        
    } else {  // Obstacle ahead
        slam_moveStop();
        curr_t = millis();
        b = (curr_t - prev_t) / unit_TH;
        
        slam_addReading(currentIteration);
        currentIteration++;
        
        // Decide which way to turn
        if (slam_lookRight() > 20) {
            slam_turnRight();
        } else if (slam_lookLeft() > 20) {
            slam_turnLeft();
        } else {
            // Dead end - turn around
            slam_turnRight();
            slam_turnRight();
        }
    }
    // Save reading for this iteration
    slam_addReading(currentIteration);
    
    // Check if mapping is complete
    if (currentIteration > 50) {  // Current safety limit
        Serial.print("Mapping complete (iteration limit reached)");
        stopSLAM();
        slam_printMap();
    }
}
void SLAMFunctionSet::executeNavigation() {
    // TODO: Implement navigation using the generated map
    Serial.print("SLAM Navigation not yet implemented");
}
 
void SLAMFunctionSet::stopSLAM() {
    slamMappingActive = false;
    slamNavigationActive = false;
    slam_moveStop();
    Serial.print("SLAM Stopped");
}

//MAP MANAGEMENT FUNCTIONS

void SLAMFunctionSet::slam_addReading(int iteration) {
    int index = iteration - 1;
    
    if (index >= mapSize) {
        slam_expandReadingsArray();
    }
    
    if (iteration % 2 == 0) {
        if ((iteration & 3) == 0) {
            readings[index] = l;
        } else {
            readings[index] = r;
        }
    } else {
        if (iteration == 1) {
            readings[0] = 0;
            readings[1] = (curr_t - prev_t) / unit_TH;
            readings[2] = 0;
            base_size = readings[1];
        } else {
            readings[index] = b;
        }
    }
    
    Serial.print("Reading added at index ");
    Serial.print(index);
    Serial.print(": ");
    Serial.print(readings[index]);
}

void SLAMFunctionSet::slam_expandReadingsArray() {
    Serial.print("Warning: Map size limit reached!");
    // In the original code, this would expand the array
    // For right now, this will just be a warning
}

void SLAMFunctionSet::slam_printMap() {
    Serial.print("\n=== SLAM MAP DATA ===");
    Serial.print("Base size: ");
    Serial.print(base_size);
    Serial.print("Total iterations: ");
    Serial.print(currentIteration);
    Serial.print("\nReadings (L, B, R format):");
    
    for (int i = 0; i < currentIteration && i < mapsize; i++) {
        Serial.print("Iteration ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(readings[i]);
    }
    Serial.print("=====================\n");
}

void SLAMFunctionSet::slam_clearMap() {
    for (int i = 0; i < mapSize; i++) {
        readings[i] = 0;
    }
    currentIteration = 1;
    Serial.print("Map cleared");
}
 
void SLAMFunctionSet::slam_saveMap() {
    // TODO: Implement EEPROM storage
    Serial.print("Map save not yet implemented");
    Serial.print("Printing map to serial instead:");
    slam_printMap();
}
void SLAMFunctionSet::slam_loadMap() {
    // TODO: Implement EEPROM loading
    Serial.print("Map load not yet implemented");
}
 
// Create global instance
//SLAMFunctionSet SLAM_FunctionSet;