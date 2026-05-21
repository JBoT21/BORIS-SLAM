/*
 * main.cpp (PlatformIO version)
 * 
 * ELEGOO Conqueror Robot with SLAM Integration
 * 
 * PlatformIO-specific notes:
 * - File must be named main.cpp (not .ino)
 * - All headers must be explicitly included
 * - Function prototypes must be declared before use
 * - No automatic prototype generation like Arduino IDE
 */

#include <Arduino.h>
#include <avr/wdt.h>

#include <DeviceDriverSet_xxx0.h>
#include <ApplicationFunctionSet_xxx0.h>
#include <SlamFunctionSet.h>


// ===== CONSTANTS =====
#define MODE_NORMAL          0
#define MODE_SLAM_MAPPING    1
#define MODE_SLAM_NAVIGATION 2
#define MODE_SLAM_CALIBRATE  3

// ===== GLOBAL VARIABLES =====
byte currentMode = MODE_NORMAL;
unsigned long lastButtonPress = 0;
const unsigned long buttonDebounceTime = 500;

// ===== FUNCTION PROTOTYPES =====
// PlatformIO requires these to be declared before use
void handleModeSwitch();
void switchToMode(byte newMode);
void executeNormalMode();
void executeSLAMMapping();
void executeSLAMNavigation();
void executeSLAMCalibration();

// ===== SETUP =====
void setup() {
    // Original Conqueror initialization
    Application_FunctionSet.ApplicationFunctionSet_Init();
    wdt_enable(WDTO_2S);
    
    Serial.begin(9600);
    delay(100); // Give serial time to initialize
    
    Serial.println(F("ELEGOO Conqueror with SLAM"));
    Serial.println(F("========================="));
    
    // Initialize SLAM system
    SLAM_FunctionSet.initializeSLAM();
    
    Serial.println(F("\nMode Controls:"));
    Serial.println(F("Send '1' - Normal Conqueror mode"));
    Serial.println(F("Send '2' - SLAM Mapping mode"));
    Serial.println(F("Send '3' - SLAM Navigation mode"));
    Serial.println(F("Send '4' - SLAM Calibration"));
    Serial.println(F("Send 'S' - Stop SLAM"));
    Serial.println(F("Send 'M' - Print SLAM map"));
    Serial.println();
}

// ===== MAIN LOOP =====
void loop() {
    // Watchdog reset
    wdt_reset();
    
    // Check for mode change commands from Serial
    handleModeSwitch();
    
    // Execute current mode
    switch (currentMode) {
        case MODE_NORMAL:
            executeNormalMode();
            break;
            
        case MODE_SLAM_MAPPING:
            executeSLAMMapping();
            break;
            
        case MODE_SLAM_NAVIGATION:
            executeSLAMNavigation();
            break;
            
        case MODE_SLAM_CALIBRATE:
            executeSLAMCalibration();
            break;
    }
}

// ===== MODE SWITCHING FUNCTIONS =====

void handleModeSwitch() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        
        // Debounce
        if (millis() - lastButtonPress < buttonDebounceTime) {
            return;
        }
        lastButtonPress = millis();
        
        switch (command) {
            case '1':
                switchToMode(MODE_NORMAL);
                break;
            case '2':
                switchToMode(MODE_SLAM_MAPPING);
                break;
            case '3':
                switchToMode(MODE_SLAM_NAVIGATION);
                break;
            case '4':
                switchToMode(MODE_SLAM_CALIBRATE);
                break;
            case 'S':
            case 's':
                SLAM_FunctionSet.stopSLAM();
                switchToMode(MODE_NORMAL);
                break;
            case 'M':
            case 'm':
                SLAM_FunctionSet.slam_printMap();
                break;
        }
    }
}

void switchToMode(byte newMode) {
    if (newMode == currentMode) {
        return;
    }
    
    // Stop current mode
    if (currentMode == MODE_SLAM_MAPPING || currentMode == MODE_SLAM_NAVIGATION) {
        SLAM_FunctionSet.stopSLAM();
    }
    
    currentMode = newMode;
    
    // Announce new mode
    Serial.print(F("\n>>> Switching to: "));
    switch (newMode) {
        case MODE_NORMAL:
            Serial.println(F("NORMAL MODE"));
            break;
        case MODE_SLAM_MAPPING:
            Serial.println(F("SLAM MAPPING MODE"));
            Serial.println(F("Robot will now map the environment"));
            break;
        case MODE_SLAM_NAVIGATION:
            Serial.println(F("SLAM NAVIGATION MODE"));
            break;
        case MODE_SLAM_CALIBRATE:
            Serial.println(F("SLAM CALIBRATION MODE"));
            break;
    }
    Serial.println();
}

// ===== MODE EXECUTION FUNCTIONS =====

void executeNormalMode() {
    Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
    Application_FunctionSet.ApplicationFunctionSet_RGB();
    Application_FunctionSet.ApplicationFunctionSet_Tracking();
    Application_FunctionSet.ApplicationFunctionSet_Obstacle();
    Application_FunctionSet.ApplicationFunctionSet_Follow();
    Application_FunctionSet.ApplicationFunctionSet_Rocker();
    Application_FunctionSet.ApplicationFunctionSet_Standby();
    Application_FunctionSet.ApplicationFunctionSet_IRrecv();
    Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();
    
}

void executeSLAMMapping() {
    // Update sensors
    Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
    
    // Execute SLAM mapping
    SLAM_FunctionSet.executeMapping();
    
    delay(100);
}

void executeSLAMNavigation() {
    // Update sensors
    Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
    
    // Execute SLAM navigation
    SLAM_FunctionSet.executeNavigation();
    
    delay(100);
}

void executeSLAMCalibration() {
    // Run calibration routine
    static bool calibrationPrinted = false;
    
    if (!calibrationPrinted) {
        SLAM_FunctionSet.calibrateSLAM();
        calibrationPrinted = true;
        
        Serial.println(F("\nCalibration Commands:"));
        Serial.println(F("Send 'F' - Test forward movement"));
        Serial.println(F("Send 'R' - Test right turn"));
        Serial.println(F("Send 'L' - Test left turn"));
        Serial.println(F("Send 'U' - Test ultrasonic sensor"));
        Serial.println(F("Send 'T[value]' - Set horizontal time (e.g., T20000)"));
        Serial.println(F("Send 'V[value]' - Set vertical time (e.g., V15000)"));
    }
    
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'F':
            case 'f':
                Serial.println(F("Testing forward movement..."));
                SLAM_FunctionSet.slam_moveForward();
                delay(3000);
                SLAM_FunctionSet.slam_moveStop();
                Serial.println(F("Stopped. Measure distance traveled."));
                break;
                
            case 'R':
            case 'r':
                Serial.println(F("Testing right turn..."));
                SLAM_FunctionSet.slam_turnRight();
                Serial.println(F("Done. Check if turn was 90 degrees."));
                break;
                
            case 'L':
            case 'l':
                Serial.println(F("Testing left turn..."));
                SLAM_FunctionSet.slam_turnLeft();
                Serial.println(F("Done. Check if turn was 90 degrees."));
                break;
                
            case 'U':
            case 'u':
                Serial.print(F("Ultrasonic reading: "));
                Serial.print(SLAM_FunctionSet.slam_readUltrasonic());
                Serial.println(F(" cm"));
                break;
                
            case 'T':
            case 't':
                if (Serial.available() > 0) {
                    unsigned long value = Serial.parseInt();
                    Serial.print(F("Setting horizontal time to: "));
                    Serial.println(value);
                    SLAM_FunctionSet.slam_setCalibration(value, SLAM_FunctionSet.unit_TV);
                }
                break;
                
            case 'V':
            case 'v':
                if (Serial.available() > 0) {
                    unsigned long value = Serial.parseInt();
                    Serial.print(F("Setting vertical time to: "));
                    Serial.println(value);
                    SLAM_FunctionSet.slam_setCalibration(SLAM_FunctionSet.unit_TH, value);
                }
                break;
        }
    }
    
    delay(100);
}