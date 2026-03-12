/*
 * Unified Wind Tunnel Control System
 *
 * Description: An all-in-one system for the Teensy 4.1. 
 * Allows the user to select between Servo Calibration (Wheel Mode), 
 * Load Cell Calibration, and the Main Wind Tunnel Test Mode.
 * Main Test Mode drives the servo in Wheel Mode to avoid positional
 * wrapping limits and runs for exact commanded distances.
 */

#include <Arduino.h>
#include <STSServoDriver.h>
#include <HX711.h>
#include <SD.h>
#include <SPI.h>
#include <utils/task_dispatcher.hpp>
#include <utils/nonblocking_delay.hpp>

// --- Configuration & Hardware Pins ---
const byte SERVO_ID = 1;
const int SERVO_DIR_PIN = 0; // Handled by Waveshare board

// Teensy 4.1 Load Cell Pins
// Note: Each Load Cell MUST be on its own pair of pins to be read simultaneously.
const int LOADCELL1_DOUT_PIN = 2;
const int LOADCELL1_SCK_PIN = 3;
const int LOADCELL2_DOUT_PIN = 4;
const int LOADCELL2_SCK_PIN = 5;

// * DEFAULT CALIBRATION FACTORS *
float CAL_FACTOR_1 = 1000.0; 
float CAL_FACTOR_2 = 1000.0;

STSServoDriver servo;
HX711 scale1;
HX711 scale2;
File logFile;

// Global State for Wind Tunnel Test
float weight1 = 0.0;
float weight2 = 0.0;
long targetCountsToMove = 0;
long totalAccumulatedCounts = 0;
int previousPosForTracking = 0;
bool isMotorRunningTest = false;
bool isRunningWheelMode = false;
unsigned long currentTestStartTime = 0;

// Setup Dispatcher
xcore::Dispatcher<5> dispatcher; 

// --- Application State ---
enum AppState {
    MODE_MENU,
    MODE_CALIBRATE_SERVO,
    MODE_CALIBRATE_LOADCELL,
    MODE_WIND_TUNNEL
};

AppState currentState = MODE_MENU;

// Forward declarations for Tasks
void readLoadCellsTask();
void printAndLogDataTask();
void handleSerialCommandsTask();

void printMenu() {
    Serial.println("\n=================================");
    Serial.println("  WIND TUNNEL CONTROL SYSTEM   ");
    Serial.println("=================================");
    Serial.println("Select an operating mode:");
    Serial.println("  1: Calibrate Servo (5 Revolutions - Wheel Mode)");
    Serial.println("  2: Calibrate Load Cells");
    Serial.println("  3: Main Wind Tunnel Test");
    Serial.println("=================================");
    Serial.print("Enter choice (1, 2, or 3): ");
}

void setupHardwareGeneral() {
    Serial.println("\nInitializing Hardware...");
    
    // Servo Serial
    Serial7.begin(1000000);
    if (!servo.init(SERVO_DIR_PIN, &Serial7)) {
        Serial.println("WARNING: Servo initialization failed! Check wiring on Pins 28/29.");
    } else {
        Serial.println("Servo initialized.");
        // Ensure starting in Servo Mode to hold position initially
        servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0);
    }

    // Load Cells
    scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
    scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
    Serial.println("Load Cells initialized.");
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    
    setupHardwareGeneral();
    printMenu();
}

// ==========================================
// MODE 1: SERVO CALIBRATION (WHEEL MODE)
// ==========================================
bool servoCalibrationSetupDone = false;

void runServoCalibration() {
    if (!servoCalibrationSetupDone) {
        Serial.println("\n\n--- SERVO CALIBRATION (WHEEL MODE) ---");
        
        servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0);
        delay(50);
        Serial.println("Homing to Position 0...");
        servo.setTargetPosition(SERVO_ID, 0, 2000);
        delay(2000); // Wait to home
        
        Serial.println("\nWaiting for user input...");
        Serial.println("Type 'y' and press Enter to START the 5-revolution test.");
        servoCalibrationSetupDone = true;
        return;
    }

    // Wait for user to type 'y'
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'q' || c == 'Q') {
            servoCalibrationSetupDone = false;
            currentState = MODE_MENU;
            printMenu();
            return;
        }
        if (c == 'y' || c == 'Y') {
            
            Serial.println("\n[ STARTING TEST ]");

            // Switch to Wheel Mode
            servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 1);
            delay(50);

            int testVelocity = 3350;
            Serial.print("Commanded Velocity: "); Serial.print(testVelocity); Serial.println(" counts/sec");
            Serial.println("Running exactly 5 full revolutions...");
            
            servo.setTargetVelocity(SERVO_ID, testVelocity);
            
            unsigned long startTime = millis();
            unsigned long printTimer = startTime;
            
            int previousPos = servo.getCurrentPosition(SERVO_ID);
            long localAccumulatedCounts = 0;
            long targetTestCounts = 4096 * 5;

            while (localAccumulatedCounts < targetTestCounts) {
                int currentPos = servo.getCurrentPosition(SERVO_ID);
                
                // Wheel mode wraps
                if (currentPos < previousPos - 2000) { 
                    localAccumulatedCounts += (4096 - previousPos) + currentPos;
                } else {
                    localAccumulatedCounts += (currentPos - previousPos);
                }
                previousPos = currentPos;

                if (millis() - printTimer >= 100) {
                    printTimer = millis();
                    Serial.print("Time: "); Serial.print(millis()/1000.0, 1);
                    Serial.print("s | Pos: "); Serial.print(currentPos);
                    Serial.print(" (Total: "); Serial.print(localAccumulatedCounts);
                    Serial.println(")");
                }
            }
            
            unsigned long timeTaken = millis() - startTime;
            Serial.println("\nStopping Motor...");
            servo.setTargetVelocity(SERVO_ID, 0);
            servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0);
            
            Serial.println("\n--- Calibration Complete ---");
            Serial.print("Total Time: "); Serial.print(timeTaken); Serial.println(" ms");
            Serial.print("Average Velocity: "); Serial.print( (float)targetTestCounts / (timeTaken / 1000.0) ); Serial.println(" counts/sec");
            
            Serial.println("\nReturning to Menu in 5 seconds...");
            delay(5000);
            servoCalibrationSetupDone = false;
            currentState = MODE_MENU;
            printMenu();
        }
    }
}

// ==========================================
// MODE 2: LOAD CELL CALIBRATION
// ==========================================
int loadCellCalibrationStep = 0; // 0: Setup, 1: Wait Scale 1, 2: Wait Scale 2
float tempCalFactor1 = 1000.0;

void runLoadCellCalibration() {
    if (loadCellCalibrationStep == 0) {
        Serial.println("\n\n--- LOAD CELL CALIBRATION MODE ---");
        Serial.println("Taring scales... Please wait.");
        scale1.set_scale();
        scale2.set_scale();
        scale1.tare();
        scale2.tare();
        
        Serial.println("\n[ STEP 1: Calibrate Load Cell 1 ]");
        Serial.println("1. Make sure Load Cell 1 is EMPTY.");
        Serial.println("2. Place a KNOWN weight (e.g., 1kg) on Load Cell 1 ONLY.");
        Serial.println("3. Type the known weight (e.g. '1.0') and press Enter.");
        Serial.println("   (Or type 'q' to quit to menu).");
        loadCellCalibrationStep = 1;
    }

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.equalsIgnoreCase("q")) {
            loadCellCalibrationStep = 0;
            currentState = MODE_MENU;
            printMenu();
            return;
        }

        if (input.length() > 0) {
            float knownWeight = input.toFloat();
            if (knownWeight > 0) {
                
                if (loadCellCalibrationStep == 1) {
                    Serial.print("Calculating Load Cell 1 for weight: "); Serial.println(knownWeight);
                    long reading1 = scale1.get_value(10);
                    tempCalFactor1 = reading1 / knownWeight;
                    Serial.print("-> Scale 1 Calibration Factor: "); Serial.println(tempCalFactor1);
                    
                    Serial.println("\n[ STEP 2: Calibrate Load Cell 2 ]");
                    Serial.println("1. Remove the weight from Load Cell 1.");
                    Serial.println("2. Place the KNOWN weight on Load Cell 2 ONLY.");
                    Serial.println("3. Type the known weight (e.g. '1.0') and press Enter.");
                    loadCellCalibrationStep = 2;
                } 
                else if (loadCellCalibrationStep == 2) {
                    Serial.print("Calculating Load Cell 2 for weight: "); Serial.println(knownWeight);
                    long reading2 = scale2.get_value(10);
                    float calFactor2 = reading2 / knownWeight;
                    Serial.print("-> Scale 2 Calibration Factor: "); Serial.println(calFactor2);
                    
                    Serial.println("\nUpdating system calibration factors in memory for Mode 3...");
                    CAL_FACTOR_1 = tempCalFactor1;
                    CAL_FACTOR_2 = calFactor2;
                    
                    Serial.println("Returning to Menu in 4 seconds...");
                    delay(4000);
                    loadCellCalibrationStep = 0;
                    currentState = MODE_MENU;
                    printMenu();
                }
            }
        }
    }
}

// ==========================================
// MODE 3: WIND TUNNEL TEST (WHEEL MODE)
// ==========================================
bool windTunnelSetupDone = false;

// Task 1: Non-Blocking Load Cell Retrieval
void readLoadCellsTask() {
    if (scale1.is_ready()) { weight1 = scale1.get_units(1); }
    if (scale2.is_ready()) { weight2 = scale2.get_units(1); }
}

// Task 2: High-Speed Distance Tracking & Motor Stopping
void motorTrackingTask() {
    if (isMotorRunningTest) {
        if (isRunningWheelMode) {
            // WHEEL MODE: Manual distance tracking
            int currentPos = servo.getCurrentPosition(SERVO_ID);
            
            // Handle wheel mode wrap-around (0 to 4095)
            if (currentPos < previousPosForTracking - 2000) { 
                totalAccumulatedCounts += (4096 - previousPosForTracking) + currentPos;
            } else {
                totalAccumulatedCounts += (currentPos - previousPosForTracking);
            }
            previousPosForTracking = currentPos;

            // Early stop compensation for high-speed momentum drift
            // If the target is very small (like R0.25 = 1024 counts), a fixed 1200 drift 
            // compensation will instantly trigger a stop. We must scale it.
            int driftCompensation = 1200; 
            if (targetCountsToMove <= 2000) {
                driftCompensation = targetCountsToMove * 0.25; // Small drift for short distances
            }
            
            if (totalAccumulatedCounts >= (targetCountsToMove - driftCompensation)) {
                servo.setTargetVelocity(SERVO_ID, 0); // Stop motor early to coast
                isMotorRunningTest = false;
                Serial.println("\n[ TEST COMPLETE - WHEEL TARGET REACHED ]");
                Serial.println("Motor stopped. Enter a new command (or 'q' to quit).");
                // Return to servo mode to hold position
                servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0);
            }
        } else {
            // SERVO MODE: Hardware tracking
            byte moving = servo.readRegister(SERVO_ID, STSRegisters::MOVING_STATUS);
            
            if (moving == 0 && (millis() - currentTestStartTime) > 50) {
                isMotorRunningTest = false;
                Serial.println("\n[ TEST COMPLETE - SERVO TARGET REACHED ]");
                Serial.println("Motor stopped. Enter a new command (or 'q' to quit).");
            }
        }
    }
}

// Task 3: Data Aggregation and Output
void printAndLogDataTask() {
    // Only log if the motor is actively running a test to avoid massive idle logs
    if (!isMotorRunningTest) return; 

    // Calculate elapsed time since this specific run started
    unsigned long elapsed_ms = millis() - currentTestStartTime;

    int pos = servo.getCurrentPosition(SERVO_ID);
    int speed = servo.getCurrentSpeed(SERVO_ID);
    int temp = servo.getCurrentTemperature(SERVO_ID);
    int load = servo.getCurrentCurrent(SERVO_ID);
    byte moving = servo.readRegister(SERVO_ID, STSRegisters::MOVING_STATUS);
    byte voltageRaw = servo.readRegister(SERVO_ID, STSRegisters::CURRENT_VOLTAGE);
    float voltage = voltageRaw / 10.0;
    
    String csvLine = String(elapsed_ms) + "," +
                     String(pos) + "," +
                     String(voltage, 1) + "," +
                     String(temp) + "," +
                     String(speed) + "," +
                     String(load) + "," +
                     String(moving) + "," +
                     String(totalAccumulatedCounts) + "," +
                     String(weight1, 3) + "," +
                     String(weight2, 3);
                     
    Serial.println(csvLine);
    if (logFile) {
        logFile.println(csvLine);
        // We flush continuously so data isn't lost if the Teensy powers off mid-spin
        logFile.flush(); 
    }
}

// Task 4: Command Listener
void handleSerialCommandsTask() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.equalsIgnoreCase("q")) {
            Serial.println("\nExiting Wind Tunnel Mode. Returning to Menu...");
            servo.setTargetVelocity(SERVO_ID, 0);
            servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0); // Back to Servo Mode
            windTunnelSetupDone = false;
            isMotorRunningTest = false;
            if(logFile) logFile.close();
            currentState = MODE_MENU;
            printMenu();
            return;
        }

        // Only accept new commands if a test isn't currently running
        if (input.length() > 0 && !isMotorRunningTest) {
            char cmdType = input.charAt(0);
            float value = input.substring(1).toFloat();
            long newTargetCounts = 0;
            
            // Parse formats: 'R2' for 2 Revs, 'D90' for 90 Deg
            if (cmdType == 'R' || cmdType == 'r') {
                targetCountsToMove = (long)(value * 4096.0);
            } else if (cmdType == 'D' || cmdType == 'd') {
                targetCountsToMove = (long)((value / 360.0) * 4096.0);
            }
            
            if (targetCountsToMove > 0) {
                // We use Wheel Mode for BOTH R and D commands because the STS3215 
                // positional mode physically limits at 4096 counts (1 revolution)
                // and cannot reliably do multi-turn absolute positioning without complex 
                // offset math. Wheel mode with software tracking handles any distance perfectly.
                isRunningWheelMode = true; 
                
                Serial.print("\n--- Starting New Run (WHEEL MODE): Target = ");
                Serial.print(targetCountsToMove);
                Serial.println(" counts ---");
                
                totalAccumulatedCounts = 0;
                previousPosForTracking = servo.getCurrentPosition(SERVO_ID);
                
                servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 1);
                delay(10);
                
                Serial.println("Timestamp_ms,Position,Voltage_V,Temp_C,Speed,Load_A,Moving,TargetCounts,Weight1_g,Weight2_g");
                isMotorRunningTest = true;
                currentTestStartTime = millis(); 
                servo.setTargetVelocity(SERVO_ID, 3350); 
            }
        }
    }
}

void runWindTunnelTest() {
    if (!windTunnelSetupDone) {
        Serial.println("\n\n--- MAIN WIND TUNNEL TEST ---");
        
        // We will run this mode entirely in standard Servo Mode to prevent drift.
        servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0);
        delay(50);
        
        // Setup SD Card with a unique file name based on startup time
        if (SD.begin(BUILTIN_SDCARD)) {
            String filename = "TEST_" + String(millis()) + ".CSV";
            logFile = SD.open(filename.c_str(), FILE_WRITE);
            if (logFile) {
                logFile.println("Timestamp_ms,Position,Voltage_V,Temp_C,Speed,Load_A,Moving,AccumulatedCounts,Weight1_kg,Weight2_kg");
                Serial.print("SD Card Logging Enabled: ");
                Serial.println(filename);
            } else {
                Serial.println("WARNING: Failed to open file on SD Card.");
            }
        } else {
            Serial.println("WARNING: SD Card not found or failed to initialize.");
        }

        // Apply Calibration Factors
        scale1.set_scale(CAL_FACTOR_1);
        scale1.tare();
        scale2.set_scale(CAL_FACTOR_2);
        scale2.tare();

        Serial.print("Scale 1 Factor applied: "); Serial.println(CAL_FACTOR_1);
        Serial.print("Scale 2 Factor applied: "); Serial.println(CAL_FACTOR_2);
        Serial.println("Load Cells Tared.");
        
        Serial.println("\n[ HOW TO RUN A TEST ]");
        Serial.println("Input 'R' followed by revolutions (e.g. R1 for 360°, R1.25 for 450°, R5 for 5 revs).");
        Serial.println("Input 'D' followed by degrees (e.g. D180 for half rev, D720 for 2 revs).");
        Serial.println("Type 'q' to quit to menu.");

        // Register new tasks for dispatcher
        dispatcher.clear();
        dispatcher << xcore::Task(readLoadCellsTask, 5, millis)        // 200Hz Load Cell Polling
                   << xcore::Task(motorTrackingTask, 1, millis)        // 1000Hz Ultra-Fast Position Tracking
                   << xcore::Task(printAndLogDataTask, 10, millis)     // 100Hz (MAX) Serial & SD Logging
                   << xcore::Task(handleSerialCommandsTask, 20, millis); // 50Hz UI polling
                   
        windTunnelSetupDone = true;
    }

    // Execute the non-blocking dispatcher
    dispatcher();
}

// ==========================================
// MAIN LOOP STATE MACHINE
// ==========================================
void loop() {
    switch (currentState) {
        case MODE_MENU:
            if (Serial.available()) {
                char choice = Serial.read();
                if (choice == '1') {
                    currentState = MODE_CALIBRATE_SERVO;
                    servoCalibrationSetupDone = false;
                } else if (choice == '2') {
                    currentState = MODE_CALIBRATE_LOADCELL;
                    loadCellCalibrationStep = 0;
                } else if (choice == '3') {
                    currentState = MODE_WIND_TUNNEL;
                    windTunnelSetupDone = false;
                }
            }
            break;
            
        case MODE_CALIBRATE_SERVO:
            runServoCalibration();
            break;
            
        case MODE_CALIBRATE_LOADCELL:
            runLoadCellCalibration();
            break;
            
        case MODE_WIND_TUNNEL:
            runWindTunnelTest();
            break;
    }
}