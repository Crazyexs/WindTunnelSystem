/*
 * Multi-Test System
 *
 * Description: Calibrates and tests the Feetech STS3215 servo in "Wheel Mode" 
 * AND provides a separate test mode to read raw values from the HX711 Load Cells.
 */

#include <Arduino.h>
#include <STSServoDriver.h>
#include <HX711.h>

STSServoDriver servo;
const byte SERVO_ID = 1;
const int SERVO_DIR_PIN = 0; // Dummy pin

// Teensy 4.1 Load Cell Pins
const int LOADCELL1_DOUT_PIN = 2;
const int LOADCELL1_SCK_PIN = 3;
const int LOADCELL2_DOUT_PIN = 4;
const int LOADCELL2_SCK_PIN = 5;

HX711 scale1;
HX711 scale2;

enum AppState {
    MODE_MENU,
    MODE_SERVO_TEST,
    MODE_LOADCELL_TEST
};

AppState currentState = MODE_MENU;
bool testSetupDone = false;

void printMenu() {
    Serial.println("\n===========================================");
    Serial.println("   HARDWARE DEBUG MENU");
    Serial.println("===========================================");
    Serial.println("  1: Servo Wheel Mode (5-Revolution Test)");
    Serial.println("  2: Load Cell Raw Value Test (10 seconds)");
    Serial.println("===========================================");
    Serial.print("Select test (1 or 2): ");
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("\n-> Initializing Hardware...");

    // Init Servo
    Serial7.begin(1000000);
    if (servo.init(SERVO_DIR_PIN, &Serial7)) {
        Serial.println("-> Servo connection established.");
        servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0); // Start in servo mode
    } else {
        Serial.println("-> ERROR: Servo connection failed! Check Pins 28/29.");
    }

    // Init Load Cells
    scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
    scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
    Serial.println("-> Load Cells initialized.");

    printMenu();
}

void runServoTest() {
    if (!testSetupDone) {
        Serial.println("\n\n--- SERVO WHEEL MODE TEST ---");
        servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0);
        delay(50);
        
        Serial.println("-> Homing motor to absolute Position 0...");
        servo.setTargetPosition(SERVO_ID, 0, 2000);
        delay(2000); // Wait for home

        Serial.println("\nWaiting for user input...");
        Serial.println("Type 'y' and press Enter to START the 5-revolution test.");
        testSetupDone = true;
        return;
    }

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'y' || c == 'Y') {
            Serial.println("\n[ STARTING SERVO TEST ]");

            servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 1); // Wheel Mode
            delay(50);

            int testVelocity = 3350;
            Serial.print("-> Commanded Velocity: "); Serial.print(testVelocity); Serial.println(" counts/sec");
            servo.setTargetVelocity(SERVO_ID, testVelocity);
            
            unsigned long startTime = millis();
            unsigned long printTimer = startTime;
            
            int previousPos = servo.getCurrentPosition(SERVO_ID);
            long totalAccumulatedCounts = 0;
            long targetCounts = 4096 * 5; 

            while (totalAccumulatedCounts < targetCounts) {
                int currentPos = servo.getCurrentPosition(SERVO_ID);
                
                if (currentPos < previousPos - 2000) { 
                    totalAccumulatedCounts += (4096 - previousPos) + currentPos;
                } else {
                    totalAccumulatedCounts += (currentPos - previousPos);
                }
                previousPos = currentPos;

                if (millis() - printTimer >= 100) {
                    printTimer = millis();
                    int speed = servo.getCurrentSpeed(SERVO_ID);
                    byte voltageRaw = servo.readRegister(SERVO_ID, STSRegisters::CURRENT_VOLTAGE);
                    float voltage = voltageRaw / 10.0;
                    
                    Serial.print("Time: "); Serial.print(millis() / 1000.0, 1);
                    Serial.print("s | Pos: "); Serial.print(currentPos);
                    Serial.print(" (Total: "); Serial.print(totalAccumulatedCounts);
                    Serial.print(") | Spd: "); Serial.print(speed);
                    Serial.print(" | Volts: "); Serial.print(voltage, 1);
                    Serial.println("V");
                }
            }
            
            unsigned long timeTaken = millis() - startTime;
            
            Serial.println("\n[ TARGET REACHED - STOPPING MOTOR ]");
            servo.setTargetVelocity(SERVO_ID, 0);
            servo.writeRegister(SERVO_ID, STSRegisters::OPERATION_MODE, 0);
            
            Serial.println("\n===========================================");
            Serial.println("          TEST RESULTS SUMMARY");
            Serial.println("===========================================");
            Serial.print("  Total Time Taken : "); Serial.print(timeTaken / 1000.0, 3); Serial.println(" seconds");
            Serial.print("  Distance Traveled: "); Serial.print(totalAccumulatedCounts); Serial.println(" counts");
            Serial.print("  Average Velocity : "); Serial.print( (float)targetCounts / (timeTaken / 1000.0) ); Serial.println(" counts/sec");
            Serial.println("===========================================\n");

            testSetupDone = false;
            currentState = MODE_MENU;
            printMenu();
        }
    }
}

void runLoadCellTest() {
    if (!testSetupDone) {
        Serial.println("\n\n--- LOAD CELL RAW VALUE TEST ---");
        Serial.println("This test will bypass calibration factors and read the raw analog output");
        Serial.println("of the HX711 chip to verify hardware connection is working.");
        Serial.println("It will read continuously for 10 seconds.");
        
        Serial.println("\nWaiting for user input...");
        Serial.println("Type 'y' and press Enter to START the Load Cell test.");
        testSetupDone = true;
        return;
    }

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'y' || c == 'Y') {
            Serial.println("\n[ STARTING LOAD CELL TEST ]");
            
            unsigned long startTime = millis();
            unsigned long printTimer = startTime;

            while (millis() - startTime < 10000) {
                if (millis() - printTimer >= 100) {
                    printTimer = millis();

                    long raw1 = 0;
                    long raw2 = 0;
                    bool ready1 = scale1.is_ready();
                    bool ready2 = scale2.is_ready();

                    if (ready1) raw1 = scale1.read();
                    if (ready2) raw2 = scale2.read();

                    Serial.print("Time: "); Serial.print((millis() - startTime) / 10000.0, 1);
                    
                    Serial.print("s | LoadCell 1 (Pins 2/3): "); 
                    if (ready1) { Serial.print(raw1); } else { Serial.print("NOT READY"); }
                    
                    Serial.print(" | LoadCell 2 (Pins 4/5): ");
                    if (ready2) { Serial.println(raw2); } else { Serial.println("NOT READY"); }
                }
            }

            Serial.println("\n[ TEST COMPLETE ]");
            Serial.println("If values were 0, -1, or NOT READY, check your wiring!");
            Serial.println("Otherwise, hardware is working.");

            delay(2000);
            testSetupDone = false;
            currentState = MODE_MENU;
            printMenu();
        }
    }
}

void loop() {
    switch (currentState) {
        case MODE_MENU:
            if (Serial.available()) {
                char choice = Serial.read();
                if (choice == '1') {
                    currentState = MODE_SERVO_TEST;
                    testSetupDone = false;
                } else if (choice == '2') {
                    currentState = MODE_LOADCELL_TEST;
                    testSetupDone = false;
                }
            }
            break;
            
        case MODE_SERVO_TEST:
            runServoTest();
            break;

        case MODE_LOADCELL_TEST:
            runLoadCellTest();
            break;
    }
}