/*
 * Standalone Load Cell Calibration Sketch
 *
 * Description: Calibrates two 10kg HX711 Load Cells step-by-step.
 * - Load Cell 1 (Cart Base): Accounts for a 1304g static deadweight.
 *   Tares with the deadweight, and configures the factor so UPWARD pulls are positive.
 * - Load Cell 2 (String Tension): Standard calibration.
 *
 * Hardware:
 * - Teensy 4.1
 * - Load Cell 1: DT (Pin 2), SCK (Pin 3)
 * - Load Cell 2: DT (Pin 4), SCK (Pin 5)
 */

#include <Arduino.h>
#include <HX711.h>

// Teensy 4.1 Load Cell Pins
const int LOADCELL1_DOUT_PIN = 2;
const int LOADCELL1_SCK_PIN = 3;
const int LOADCELL2_DOUT_PIN = 4;
const int LOADCELL2_SCK_PIN = 5;

HX711 scale1;
HX711 scale2;

float finalCalFactor1 = 1.0;
float finalCalFactor2 = 1.0;

enum CalibrationState {
    STATE_INIT,
    STATE_LC1_TARE,
    STATE_LC1_CALIBRATE,
    STATE_LC2_TARE,
    STATE_LC2_CALIBRATE,
    STATE_DONE
};

CalibrationState currentState = STATE_INIT;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("\n===========================================");
    Serial.println("  LOAD CELL CALIBRATION WIZARD");
    Serial.println("===========================================");
    
    scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
    scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
    
    scale1.set_scale();
    scale2.set_scale();
}

void loop() {
    switch (currentState) {
        case STATE_INIT:
            Serial.println("\n[ STEP 1: LOAD CELL 1 (Cart Base) TARE ]");
            Serial.println("-> This load cell sits under the cart.");
            Serial.println("-> Ensure the cart (1304g static deadweight) is resting completely on Load Cell 1.");
            Serial.println("-> Do NOT pull the string yet.");
            Serial.println("-> Send 't' to tare (zero) Load Cell 1 with the cart's weight.");
            currentState = STATE_LC1_TARE;
            break;

        case STATE_LC1_TARE:
            if (Serial.available()) {
                char c = Serial.read();
                if (c == 't' || c == 'T') {
                    Serial.println("Taring Load Cell 1... Please wait.");
                    scale1.tare(20); // Average 20 readings for a solid tare
                    Serial.println("Load Cell 1 Tared successfully! (Cart deadweight is now 0).");
                    
                    Serial.println("\n[ STEP 2: LOAD CELL 1 CALIBRATION ]");
                    Serial.println("-> Place a KNOWN weight (e.g., 500g) ON TOP of the resting cart.");
                    Serial.println("-> Note: Because we want an UPWARD pull to read as positive, the code will automatically invert this downward weight calibration.");
                    Serial.println("-> Type the weight you placed in grams (e.g., '500') and press Enter.");
                    currentState = STATE_LC1_CALIBRATE;
                }
            }
            break;

        case STATE_LC1_CALIBRATE:
            if (Serial.available()) {
                String input = Serial.readStringUntil('\n');
                input.trim();
                if (input.length() > 0) {
                    float knownWeight = input.toFloat();
                    if (knownWeight > 0) {
                        Serial.print("Calculating Load Cell 1 for weight: "); Serial.print(knownWeight); Serial.println("g");
                        
                        long reading = scale1.get_value(20); // Average 20 readings
                        
                        // We divide the raw reading by the known weight.
                        // Because putting weight ON the cart is a DOWNWARD force, but we want UPWARD pulls 
                        // from the string to be positive, we INVERT the calibration factor by making it negative!
                        finalCalFactor1 = -(reading / knownWeight);
                        
                        Serial.print(">>> Load Cell 1 Calibration Factor: ");
                        Serial.println(finalCalFactor1);
                        
                        currentState = STATE_LC2_TARE;
                    }
                }
            }
            break;

        case STATE_LC2_TARE:
            Serial.println("\n[ STEP 3: LOAD CELL 2 (String Tension) TARE ]");
            Serial.println("-> This load cell measures direct string pull.");
            Serial.println("-> Ensure there is absolutely NO tension on the string (resting naturally).");
            Serial.println("-> Send 't' to tare (zero) Load Cell 2.");
            currentState = STATE_LC2_TARE + 1; // move to wait state
            break;

        case STATE_LC2_TARE + 1:
            if (Serial.available()) {
                char c = Serial.read();
                if (c == 't' || c == 'T') {
                    Serial.println("Taring Load Cell 2... Please wait.");
                    scale2.tare(20);
                    Serial.println("Load Cell 2 Tared successfully!");
                    
                    Serial.println("\n[ STEP 4: LOAD CELL 2 CALIBRATION ]");
                    Serial.println("-> Apply a KNOWN pulling force (e.g., hang a 500g weight from the string).");
                    Serial.println("-> Type the weight in grams (e.g., '500') and press Enter.");
                    currentState = STATE_LC2_CALIBRATE;
                }
            }
            break;

        case STATE_LC2_CALIBRATE:
            if (Serial.available()) {
                String input = Serial.readStringUntil('\n');
                input.trim();
                if (input.length() > 0) {
                    float knownWeight = input.toFloat();
                    if (knownWeight > 0) {
                        Serial.print("Calculating Load Cell 2 for weight: "); Serial.print(knownWeight); Serial.println("g");
                        
                        long reading = scale2.get_value(20);
                        
                        // Standard calibration for direct tension
                        finalCalFactor2 = reading / knownWeight;
                        
                        Serial.print(">>> Load Cell 2 Calibration Factor: ");
                        Serial.println(finalCalFactor2);
                        
                        currentState = STATE_DONE;
                    }
                }
            }
            break;

        case STATE_DONE:
            Serial.println("\n===========================================");
            Serial.println("        CALIBRATION COMPLETE!");
            Serial.println("===========================================");
            Serial.println("Copy these exact values into your main project code (src/main.cpp):");
            Serial.print("float CAL_FACTOR_1 = "); Serial.print(finalCalFactor1); Serial.println(";");
            Serial.print("float CAL_FACTOR_2 = "); Serial.print(finalCalFactor2); Serial.println(";");
            Serial.println("===========================================");
            
            // Halt execution
            while(true) { delay(1000); }
            break;
    }
}