/**
 * PROJECT: FLOAT (Federated Learning for Observation of Aquatic Tanks)
 * MODULE: Edge-AI Anomaly Detection & Predictive Maintenance
 * HARDWARE: ESP32-S3, INA219 (I2C), DC Pump, Micro Servo, Active Buzzer
 * * DESCRIPTION:
 * This system implements a Statistical Anomaly Detection model to monitor
 * the health of an aquarium pump. It uses a dynamic 3-Sigma threshold 
 * calculated during a learning phase and validates faults through 
 * temporal debouncing (3 consecutive samples) to eliminate false positives.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <math.h> 

// --- PIN CONFIGURATION ---
const int PUMP_PIN = 47;      
const int SERVO_PIN = 4;      
const int BUZZER_PIN = 7;     
const int I2C_SDA = 5;       
const int I2C_SCL = 6;       

Adafruit_INA219 ina219;

// --- MONITORING PARAMETERS ---
const int PUMP_DURATION_MS = 10000;  // Standard filtration cycle (10s)
const int BLIND_WINDOW_MS = 1000;    // Wait for inrush current to stabilize
const float SIGMA_MULTIPLIER = 3.0;  // 3-Sigma rule (covers 99.7% of normal variance)
const int MAX_CONFIRMATIONS = 3;     // Samples required to confirm a persistent fault

// --- EDGE-AI VARIABLES ---
bool is_calibrated = false;
float mean_mA = 0;           
float std_dev_mA = 0;        
float dynamic_threshold = 0; 
bool system_locked = false; 

// Statistical buffers
float samples[20];
int sample_idx = 0;
int anomaly_confirm_count = 0;

// Simulation variables
float simulated_turbidity = 80.0;

// ==========================================
// UTILITY FUNCTIONS
// ==========================================

/**
 * Triggers the buzzer with a specific pattern.
 * Logic: Used only when the pump is OFF to prevent I2C brownouts.
 */
void triggerBuzzer(int beepCount) {
  for(int i=0; i < beepCount; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// Funzione manuale per il servo (Anti-Crash)
void muoviServo(int gradi) {
  pinMode(SERVO_PIN, OUTPUT); // <-- Aggiungi questa riga come scudo!
  int microsecondi_alto = map(gradi, 0, 180, 500, 2400);
  for(int i = 0; i < 50; i++) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(microsecondi_alto);
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(20000 - microsecondi_alto);
  }
}

/**
 * Executes the Automated Feeder behavior.
 * This action is blocked if the system detects a pump failure (Interlock).
 */
void triggerFeeder() {
  Serial.println("   [SERVO] Action: Opening feeder (90°)...");
  muoviServo(90); // INVIA IL SEGNALE REALE
  delay(1000); 
  
  Serial.println("   [SERVO] Action: Closing feeder (0°)...");
  muoviServo(0);  // INVIA IL SEGNALE REALE
  
  // Mettiamo il pin a LOW per spegnere il motore a riposo e non farlo ronzare
  digitalWrite(SERVO_PIN, LOW); 
}

/**
 * Updates the simulated water quality.
 * If the pump fails, water remains dirty, triggering the logic alert.
 */
void updateTurbidity(bool success) {
  if (success) {
    simulated_turbidity -= random(30, 50);
    if (simulated_turbidity < 10) simulated_turbidity = 10;
  } else {
    simulated_turbidity += random(5, 15);
    if (simulated_turbidity > 99) simulated_turbidity = 99;
  }
}

// ==========================================
// CORE SYSTEM SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(SERVO_PIN, LOW);

  Serial.println("\n=============================================");
  Serial.println("    FLOAT IoT: STATISTICAL EDGE MONITORING   ");
  Serial.println("=============================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!ina219.begin()) {
    Serial.println("[CRITICAL] INA219 Sensor not found! System Halted.");
    while (1);
  }

  // Calibrate INA219 for high precision (Max 400mA range)
  ina219.setCalibration_16V_400mA();
  
  // Seed random generator using a floating ADC pin
  randomSeed(analogRead(10));
  
  Serial.println("[OK] System initialized. Ready for Learning Phase.\n");
}

// ==========================================
// MAIN OPERATIONAL LOOP
// ==========================================
void loop() {
  Serial.println("---------------------------------------------");
  
  if (system_locked) {
    Serial.println("[!] SYSTEM HALTED: Hardware Anomaly Confirmed.");
    Serial.println("    Ecosystem protection active. Feeder blocked.");
    triggerBuzzer(1); 
    delay(5000);
    return;
  }

  Serial.printf("[SENSOR] Real-time Turbidity: %.1f%%\n", simulated_turbidity);

  if (simulated_turbidity >= 50.0) {
    // --- SCENARIO A: FILTRATION CYCLE ---
    Serial.println(">>> Action: High Turbidity detected. Starting Pump...");
    digitalWrite(PUMP_PIN, HIGH);
    
    unsigned long startTime = millis();
    bool cycle_fault = false;
    anomaly_confirm_count = 0; // Reset confirmation counter for new cycle

    delay(BLIND_WINDOW_MS); // Skip startup inrush current

    while (millis() - startTime < PUMP_DURATION_MS) {
      float current = ina219.getCurrent_mA();

      // I2C Safety: Detect potential bus crashes or power issues
      if (current > 800.0) {
        digitalWrite(PUMP_PIN, LOW);
        Serial.println("[CRITICAL] I2C Data Corruption / Brownout detected.");
        system_locked = true; cycle_fault = true; break;
      }

      if (!is_calibrated) {
        // --- PHASE 1: STATISTICAL LEARNING ---
        if (sample_idx < 20) {
          samples[sample_idx++] = current;
          Serial.printf("   [LEARNING] Sample %d: %.2f mA\n", sample_idx, current);
        }
      } 
      else {
        // --- PHASE 2: PREDICTIVE MAINTENANCE (TEMPORAL DEBOUNCING) ---
        Serial.printf("   [EDGE AI] Load: %.2f mA (Limit: %.2f) [%d/%d]\n", 
                      current, dynamic_threshold, anomaly_confirm_count, MAX_CONFIRMATIONS);

        if (current > dynamic_threshold) {
          anomaly_confirm_count++;
          if (anomaly_confirm_count >= MAX_CONFIRMATIONS) {
            // FAULT CONFIRMED
            digitalWrite(PUMP_PIN, LOW);
            system_locked = true;
            cycle_fault = true;
            Serial.println("\n[!!!] ANOMALY DETECTED: MECHANICAL STALL CONFIRMED [!!!]");
            Serial.printf("Current exceeded 3-Sigma limit (%.2f mA) for 1500ms.\n", dynamic_threshold);
            triggerBuzzer(3); // Alarm sounds ONLY after pump shutdown
            break;
          }
        } else {
          // Reset counter if it was a transient peak (noise/bubbles)
          anomaly_confirm_count = 0; 
        }
      }
      delay(500); // 2Hz Sampling Rate
    }

    digitalWrite(PUMP_PIN, LOW);

    // Finalize calibration after the first successful cycle
    if (!is_calibrated && !cycle_fault && sample_idx >= 15) {
      float sum = 0;
      for(int i=0; i < sample_idx; i++) sum += samples[i];
      mean_mA = sum / sample_idx;

      float sq_sum = 0;
      for(int i=0; i < sample_idx; i++) sq_sum += pow(samples[i] - mean_mA, 2);
      std_dev_mA = sqrt(sq_sum / sample_idx);

      // Dynamic Thresholding via 3-Sigma Rule
      dynamic_threshold = mean_mA + (SIGMA_MULTIPLIER * std_dev_mA);
      
      // Absolute minimum safety margin (20mA)
      if (dynamic_threshold < mean_mA + 20.0) dynamic_threshold = mean_mA + 20.0;

      is_calibrated = true;
      Serial.println("\n[EDGE AI] Dynamic Calibration Complete!");
      Serial.printf("   Mean (u): %.2f mA | StdDev (s): %.2f mA\n", mean_mA, std_dev_mA);
      Serial.printf("   >>> Dynamic Stall Threshold (3-s): %.2f mA <<<\n", dynamic_threshold);
    }

    updateTurbidity(!cycle_fault);
    if (!cycle_fault) Serial.println(">>> Filtration cycle finished successfully.");
    
  } 
  else {
    // --- SCENARIO B: BIOLOGICAL CYCLE ---
    Serial.println(">>> Action: Water is clean. Starting Feeding Cycle...");
    triggerFeeder();
    Serial.println(">>> Feeding finished.");
    simulated_turbidity = 80.0; // Force dirty for next test cycle
  }

  Serial.println("\nWaiting for next monitoring window...\n");
  delay(5000);
}