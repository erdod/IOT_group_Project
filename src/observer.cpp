#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

const int I2C_SDA = 41;
const int I2C_SCL = 42;
const int BUZZER_PIN = 7;
Adafruit_INA219 ina219;

uint8_t targetAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

float dynamic_threshold = 0;
bool is_calibrated = false;
bool system_locked = false;
String current_mode = "IDLE";

float samples[50];
int sample_idx = 0;
int anomaly_confirm = 0;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char msg[len + 1]; memcpy(msg, incomingData, len); msg[len] = '\0';
  String data = String(msg);

  if (data.startsWith("LOG:")) {
    Serial.println(data.substring(4));
  } 
  else if (data == "CMD:START_LEARN") { 
    current_mode = "LEARNING"; 
    sample_idx = 0; 
  } 
  else if (data == "CMD:START_MONITOR") { 
    current_mode = "MONITORING"; 
    anomaly_confirm = 0; 
  } 
  else if (data == "CMD:STOP_MEASURE") { 
    if (current_mode == "LEARNING" && sample_idx > 5) {
      float sum = 0; for(int i=0; i<sample_idx; i++) sum += samples[i];
      float mean = sum / sample_idx;
      
      float sq_sum = 0; for(int i=0; i<sample_idx; i++) sq_sum += pow(samples[i] - mean, 2);
      float stdev = sqrt(sq_sum / sample_idx);
      
      dynamic_threshold = mean + (3.0 * stdev);
      
      if (dynamic_threshold < mean + 20.0) dynamic_threshold = mean + 20.0; 
      
      is_calibrated = true;
      Serial.println("\n[EDGE ] Statistical Calibration (3-Sigma) Completed!");
      Serial.printf("   Samples analyzed: %d\n", sample_idx);
      Serial.printf("   Mean (\u03BC): %.2f mA | Std Dev (\u03C3): %.2f mA\n", mean, stdev);
      Serial.printf("   >>> Dynamic Stall Threshold: %.2f mA <<<\n", dynamic_threshold);
    }
    current_mode = "IDLE"; 
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  ina219.begin();
  
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, targetAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  
  Serial.println("\n=============================================");
  Serial.println("  FLOAT IoT: STATISTICAL EDGE MONITORING");
  Serial.println("=============================================");
}

void loop() {
  if (system_locked) {
    Serial.printf("[HALT INFO] System safely locked. Current: %.2f mA\n", max(0.0f, ina219.getCurrent_mA()));
    delay(5000);
    return;
  }

  float current = ina219.getCurrent_mA();
  if (current < 0) current = 0;

  if (current_mode == "LEARNING") {
    if (current > 150.0 && sample_idx < 50) {
      samples[sample_idx++] = current;
      Serial.printf("   [LEARNING] Sample %d: %.2f mA\n", sample_idx, current);
    }
    delay(300);
  } 
  else if (current_mode == "MONITORING" && is_calibrated) {
    Serial.printf("   [EDGE] Load: %.2f mA (Threshold: %.2f) [%d/3]\n", current, dynamic_threshold, anomaly_confirm);
    
    if (current > dynamic_threshold) {
      anomaly_confirm++;
      if (anomaly_confirm >= 3) {
        Serial.println("\n[!!!] ANOMALY DETECTED: MECHANICAL STALL CONFIRMED [!!!]");
        esp_now_send(targetAddress, (uint8_t *)"HALT", 4);
        
        for(int i=0; i<3; i++) { digitalWrite(BUZZER_PIN, HIGH); delay(150); digitalWrite(BUZZER_PIN, LOW); delay(100); }
        system_locked = true;
      }
    } else { 
      anomaly_confirm = 0;
    }
    delay(400);
  }
  else {
    Serial.printf("[POWER INFO] Current draw: %.2f mA\n", current);
    delay(2000);
  }
}