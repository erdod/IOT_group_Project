#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h"  

const int PUMP_PIN = 47;
const int SERVO_PIN = 6;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool system_halted = false;

uint8_t observerAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
volatile bool emergency_stop = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char msg[len + 1]; memcpy(msg, incomingData, len); msg[len] = '\0';
  if (String(msg) == "HALT") { 
    digitalWrite(PUMP_PIN, LOW); 
    emergency_stop = true; 
    system_halted = true; 
  }
}

void sendMsg(String type, String content) {
  String fullMsg = type + ":" + content;
  esp_now_send(observerAddress, (uint8_t *)fullMsg.c_str(), fullMsg.length());
  delay(80); 
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(SERVO_PIN, LOW);

  if (system_halted) {
    esp_sleep_enable_timer_wakeup(10 * 1000000ULL);
    esp_deep_sleep_start();
  }

  int turbidity = random(0, 101);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_POWER_2dBm);
  
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, observerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  delay(2000);

  sendMsg("LOG", "---------------------------------------------");
  sendMsg("LOG", "[SENSOR] Real-time Turbidity: " + String(turbidity) + "%");

  if (bootCount == 0) {
    sendMsg("LOG", ">>> Action: First Boot. Starting Dynamic Learning...");
    sendMsg("CMD", "START_LEARN");
    delay(500);
    
    digitalWrite(PUMP_PIN, HIGH);
    unsigned long start = millis();
    while (millis() - start < 10000 && !emergency_stop) { delay(10); }
    digitalWrite(PUMP_PIN, LOW);
    
    sendMsg("CMD", "STOP_MEASURE");
  } 
  else if (turbidity > 50) {
    sendMsg("LOG", ">>> Action: High Turbidity. Starting Pump (10s)...");
    sendMsg("CMD", "START_MONITOR");
    delay(500);

    digitalWrite(PUMP_PIN, HIGH);
    unsigned long start = millis();
    while (millis() - start < 10000 && !emergency_stop) { delay(10); }
    digitalWrite(PUMP_PIN, LOW);

    sendMsg("CMD", "STOP_MEASURE");
  } 
  else {
    sendMsg("LOG", ">>> Action: Water Clean. Starting Feeding...");
    delay(500);
    
    sendMsg("LOG", "   [SERVO] Opening 90°...");
    for(int i=0; i<35; i++) { digitalWrite(SERVO_PIN, HIGH); delayMicroseconds(1500); digitalWrite(SERVO_PIN, LOW); delay(18); }
    delay(1000);
    
    sendMsg("LOG", "   [SERVO] Closing 0°...");
    for(int i=0; i<35; i++) { digitalWrite(SERVO_PIN, HIGH); delayMicroseconds(1000); digitalWrite(SERVO_PIN, LOW); delay(19); }
  }

  bootCount++;
  sendMsg("LOG", "[SLEEP] Entering Deep Sleep (10s)...");
  delay(200);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_timer_wakeup(10 * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() {}