#include <Wire.h>
#include <avr/wdt.h>

// 마스터 주소
#define MASTER_ADDRESS 0x04

// 마스터로부터 수신한 신호
#define MASTER_TO_SLAVE_SIGNAL 0xAA
#define COMMUNICATION_INTERVAL 1000
unsigned long lastCommunicationTime = 0;

unsigned long ToggleDelay;


void setup() {
  Wire.begin(MASTER_ADDRESS); // I2C 통신 초기화, 마스터 주소 설정
  Wire.onReceive(receiveEvent); // 수신 이벤트 핸들러 등록
  Serial.begin(9600); // 시리얼 통신 초기화
  Serial.println("Start");
  wdt_disable();
  ToggleDelay = 500;
  wdt_enable(WDTO_500MS);
}

void loop() {
  unsigned long currentTime = millis();
  
  // 마지막 통신 시간에서 COMMUNICATION_INTERVAL 만큼 경과했다면
  if (currentTime - lastCommunicationTime >= COMMUNICATION_INTERVAL) {
    lastCommunicationTime = currentTime;
    
    // 마스터에게 수신 완료를 알림
    Wire.beginTransmission(MASTER_ADDRESS);
    Wire.endTransmission();
    
    Serial.println("Received signal from master");
  }
}

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    byte receivedByte = Wire.read();
    
    // 마스터로부터 신호를 수신한 경우, WATCH DOG를 리셋
    if (receivedByte == MASTER_TO_SLAVE_SIGNAL) {
      lastCommunicationTime = millis();
      
      Serial.println("Watchdog reset");
    }
  }
}
