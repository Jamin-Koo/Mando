#include <Wire.h>
#include <avr/wdt.h>

int SLAVE = 4;
int WCD_Flag = 0;
  
//const uint8_t button_PIN = 2;
const uint8_t LED_PIN = 3;
const uint8_t LED_PIN_WDTOUT = 4;
//int startFlag = 0;
int resetFlag = 0;

void setup() {
  Wire.begin(SLAVE);
  Wire.onReceive(receiveFromMaster);
  // 마스터의 데이터 전송 요구가 있을 때 처리할 함수 등록
  Wire.onRequest(sendToMaster);
  
//  pinMode(button_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_WDTOUT, OUTPUT);
  digitalWrite(LED_PIN_WDTOUT, HIGH);
  wdt_enable(WDTO_4S);
  Serial.begin(115200);
  Serial.println("START");
  resetFlag = 0;
}

void loop() { 
}

//master에서 값 받기
void receiveFromMaster() {
  digitalWrite(LED_PIN, LOW);
  WCD_Flag = Wire.read();
  if (WCD_Flag == 1){
    wdt_reset();
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN_WDTOUT, LOW);
    Serial.println("data_Received");
  }
  else{
    Serial.println("data_Not Received");
  }
}  

void sendToMaster() {
  if (resetFlag == 0){
    Wire.write("r");
    resetFlag = 1;
  }
//  else{
//    Wire.write("0");
//  }
}
