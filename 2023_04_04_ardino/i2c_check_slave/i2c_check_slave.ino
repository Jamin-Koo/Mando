#include <Wire.h>

// 자신의 슬레이브 주소를 설정해 줍니다.(슬레이브 주소에 맞게 수정해야 합니다.)
#define SLAVE 4 
#define BYTES 4

unsigned int data[BYTES] = {0, };
  
void setup() {
  // Wire 라이브러리 초기화
  // 슬레이브로 참여하기 위해서는 주소를 지정해야 한다.
  Wire.begin(SLAVE);
  Wire.onReceive(receiveFromMaster);
  // 마스터의 데이터 전송 요구가 있을 때 처리할 함수 등록
  Wire.onRequest(sendToMaster);
  Serial.begin(38400);
}

void loop () {
}

void receiveFromMaster() {
    // 수신 버퍼 읽기
  Serial.print("Receive : ");
  for(int i = 0; i<BYTES; i++){
    data[i] = Wire.read();
    Serial.print(data[i]);
    Serial.print("  ");
  }
  Serial.println("");
}  

void sendToMaster() {
  Serial.print("Send :    ");
  for(int i = 0; i<BYTES; i++){
    Wire.write(data[i]);
    Serial.print(data[i]);
    Serial.print("  ");
  }
  Serial.println("");
}
