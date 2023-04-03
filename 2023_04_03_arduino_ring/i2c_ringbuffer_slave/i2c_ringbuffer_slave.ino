#include <Wire.h>

// 자신의 슬레이브 주소를 설정해 줍니다.(슬레이브 주소에 맞게 수정해야 합니다.)
#define SLAVE 4  
#define BYTES 10

char ch[BYTES];
int data= 0;

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
  char Start;
    // 수신 버퍼 읽기
  Start = Wire.read();
  if(Start == 's'){
    for(int i = 0; i<BYTES; i++){
      ch[i] = Wire.read();
      if(ch[i] == 'e') break;
      //Serial.print(i);
      //Serial.print(" : ");
      Serial.print(ch[i]);
      Serial.print("  ");
    }
    Serial.println();
    //ch[data] = Wire.read();
    
//    if(ch[data] == 'e') {
//      data --;
//      init_buffer();
//    }
//    data++;
//    if(data > 8) {
//      data %= 8;
//      Serial.println(" ");
//    }
  }
  init_buffer();
}

void sendToMaster() {
  // 자신의 슬레이브 주소를 담은 메세지를 마스터에게 보냅니다. 슬레이브 주소에 맞게 수정해야 합니다.
  for (int i = 0 ; i < BYTES ; i++) {
    if(ch[i] != 0) {
      ch[i] = Wire.write("ch[i]");  
    }
  }
}

void init_buffer(void){
  for (int i = 0 ; i < BYTES ; i++) {
     ch[i] = '0';
  }
}
