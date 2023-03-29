#include <Wire.h>
#include <MsTimer2.h>

// 슬레이브 주소
int SLAVE = 4;

// 슬레이브에서 받는 데이터
int receiveData[8] = {0,};
int read_data;

union
{
  uint16_t data;
  uint8_t a[2];
}ADC_val;

void setup() {
  // Wire 라이브러리 초기화
  Wire.begin(); 
  // 직렬 통신 초기화
  Serial.begin(115200); 

//  MsTimer2::set(500, i2c_communication); // 500ms period
//  MsTimer2::start();
  
}

void init_sendbuff()
{
  for(int i=0; i<8; i++)
  {
    receiveData[i] = 0;
  } 
}

void loop() {
      i2c_communication();
}

void i2c_communication() {
  init_sendbuff();
  unsigned int c1;
  
  // 10 바이트 크기의 데이터 요청
  Wire.requestFrom(SLAVE, 10); 
  
    // 수신 데이터 읽기
    int c = Wire.read();
    if (c == 's'){
      // 10 바이트 모두 출력할 때까지 반복
      for (int j = 0 ; j < 6 ; j++) {
        int d = Wire.read();
        if(d == 'e') break;
        
        receiveData[j] = d ;
        
        // 수신 데이터 출력
        Serial.print(j);
        Serial.print(" : ");
        Serial.println(receiveData[j]);

      }
    }
  
   ADC_val.a[0] = receiveData[1];
   ADC_val.a[1] = receiveData[2];
   c1 = ADC_val.data;
   Serial.println(c1);
   Serial.println();
}
