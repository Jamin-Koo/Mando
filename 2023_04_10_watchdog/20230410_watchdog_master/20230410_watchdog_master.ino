#include <Wire.h>
#include <MsTimer2.h>
#include <avr/wdt.h>

/*** Btn Pin Setting ***/
#define Btn_PIN  8

/*** Slave Adress Setting ***/
int SLAVE_ADDR = 4;

int flag = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(Btn_PIN, INPUT);
  // Watchdog 시간 4초로 지정
  wdt_enable(WDTO_4S);
  // 200ms 주기로 TX_Interrupt 발생
  MsTimer2::set(200, TX_Interrupt);
  MsTimer2::start();
}

void i2c_communication() {
  
  // 10 바이트 크기의 데이터 요청
  Wire.requestFrom(SLAVE_ADDR, 1); 
  
    // 수신 데이터 읽기
    int SLAVE_TO_MASTER = Wire.read();
    if (SLAVE_TO_MASTER == 'r')
    { 
        //수신 데이터 출력
        Serial.println("Slave_Reset");
    }
}

void TX_Interrupt()
{
  flag = 1;
}


void loop() 
{
  // TX_Interrupt 수행 시 발동
  if(flag == 1)
  {
    i2c_communication();
  
    int btn = digitalRead(Btn_PIN);
    
    // SLAVE로 전송
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(btn);
    Serial.print("Send: ");
    Serial.println(btn);
    Wire.endTransmission(SLAVE_ADDR);
    flag = 0;
  }
}
