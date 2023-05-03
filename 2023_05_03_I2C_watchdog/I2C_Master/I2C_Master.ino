#include <Wire.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>

#define MsTimer_check 2

int flag = 0;
// 슬레이브 주소
int SLAVE = 4;
int RX_buffer = 0;

// 슬레이브에서 받는 데이터
int I2C_Receive_buffer[8] = {0,};        // SLAVE에서 받는 버퍼

uint8_t TX_buffer[9] = {0,};    // pc로 송신할 버퍼


unsigned int c1;    // Button 값
union
{
  uint16_t data;
  uint8_t a[2];
}ADC_val;           //ADC_value(1),(2)

union
{
  uint16_t data;
  uint8_t a[2];
}send_ADC_val;      //send_ADC_value(1),(2)

union
{
    uint16_t check_sum;
    uint8_t b[2];
} crc_16_val_S;


SoftwareSerial mySerial(0, 1); // RX,TX

void setup() {
  // Wire 라이브러리 초기화
  Wire.begin(); 
  // 직렬 통신 초기화
  Serial.begin(115200);
  mySerial.begin(115200);
  
  MsTimer2::set(500, RX_Interrupt); // 500ms period
  MsTimer2::start();

  pinMode(MsTimer_check, OUTPUT);

//  MsTimer2::set(500, RX_Interrupt); // 500ms period
//  MsTimer2::start();
}

void init_Receivebuff()
{
  for(int i=0; i<8; i++)
  {
    I2C_Receive_buffer[i] = 0;
  } 
}

void init_Sendbuff()
{
  for(int i=0; i<9; i++)
  {
    TX_buffer[i] = 0;
  } 
}

struct DATA{
  unsigned char input_dataArr;
};

void loop() {
  if(flag == 1)
  {
    i2c_communication();      //SLAVE 에서 받기
      
    //slave로 RX_buffer 값 전송
    Wire.beginTransmission(SLAVE);
    Wire.write(RX_buffer);
    Wire.endTransmission(SLAVE);
    flag = 0;
  }
}

void i2c_communication() {
  
  // 10 바이트 크기의 데이터 요청
  Wire.requestFrom(SLAVE, 5); 
  
    // 수신 데이터 읽기
    int c = Wire.read();
    if (c == 's'){
      // 10 바이트 모두 출력할 때까지 반복
      for (int j = 0 ; j < 4 ; j++) {
        int d = Wire.read();
        if(d == 'e') break;
        
        I2C_Receive_buffer[j] = d ;
        
         //수신 데이터 출력
//        Serial.print(j);
//        Serial.print(" : ");
//        Serial.println(d);
      }
   }
   
   if(I2C_Receive_buffer[1]!= 0 ||I2C_Receive_buffer[2] != 0){
   ADC_val.a[0] = I2C_Receive_buffer[1];
   ADC_val.a[1] = I2C_Receive_buffer[2];
   }
}


// CRC16-Modbus checksum 계산 함수
int CRC16_MODBUS(const uint8_t *nData, uint16_t wLength)
{

    static const uint8_t wCRCTable[] = { 0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780,
        0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1,
        0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801,
        0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40,
        0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680,
        0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0,
        0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501,
        0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981,
        0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1,
        0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200,
        0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141,
        0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480,
        0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0,
        0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01,
        0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381,
        0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0,
        0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01,
        0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40,
        0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81,
        0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1,
        0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341, 0X4100,
        0X81C1, 0X8081, 0X4040 };   //체크섬 테이블

    uint16_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (wLength--)
    {
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }       //CRC checksum xor 계산

    return wCRCWord;    //checksum 반환
}



void TX_Interrupt(int data, int adc)
{
    digitalWrite(MsTimer_check, HIGH);
    flag = 1;
    unsigned char mychar;

    TX_buffer[0] = 174;    // Aurix 주소
    TX_buffer[1] = 184;    // PC 주소
    TX_buffer[2] = 'i';    // 프로토콜의 종류를 나타내는 값
    TX_buffer[3] = 3;      // 프로토콜의 데이터 번호를 나타내는 값
    TX_buffer[4] = I2C_Receive_buffer[0];    // 데이터 값
    TX_buffer[5] = ADC_val.a[0]; // ADC 측정값의 하위 바이트
    TX_buffer[6] = ADC_val.a[1]; // ADC 측정값의 상위 바이트

    crc_16_val_S.check_sum = CRC16_MODBUS(TX_buffer, 7); // CRC16 체크섬 계산

    TX_buffer[7] = crc_16_val_S.b[0]; // CRC16 체크섬의 하위 바이트
    TX_buffer[8] = crc_16_val_S.b[1]; // CRC16 체크섬의 상위 바이트

    
    for(int i = 0; i < 9; i++){
      mychar = TX_buffer[i];
      mySerial.write(mychar);  // 시리얼 포트(UART)로 데이터 전송
      //  데이터 확인
      Serial.print("TX_Interrupt : : :  ");
      Serial.print(i);
      Serial.print("  ");
      Serial.println(TX_buffer[i]);
    }
    
    Serial.println();
    //버퍼 초기화
    init_Sendbuff();
    
    
    digitalWrite(MsTimer_check, LOW);
}

void RX_Interrupt(){
  digitalWrite(MsTimer_check, HIGH);
  flag = 1;
      //mySerial이 사용 가능할 때 if문 진입
      if(mySerial.available()>0){
        //mySerial 버퍼 초기화
        mySerial.flush();
        //mySerial에서 받아온 값 RX_buffer에 저장
        RX_buffer = mySerial.read();

        //값 확인용 
        Serial.println(RX_buffer);
    }
    digitalWrite(MsTimer_check, LOW);
}
