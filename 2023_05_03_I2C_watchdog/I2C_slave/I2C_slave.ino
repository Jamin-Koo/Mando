#include <Wire.h>

/***  Slave Adress Setting  ***/
#define SLAVE 4

/***  Btn Pin Setting  ***/
#define BtnPIN1 2
#define BtnPIN2 3
#define BtnPIN3 4
#define BtnPIN4 5
#define BtnPIN5 6
#define BtnPIN6 7
#define BtnPIN7 8
#define BtnPIN8 9


/***  LED Pin Setting  ***/
#define LED1 38
#define LED2 39
#define LED3 40
#define LED4 41
#define LED5 42
#define LED6 43
#define LED7 44
#define LED8 45


/***  ADC Pin Setting  ***/
#define ADCPIN A0

union
{
  uint16_t data;
  uint8_t a[2];
}ADC_val;

// 카운터
byte count = 0; 

int INPUT_buff[9] = {0,};
int send_btnData, adc_data, send_adcData;

void setup() 
{
  // Wire 라이브러리 초기화
  // 슬레이브 참여 위해 주소 지정
  Wire.begin(SLAVE);
  Wire.onReceive(receiveFromMaster);
  // 마스터의 데이터 전송 요구가 있을 때 처리할 함수 등록
  Wire.onRequest(sendToMaster);
  // 누르면 0 안누르면 1
  pinMode(BtnPIN1, INPUT_PULLUP);
  pinMode(BtnPIN2, INPUT_PULLUP);
  pinMode(BtnPIN3, INPUT_PULLUP);
  pinMode(BtnPIN4, INPUT_PULLUP);
  pinMode(BtnPIN5, INPUT_PULLUP);
  pinMode(BtnPIN6, INPUT_PULLUP);
  pinMode(BtnPIN7, INPUT_PULLUP);
  pinMode(BtnPIN8, INPUT_PULLUP);

  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
  
  Serial.begin(115200);
  
  wdt_enable(WDTO_1S);
  //watchdog_ 1초
  Serial.println("Watchdog_start");
  //watchdog 실행
  
}

void init_INPUT_data(void)
{
  for(int i=0; i<8; i++)
  {
    INPUT_buff[i] = 0;
  }
}

void OUTPUT_control(int data)
{
    // 입력된 데이터를 8비트 이진수로 변환하여 저장하는 배열입니다.
    int OUTPUT_buff[8];

    // 7부터 0까지 8번 반복하는 반복문입니다.
    for(int i = 7; i>= 0 ; --i)
    {
        // i만큼 데이터를 오른쪽으로 시프트하고, 맨 오른쪽 비트와 AND 연산을 수행하여 값을 저장합니다.
        OUTPUT_buff[i] = (data >> i) & 1;
    }

    // 각 비트에 대한 값을 LED로 출력합니다. OUTPUT_buff 배열에서 비트 값을 가져와서, 1인 경우 해당 LED를 켜고, 0인 경우 해당 LED를 끄도록 합니다.
    (OUTPUT_buff[0] == 1) ?  digitalWrite(LED1,HIGH) : digitalWrite(LED1,LOW);
    (OUTPUT_buff[1] == 1) ?  digitalWrite(LED2,HIGH) : digitalWrite(LED2,LOW);
    (OUTPUT_buff[2] == 1) ?  digitalWrite(LED3,HIGH) : digitalWrite(LED3,LOW);
    (OUTPUT_buff[3] == 1) ?  digitalWrite(LED4,HIGH) : digitalWrite(LED4,LOW);
    (OUTPUT_buff[4] == 1) ?  digitalWrite(LED5,HIGH) : digitalWrite(LED5,LOW);
    (OUTPUT_buff[5] == 1) ?  digitalWrite(LED6,HIGH) : digitalWrite(LED6,LOW);
    (OUTPUT_buff[6] == 1) ?  digitalWrite(LED7,HIGH) : digitalWrite(LED7,LOW);
    (OUTPUT_buff[7] == 1) ?  digitalWrite(LED8,HIGH) : digitalWrite(LED8,LOW);

}

void receiveFromMaster() {
  int ch2;
  ch2 = Wire.read();
  OUTPUT_control(ch2);
  Serial.println(ch2);
  
  wdt_reset();
  //마스터에서 값을 받았을 때 watchdog 초기화

}


int pushed_button(void)
{
  int button_data = 0;
  
  int btn1 = digitalRead(BtnPIN1);
  int btn2 = digitalRead(BtnPIN2);
  int btn3 = digitalRead(BtnPIN3);
  int btn4 = digitalRead(BtnPIN4);
  int btn5 = digitalRead(BtnPIN5);
  int btn6 = digitalRead(BtnPIN6);
  int btn7 = digitalRead(BtnPIN7);
  int btn8 = digitalRead(BtnPIN8);

  btn1 == 1 ? (INPUT_buff[0] = 0) : (INPUT_buff[0] = 1);
  btn2 == 1 ? (INPUT_buff[1] = 0) : (INPUT_buff[1] = 2);
  btn3 == 1 ? (INPUT_buff[2] = 0) : (INPUT_buff[2] = 4);
  btn4 == 1 ? (INPUT_buff[3] = 0) : (INPUT_buff[3] = 8);
  btn5 == 1 ? (INPUT_buff[4] = 0) : (INPUT_buff[4] = 16);
  btn6 == 1 ? (INPUT_buff[5] = 0) : (INPUT_buff[5] = 32);
  btn7 == 1 ? (INPUT_buff[6] = 0) : (INPUT_buff[6] = 64);
  btn8 == 1 ? (INPUT_buff[7] = 0) : (INPUT_buff[7] = 128);
  


  for(int i=0;i<8;i++)    button_data += INPUT_buff[i];

  INPUT_buff[0] = button_data;

//  Serial.print("Button_data: ");
//  Serial.println(INPUT_buff[0]);
}

int adc_move(void)
{
  adc_data = analogRead(ADCPIN);

//  Serial.print("ADC_data: ");
//  Serial.println(adc_data);

  ADC_val.data = adc_data;
  
  INPUT_buff[1]= ADC_val.a[0];
  INPUT_buff[2]= ADC_val.a[1];
}

void sendToMaster(void)
{
    Wire.write("s");
    for (int i=0; i < 3; i++)
    {
      Wire.write(INPUT_buff[i]);
    }
    Wire.write("e");
}

void loop() 
{
   pushed_button();
   adc_move();
}
