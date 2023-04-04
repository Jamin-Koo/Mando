#include <Wire.h>

#define RING_BUFFER_SIZE 8
#define TEMP_BUFFER_SIZE 64

// 슬레이브 주소
int SLAVE = 4;

static char ring_buff[RING_BUFFER_SIZE]={0,};
static int data = 0;

void setup()
{
//put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400); 
  pinMode (45,OUTPUT);
}

void reset_buffer()
{
    for(int j = 0; j < RING_BUFFER_SIZE ; j++)
    {
     ring_buff[j] = '0';
    }
}

void loop()
{  
  char read_temp[TEMP_BUFFER_SIZE];
    
  if(Serial.available() >0)
  {
    reset_buffer();
    byte leng = Serial.readBytes(read_temp, TEMP_BUFFER_SIZE);
    digitalWrite(45,HIGH);
    delay(2);
    digitalWrite(45,LOW);
    Serial.print("INPUT data Lengh : "); Serial.println(leng);
    
    for(int i = 0; i < leng; i++)
    {
      ring_buff[data] = read_temp[i];
      
      Serial.print(read_temp[i]);
      data++;
      if(data >= RING_BUFFER_SIZE) data = 0;
    }
    Serial.println(" ");        Serial.print("Front : "); Serial.println(data);
    for(int j = 0; j < RING_BUFFER_SIZE ; j++)
    {
      Serial.print(ring_buff[j]);   
    }
    Serial.println(" ");
  }

    // SLAVE로 전송
   Serial.print("SendBuff: ");
   Wire.beginTransmission(SLAVE);
   Wire.write('s');
   //Wire.endTransmission(SLAVE);
   for(int i=0; i<8; i++)
   {
      //Wire.beginTransmission(SLAVE);
      Wire.write(ring_buff[i]);
      //Wire.endTransmission(SLAVE);
      Serial.print(ring_buff[i]);
   }
   Wire.write('e');
   Wire.endTransmission(SLAVE);
   Serial.println(" ");
}
