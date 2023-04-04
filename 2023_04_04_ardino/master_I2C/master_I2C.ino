#include <Wire.h>
#include <MsTimer2.h>

// 슬레이브 주소
int SLAVE = 4;
int flag = 0;

static int receive_buff[4] ={0,};
static int send_buff[4] ={0,};

void setup()
{
  Wire.begin();
  Serial.begin(38400); 

  MsTimer2::set(500,send_To_slave);
  MsTimer2::start();
}

void send_To_slave()
{ 
  flag = 1;
}

void i2c_communication() 
{
  Wire.requestFrom(SLAVE, 4);
  Serial.print("Receive: "); 

  for (int j = 0 ; j < 4 ; j++) 
  {  
     receive_buff[j] = Wire.read(); 
     Serial.print(receive_buff[j]); Serial.print(" "); 
  }
  Serial.println();
}

void loop()
{  
  if(flag == 1){
   send_buff[0] = 19;
   send_buff[1] = 98;
   send_buff[2] = 4;
   send_buff[3] = 11;
   Wire.beginTransmission(SLAVE);
   Serial.print("Send:    ");
   for (int i = 0 ; i < 4 ; i++) 
   {  
     Wire.write(send_buff[i]);
     Serial.print(send_buff[i]); Serial.print(" ");
   }
   Wire.endTransmission(SLAVE);
   Serial.println();
   i2c_communication(); 
   flag = 0;
  }
}
