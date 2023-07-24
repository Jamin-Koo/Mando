#include <SoftwareSerial.h>

SoftwareSerial mySerial(1,0);
void sendDataToReceiver(int data);

void setup() {
  // put your setup code here, to run once:
  DDRD = B11111111; // Digital pin 0~7
  DDRB = B00000111; // Digital pin 8~10
  Serial.begin(115200);
  mySerial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int data;

  PORTB = B00000110; //6
  data = PORTB;
//  Serial.print(data);
//  Serial.print(" ");
  sendDataToReceiver(data);
  delay(100);
  
  PORTB = B00000100; //4
  data = PORTB;
//  Serial.print(data);
//  Serial.print(" ");
  sendDataToReceiver(data);
  delay(100);
  
  PORTB = B00000101; //5
  data = PORTB;
//  Serial.print(data);
//  Serial.print(" ");
  sendDataToReceiver(data);
  delay(100);
  
  PORTB = B00000010; //2
  data = PORTB;
//  Serial.print(data);
//  Serial.print(" ");
  sendDataToReceiver(data);
  delay(100);
  
  PORTB = B00000001; //1
  data = PORTB;
  //Serial.print(data);
  //Serial.print(" ");
  sendDataToReceiver(data);
  delay(100);
  
  PORTB = B00000011; //3
  data = PORTB;
  //Serial.println(data);
  sendDataToReceiver(data);
  delay(100);

}

void sendDataToReceiver(int data) 
{
  Serial.println(data);
  mySerial.write(data);
}
