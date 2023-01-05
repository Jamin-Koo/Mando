#include <MsTimer2.h>

#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 4
#define forward 1
#define backward -1

int pos = 0;
int Dead_count = 0;
int Dead_count_old = -1;

int32_t target_Pos = 0;
int32_t pos_error = 0;
int32_t pos_error_d = 0;
int32_t  pos_error_old = 0;
volatile int32_t encoderPos = 0; 
volatile int32_t encoderPos_old = 0; 

void setup() {

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  MsTimer2::set(1000, timer);
  MsTimer2::start();
  Serial.begin(9600);
}

void loop() {
  setMotor(backward, Dead_count, PWM, IN1);
  
  //Serial.print("Dead_count : ");
  if(Dead_count != Dead_count_old){
  Serial.print(Dead_count);
  Serial.print(",");
  //Serial.print("POS : ");
  Serial.println(pos_error_d);
  Dead_count_old = Dead_count;
  }

}

void timer() {
  
  pos_error = pos;
  pos_error_d = pos_error - pos_error_old;
  pos_error_old = pos_error;
  
  //if(abs(pos) <= 2 ){
  Dead_count++;
  //}
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,HIGH);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos--;
  }
  else{
    pos++;
  }
}

//forward == 13  backward == 17
