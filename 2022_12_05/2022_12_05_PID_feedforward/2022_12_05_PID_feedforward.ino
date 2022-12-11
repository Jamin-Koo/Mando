/*
  MsTimer2 is a small and very easy to use library to interface Timer2 with
  humans. It's called MsTimer2 because it "hardcodes" a resolution of 1
  millisecond on timer2
  For Details see: http://www.arduino.cc/playground/Main/MsTimer2
*/
#include <MsTimer2.h>
#define encodePinA1 2
#define encodePinB1 3
#define INA 4
#define PWM 5
#define check_pin 6

volatile int32_t encoderPos = 0; 
volatile int32_t encoderPos_old = 0; 

int32_t target_Pos = 0;
int32_t pos_error = 0;
int32_t pos_error_d = 0;
int32_t  pos_error_old = 0;


// P = 5.8 Pd = 8.2 
float P = 3.0;
float Pd = 5.0;
float Pi = 0.25;

int pid_pwm = 0;
double pos_error_sum = 0.0;

// Switch on LED on and off each half second

#if ARDUINO >= 100
const int led_pin = LED_BUILTIN;	// 1.0 built in LED pin var
#else
const int led_pin = 13;			// default to pin 13
#endif

void interrupt_setup(void)
{
  delayMicroseconds(50);
  pinMode(encodePinA1, INPUT_PULLUP);                 
  pinMode(encodePinB1, INPUT_PULLUP);             
  attachInterrupt(0, encoder, RISING);                               
}

void encoder()  
{                                               
  if(digitalRead(encodePinB1)==HIGH){
    encoderPos--;            
  }
  else{
    encoderPos++;          
  }
}

  
void timer()
{
  static boolean output = HIGH;
  
  digitalWrite(led_pin, output);
  digitalWrite(check_pin, output);
  output = !output;

  pos_error = target_Pos - encoderPos;
  pos_error_d = pos_error - pos_error_old;
  encoderPos_old = encoderPos;
  
  pos_error_sum += pos_error;
  pos_error_sum = (pos_error_sum > 50) ? 50 : pos_error_sum;
  pos_error_sum = (pos_error_sum < -50) ? -50: pos_error_sum;
  if(fabs(pos_error)<=2) pos_error_sum = 0;
  pos_error_old = pos_error;
  
  pid_pwm = P * pos_error + Pd * pos_error_d + Pi *pos_error_sum;
  
  pid_pwm = (pid_pwm >= 255)? 255 : pid_pwm;

  pid_pwm = (pos_error >= target_Pos / 3) ? 255 : pid_pwm; // forward_control
  
  if(pid_pwm > 0) motor_control(1,pid_pwm);
  else motor_control(-1,-pid_pwm);
  
  digitalWrite(check_pin, LOW);
}

void motor_control(int direc, int pwm){
  pwm = feedforward(pwm);
  
  switch(direc)
  {
    case -1 : digitalWrite(INA, HIGH);
          analogWrite(PWM,pwm);
          break;
    case 0 : digitalWrite(INA, LOW);
          analogWrite(PWM,0);
          break;
    case 1 : digitalWrite(INA, LOW);
          analogWrite(PWM,pwm);
          break;
  }
}

int feedforward(int pwm) {
    pwm = pwm >= 255 ? pwm = 255 : pwm ;
}

void setup()
{
  pinMode(led_pin, OUTPUT);
  pinMode(check_pin, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(PWM, OUTPUT);
  
  MsTimer2::set(50, timer); // 500ms period
  MsTimer2::start();
  interrupt_setup();
  Serial.begin(115200);
  Serial.println("pos, pos_error");
  target_Pos = 500;
}

void loop()
{
  //motor_control(1,30);
  if(fabs(pos_error) >= 0) {
  Serial.print(encoderPos);
  Serial.print(",");
  Serial.print(pos_error);
  Serial.print(",");
  Serial.print(pid_pwm);
  Serial.print(",");
  Serial.println(pos_error_sum);

  }
//  if(pos_error<5){
//    motor_control(1,0);
//  }
//  else{
//    motor_control(-1,30);
//  }

//forward 14
  //backwrad 13
}
