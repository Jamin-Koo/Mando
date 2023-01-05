#include <MsTimer2.h>

#define MOTOR_FRONT_PWM 2
#define MOTOR_FRONT_ENA 3
#define MOTOR_FRONT_ENB 4

#define MOTOR_REAR_PWM 5
#define MOTOR_REAR_ENA 6
#define MOTOR_REAR_ENB 7

float r_speed = 0;
float f_speed = 0;

char data_buffer[5] = {0};
char motor_direction;

static boolean output = HIGH;
 
void front_motor_control(int motor_f_pwm)
{
   if (motor_f_pwm > 0)
  {
    digitalWrite(MOTOR_FRONT_ENA, HIGH);
    digitalWrite(MOTOR_FRONT_ENB, LOW);
    analogWrite(MOTOR_FRONT_PWM, motor_f_pwm);
  }
  else if (motor_f_pwm < 0) 
  {
    digitalWrite(MOTOR_FRONT_ENA, LOW);
    digitalWrite(MOTOR_FRONT_ENB, HIGH);
    analogWrite(MOTOR_FRONT_PWM, -motor_f_pwm);
  }
  else
  {
    digitalWrite(MOTOR_FRONT_ENA, LOW);
    digitalWrite(MOTOR_FRONT_ENB, LOW);
    digitalWrite(MOTOR_FRONT_PWM, 0);
  }
}

void rear_motor_control(int motor_r_pwm)
{
   if (motor_r_pwm > 0)
  {
    digitalWrite(MOTOR_REAR_ENA, LOW);
    digitalWrite(MOTOR_REAR_ENB, HIGH);
    analogWrite(MOTOR_REAR_PWM, motor_r_pwm);
  }
  else if (motor_r_pwm < 0)
  {
    digitalWrite(MOTOR_REAR_ENA, HIGH);
    digitalWrite(MOTOR_REAR_ENB, LOW);
    analogWrite(MOTOR_REAR_PWM, -motor_r_pwm);
  }
  else
  {
    digitalWrite(MOTOR_REAR_ENA, LOW);
    digitalWrite(MOTOR_REAR_ENB, LOW);
    digitalWrite(MOTOR_REAR_PWM, 0);
  }
}

void motor_control(int front_speed, int rear_speed)
{
  front_motor_control(front_speed);
  rear_motor_control(rear_speed);  
}

void control_callback()
{  
  digitalWrite(13, output);
  output =!output;

  motor_control(f_speed, r_speed);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);

  pinMode(MOTOR_FRONT_PWM, OUTPUT);
  pinMode(MOTOR_FRONT_ENA, OUTPUT);  
  pinMode(MOTOR_FRONT_ENB, OUTPUT);


  pinMode(MOTOR_REAR_PWM, OUTPUT);
  pinMode(MOTOR_REAR_ENA, OUTPUT);  
  pinMode(MOTOR_REAR_ENB, OUTPUT);
  
  MsTimer2::set(10, control_callback); //
  MsTimer2::start();
  Serial.begin(115200);
}

void loop() {
  int data;
  String s;
  char check_sp;
  
if (Serial.available() > 0) 
  {
    delay(4);



    check_sp = Serial.read();
    if (check_sp == 's') 
    {
      
      motor_direction = Serial.read();
      for (int i = 0; i < 3; i++) 
      {
        data_buffer[i] = Serial.read();
      }
      
      Serial.write(data_buffer[0]);
      Serial.write(data_buffer[1]);
      Serial.write(data_buffer[2]);
      s=String(data_buffer);
      data = s.toInt();
      if(motor_direction =='r') data = -data;
      Serial.print("  : ");      
      Serial.println(data);
      
      delay(1000);
      f_speed = data;
      r_speed = data;
     }
  }
}
