#include <MsTimer2.h>

#define MOTOR_FRONT_PWM 5
#define MOTOR_FRONT_ENA 7
#define MOTOR_FRONT_ENB 6

#define MOTOR_REAR_PWM 2
#define MOTOR_REAR_ENA 4
#define MOTOR_REAR_ENB 3

#define FRONT_FORWARD_DEADZONE    22
#define FRONT_BACKWARD_DEADZONE   -28
#define REAR_FORWARD_DEADZONE     26
#define REAR_BACKWARD_DEADZONE    -26

float r_speed = 0;
float f_speed = 0;

volatile int32_t encoderPosF = 0;
volatile int32_t encoderPosR = 0;

char data_buffer[5] = {0};
char motor_direction;
char motor_mode;

int32_t target_Pos = 0;
int32_t pos_error = 0;
int32_t pos_error_old = 0;
int32_t pos_error_d = 0;
int32_t pos_error_sum = 0;
int32_t encoderPos_old = 0;

//PID Control P = 5.8 Pd = 8.2  Pi = 0.25 
float P = 4.2;
float Pd = 7.0;
float Pi = 0.25;

int check_Dead = 0;

int pos_pid_pwm = 0;

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

#include <SPI.h>
#define ENC1_ADD 22
#define ENC2_ADD 23
signed long encoder1count = 0;
signed long encoder2count = 0;

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT); 
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);
 
  SPI.begin();
  
 
  digitalWrite(ENC1_ADD,LOW);       
  SPI.transfer(0x88);            
  SPI.transfer(0x03); 
  digitalWrite(ENC1_ADD,HIGH);    

  digitalWrite(ENC2_ADD,LOW);  
  SPI.transfer(0x88);             
  SPI.transfer(0x03); 
  digitalWrite(ENC2_ADD,HIGH); 
}

long readEncoder(int encoder_no) 
{  
  
  unsigned int count_1, count_2, count_3, count_4;
  long count_value1;   
  long count_value;   
  
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);
  SPI.transfer(0x60);
  count_1 = SPI.transfer(0x00); 
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00); 
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);  
  count_value1 = ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  count_value = -count_value1;
  
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);

  SPI.transfer(0x98);    

  SPI.transfer(0x00); 
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00); 
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     
  
  delayMicroseconds(100);  
  
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     
}


void control_callback()
{  
  digitalWrite(13, output);
  output =!output;

  encoder1count = readEncoder(1);
  encoder2count = readEncoder(2);

  if(target_Pos == 0 && check_Dead == 0){
    motor_control(f_speed, r_speed);
  }

  if(check_Dead != 0 && target_Pos == 0) {
    check_Dead = check_Dead>155 ? 155 : check_Dead;
    check_Dead = check_Dead<-155 ? -155 : check_Dead;
    motor_control(f_speed, r_speed);
    f_speed += check_Dead;
    r_speed += check_Dead;
  }
  
  Timer();
}

void Timer()
{
  static boolean output = HIGH;

  output = !output;

  pos_error = target_Pos - encoder1count;
  pos_error_d = pos_error - pos_error_old;
  
  pos_error_sum += pos_error;
  pos_error_sum = (pos_error_sum > 50) ? 50 : pos_error_sum;
  pos_error_sum = (pos_error_sum < -50) ? -50: pos_error_sum;
  if(fabs(pos_error)<=2) pos_error_sum = 0;

  if(f_speed==0 && r_speed == 0 && check_Dead == 0){
    pos_pid_control();
  }
//pid

  encoderPos_old = encoder1count;
  pos_error_old = pos_error;
}

int pos_pid_control(void){
  pos_pid_pwm = P * pos_error + Pd * pos_error_d + Pi *pos_error_sum ;

  pos_pid_pwm = (pos_pid_pwm >= 150) ? 150 : pos_pid_pwm;
  pos_pid_pwm = (pos_pid_pwm <= -150) ? -150 : pos_pid_pwm;
  
  
  motor_control(pos_pid_pwm, 0);
  
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

  initEncoders();          // initialize encoder
  clearEncoderCount(1); 
  clearEncoderCount(2); 
  
  MsTimer2::set(50, control_callback); //
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
    f_speed = 0;
    r_speed = 0;
    target_Pos = 0;
    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(1); 
    clearEncoderCount(2); 
    check_Dead = 0;
    
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

     if (check_sp == 'd') 
    {
    f_speed = 0;
    r_speed = 0;
    target_Pos = 0;
    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(1); 
    clearEncoderCount(2); 
    check_Dead = 0;
    
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
      check_Dead = data;
     }
     
     if (check_sp == 'p') 
    {

    f_speed = 0;
    r_speed = 0;
    target_Pos = 0;
    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(1); 
    clearEncoderCount(2); 
    check_Dead = 0;
    
      motor_direction = Serial.read();
      motor_mode = 'p';
      for (int i = 0; i < 5; i++) 
      {
        data_buffer[i] = Serial.read();
      }
      
      Serial.write(data_buffer[0]);
      Serial.write(data_buffer[1]);
      Serial.write(data_buffer[2]);
      Serial.write(data_buffer[3]);
      Serial.write(data_buffer[4]);
      s=String(data_buffer);
      data = s.toInt();
      if(motor_direction =='r') data = -data;
      Serial.print("  : ");      
      Serial.println(data);
      delay(1000);
      target_Pos = data;
     }
  }
    Serial.print("ENCODER_F : ");
    Serial.print(encoder1count);
    Serial.print("  ENCODER_R : ");
    Serial.print(encoder2count);
    Serial.print("  f_speed : ");
    Serial.print(f_speed);
    Serial.print("  r_speed : ");     
    Serial.println(r_speed);
//    Serial.print("  target_Pos : ");           
//    Serial.println(target_Pos);
}
