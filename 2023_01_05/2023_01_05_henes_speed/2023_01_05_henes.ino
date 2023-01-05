#include <MsTimer2.h>

#define MOTOR_FRONT_PWM 2
#define MOTOR_FRONT_ENA 3
#define MOTOR_FRONT_ENB 4

#define MOTOR_REAR_PWM 5
#define MOTOR_REAR_ENA 6
#define MOTOR_REAR_ENB 7

float r_speed = 0;
float f_speed = 0;

volatile int32_t encoderPosF = 0;
volatile int32_t encoderPosR = 0;

char data_buffer[4] = {0};
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
  Serial.print("motor_r_pwm");
  Serial.println(motor_f_pwm);
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
  Serial.print("motor_r_pwm");
  Serial.println(motor_r_pwm);
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
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
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

  initEncoders();          // initialize encoder
  clearEncoderCount(1); 
  clearEncoderCount(2); 
  
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
      for (int i = 0; i < 4; i++) 
      {
        data_buffer[i] = Serial.read();
      }
      
      Serial.write(data_buffer[0]);
      Serial.write(data_buffer[1]);
      Serial.write(data_buffer[2]);
      Serial.write(data_buffer[3]);
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
    encoder1count = readEncoder(1);
    encoder2count = readEncoder(2);
//    Serial.print("ENCODER_F : ");
//    Serial.print(encoder1count);
//    Serial.print("  ENCODER_R : ");
//    Serial.println(encoder2count);
//    Serial.print("  f_speed : ");
//    Serial.print(f_speed);
//    Serial.print("  r_speed : ");
//    Serial.println(r_speed);
}
