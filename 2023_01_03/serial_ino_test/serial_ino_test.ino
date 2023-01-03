char data_buffer[4] = {0};
char motor_direction;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  
  int data;
  String s;
  
  // Run if data available
  
  if (Serial.available() > 0) 
  {
    delay(4);
    // Check for packet header character 0xff
    
    if (Serial.read() == 's') 
    {
      // Insert header into array
      //data_buffer[0] = 's';
      
      // Read remaining 3 characters of data and insert into array
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
     } 
 }
}

//ex) (s)(motordirection)(speed) , __sr040__
