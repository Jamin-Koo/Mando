#define RING_BUFFER_SIZE 8
#define TEMP_BUFFER_SIZE 64
static char ring_buff[RING_BUFFER_SIZE]={0,};
static int data = 0;

void setup(){
//put your setup code here, to run once:
  Serial.begin(38400); 
  pinMode (45,OUTPUT);
}

void loop(){
  char read_temp[TEMP_BUFFER_SIZE];
  if(Serial.available() >0)
  {
    byte leng = Serial.readBytes(read_temp, TEMP_BUFFER_SIZE);
    digitalWrite(45,HIGH);
    delay(2);
    Serial.print("INPUT data Length : ");
    Serial.println(leng);
    
    for(int i = 0; i < leng; i++)
    {
        ring_buff[data] = read_temp[i];
        if(read_temp[i] == '*') {
          clear_buff();
          --data;
        }
        Serial.print(read_temp[i]);
        data++;
        if(data >= RING_BUFFER_SIZE) data %= RING_BUFFER_SIZE;
    }
    
    Serial.println(" ");
    Serial.print("data no : ");
    Serial.println(data);
    
    for(int j = 0; j < RING_BUFFER_SIZE ; j++)
    {
      Serial.print(ring_buff[j]);
      Serial.print(" ");
    }
    Serial.println(" ");
    digitalWrite(45,LOW);
  }
  
}


void clear_buff(void){
  for(int j = 0; j < RING_BUFFER_SIZE ; j++)
    {
      ring_buff[j] = 0;
    }
}
