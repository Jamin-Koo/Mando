#include <avr/wdt.h>
unsigned long ToggleDelay;
int check = 2;

void setup()
{
  pinMode(2,OUTPUT);
  Serial.begin(115200);
  Serial.println("start");
  wdt_disable();
  ToggleDelay = 450;
  wdt_enable(WDTO_2S);
  digitalWrite(check, LOW);
}

void loop()
{
  digitalWrite(check, LOW);
  wdt_reset();
  unsigned long now = millis();
  delay(ToggleDelay);
  ToggleDelay += 500;
  Serial.println(ToggleDelay);
  
  digitalWrite(check, HIGH);
}
