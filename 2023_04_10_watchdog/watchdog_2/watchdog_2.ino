#include <Wire.h>
#include <avr/wdt.h>

const uint8_t MASTER_ADDR = 4;

const uint8_t button_PIN = 2;
const uint8_t LED_PIN = 3;
const uint8_t LED_PIN_WDTOUT = 4;
int startFlag = 0;

const uint32_t WATCHDOG_TIMEOUT = 5000; // 5 seconds

void setup() {
  Wire.begin(MASTER_ADDR);
  pinMode(button_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_WDTOUT, OUTPUT);
  digitalWrite(LED_PIN_WDTOUT, HIGH);
  wdt_enable(WDTO_2S);
  Serial.begin(115200);
  Serial.println("START");
  startFlag = 0;
}

void loop() {
  if(startFlag == 0){
   digitalWrite(LED_PIN_WDTOUT, LOW);
   delay(300);
   startFlag = 1; 
  }
  uint8_t signal = digitalRead(button_PIN);
  Wire.beginTransmission(MASTER_ADDR);
  Wire.write(signal);
  if (signal == LOW){
    Serial.println("OK");
    digitalWrite(LED_PIN, HIGH);
    wdt_reset();
  }
  Wire.endTransmission();


  delay(100);
  digitalWrite(LED_PIN, LOW);
}
