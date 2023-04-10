#include <Wire.h>
#include <avr/wdt.h>

const uint8_t MASTER_ADDR = 4;
const uint8_t SIGNAL_PIN = 2;
const uint32_t WATCHDOG_TIMEOUT = 5000; // 5 seconds

void setup() {
  Wire.begin(MASTER_ADDR);
  pinMode(SIGNAL_PIN, INPUT);
  wdt_enable(WDTO_8S);
}

void loop() {
  uint8_t signal = digitalRead(SIGNAL_PIN);
  Wire.beginTransmission(MASTER_ADDR);
  Wire.write(signal);
  Wire.endTransmission();

  wdt_reset();

  delay(100);
}
