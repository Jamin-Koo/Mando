// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define echo_A   4  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define echo_B   5 
#define echo_C   6 
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonarA(echo_A,echo_A , MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarB(echo_B,echo_B , MAX_DISTANCE);
NewPing sonarC(echo_C,echo_C , MAX_DISTANCE);

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  delay(500);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("PingA: ");
  Serial.print(sonarA.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");

  Serial.print("PingB: ");
  Serial.print(sonarB.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  
  Serial.print("PingC: ");
  Serial.print(sonarC.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
}
