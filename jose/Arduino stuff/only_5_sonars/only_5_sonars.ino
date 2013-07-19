
#include <NewPing.h>


//the ones on the crown
//1 is the one closes to the arduino, 5 is the one closes to the breadboard
#define TRIGGER_PIN1  22 
#define TRIGGER_PIN2  24 
#define TRIGGER_PIN3  26 
#define TRIGGER_PIN4  28 
#define TRIGGER_PIN5  30 

#define ECHO_PIN1     10  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ECHO_PIN2     11 
#define ECHO_PIN3     12 
#define ECHO_PIN4     13 
#define ECHO_PIN5     9

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing sonar4(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE);
NewPing sonar5(TRIGGER_PIN5, ECHO_PIN5, MAX_DISTANCE);

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  

  unsigned int uS1 = sonar1.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int uS2 = sonar2.ping();
  unsigned int uS3 = sonar3.ping();
  unsigned int uS4 = sonar4.ping();
  unsigned int uS5 = sonar5.ping();
   
  Serial.print("Ping1: ");
  Serial.print(uS1 / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.print("cm  ");
  
  Serial.print("Ping2: ");
  Serial.print(uS2 / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.print("cm  ");
  
  Serial.print("Ping3: ");
  Serial.print(uS3 / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.print("cm  ");
  
  Serial.print("Ping4: ");
  Serial.print(uS4 / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.print("cm  ");
  
  Serial.print("Ping5: ");
  Serial.print(uS5 / US_ROUNDTRIP_CM); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.printlnsn("cm  ");
  
}
