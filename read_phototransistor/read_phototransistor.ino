/*
 * Code from Sweep sample code used
 */

#include <Servo.h>

#define BAUD_RATE 115200
#define LOOP_INTERVAL 20
#define SERVO_PIN 1             // D1
#define PHOTOTRANSISTOR_PIN A0
#define THRESHOLD 5
#define DELAY 5

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position
uint32_t loop_time;
bool head_in;      // toggle based on head position

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  loop_time = millis();
  head_in = true;
  
  myservo.attach(SERVO_PIN);
  myservo.write(0);         // initial position of head in
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t t;
  uint16_t res;

  t = millis();
  res = analogRead(PHOTOTRANSISTOR_PIN);
//  Serial.println(res);

  loop_time = t;

  if ((res <= THRESHOLD) & head_in) {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
      Serial.print(head_in);
      Serial.println(pos);
      myservo.write(pos);                 // tell servo to go to position in variable 'pos'
      delay(DELAY);                       // waits for the servo to reach the position
    }
    head_in = false;
  } else if ((res > THRESHOLD) & !head_in) {
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      Serial.print(head_in);
      Serial.println(pos);
      myservo.write(pos);                 // tell servo to go to position in variable 'pos'
      delay(DELAY);                       // waits for the servo to reach the position
    }
    head_in = true;
  }
}
