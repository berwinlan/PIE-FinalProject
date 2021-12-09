/*
 * Moves a servo based on phototransistor.
 * Code from Sweep sample code used
 */

#include <Servo.h>
#include <movingAvg.h>

#define BAUD_RATE 115200
#define SERVO_PIN 9             // D9
#define PHOTOTRANSISTOR_PIN A0
#define THRESHOLD 10            // light/dark threshold
#define DELAY 900

// Servo positions of head
#define OUT_POS 0
#define IN_POS 180

Servo myservo;  // create servo object to control a servo
movingAvg phototransistor(10);      // use 10 data points for moving avg

bool head_in;      // toggle based on head position

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  head_in = true;
  
  myservo.attach(SERVO_PIN);
  myservo.write(IN_POS);         // initial position of head in
  
  phototransistor.begin();
  phototransistor.reset();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t t;
  uint16_t sensorData;
  uint16_t sensorMovingAvg;

  t = millis();
  sensorData = analogRead(PHOTOTRANSISTOR_PIN);
  sensorMovingAvg = phototransistor.reading(sensorData);
//  Serial.print("data reading: ");
//  Serial.println(sensorData);
//  Serial.print("moving avg: ");
  Serial.println(sensorMovingAvg);

  if ((sensorMovingAvg <= THRESHOLD) && head_in) {
    myservo.write(OUT_POS);
    delay(DELAY);
    head_in = !head_in;
  } else if ((sensorMovingAvg > THRESHOLD) && !head_in) {
    myservo.write(IN_POS);
    delay(DELAY);
    head_in = !head_in;
  }
}
