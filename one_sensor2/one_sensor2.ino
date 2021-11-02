/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-motion-sensor
 */

#define LEFT_SENSOR 7
int val = 0;

void setup() {
  Serial.begin(9600);            // initialize serial
  pinMode(LEFT_SENSOR, INPUT); // set arduino pin to input mode to read value from OUTPUT pin of sensor
}

void loop() {
  val = digitalRead(LEFT_SENSOR);   // read new state
  Serial.println(val);
  if (not val) {
    digitalWrite(12, HIGH);
  }

//  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {   // pin state change: LOW -> HIGH
//    Serial.println("Motion detected!");
    // TODO: turn on alarm, light or activate a device ... here
//  }
//  else
//  if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {   // pin state change: HIGH -> LOW
//    Serial.println("Motion stopped!");
    // TODO: turn off alarm, light or deactivate a device ... here
//  }
}
