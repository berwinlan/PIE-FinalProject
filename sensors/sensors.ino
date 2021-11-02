/*
 * PIR sensor tester
 * Source: https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/using-a-pir-w-arduino
 */

// define pins
#define LEFT_LED 12
#define RIGHT_LED 13
#define LEFT_INPUT A0
#define RIGHT_INPUT A1

// initialize values
int leftPirState = LOW;             // we start, assuming no motion detected
int rightPirState = LOW;            // we start, assuming no motion detected
int leftVal = 0;                    // variable for reading the pin status
int rightVal = 0;                   // variable for reading the pin status

void setup() {
  pinMode(LEFT_LED, OUTPUT);      // declare LED as output
  pinMode(RIGHT_LED, OUTPUT);     // declare LED as output
  pinMode(LEFT_INPUT, INPUT);     // declare sensor as input
  pinMode(RIGHT_INPUT, INPUT);    // declare sensor as input

  // initialize serial monitor
  Serial.begin(9600);
}
 
void loop(){
  // read sensor input values
  leftVal = digitalRead(LEFT_INPUT);
  rightVal = digitalRead(RIGHT_INPUT);
  
  if (leftVal == HIGH and rightVal == HIGH) {            // check if both inputs are HIGH
    digitalWrite(LEFT_LED, HIGH);   // turn LED ON
    digitalWrite(RIGHT_LED, HIGH);  // turn LED ON
    if (leftPirState == LOW and rightPirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected on LEFT and RIGHT!");
      // We only want to print on the output change, not state
      leftPirState = HIGH;
      rightPirState = HIGH;
    }
  } else if (leftVal == HIGH and rightVal == LOW) {            // check if left input is HIGH
    digitalWrite(LEFT_LED, HIGH);  // turn LED ON
    if (leftPirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected on LEFT!");
      // We only want to print on the output change, not state
      leftPirState = HIGH;
    }
  } else if (rightVal == HIGH and leftVal == LOW) {            // check if right input is HIGH
    digitalWrite(RIGHT_LED, HIGH);  // turn LED ON
    if (rightPirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected on RIGHT!");
      // We only want to print on the output change, not state
      rightPirState = HIGH;
    }
  } else if (leftVal == LOW and rightVal == LOW) {          // check if both inputs are LOW
    digitalWrite(LEFT_LED, LOW); // turn LED OFF
    digitalWrite(RIGHT_LED, LOW); // turn LED OFF
    if (leftPirState == HIGH and rightPirState == HIGH){
      // we have just turned off
      Serial.println("Motion ended on LEFT and RIGHT!");
      // We only want to print on the output change, not state
      leftPirState = LOW;
      rightPirState = LOW;
    }
  }
}