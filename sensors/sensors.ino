/*
 * PIR sensor tester
 * Source: https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/using-a-pir-w-arduino
 */

// define pins
#define LEFT_LED 12
#define RIGHT_LED 13
#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1

// threshold of HIGH v. LOW
#define THRESHOLD 800

// Serial baud rate
#define BAUD_RATE 9600

// initialize values
int leftPirState = LOW;             // we start, assuming no motion detected
int rightPirState = LOW;            // we start, assuming no motion detected
int leftVal = 0;                    // variable for reading the pin status
int rightVal = 0;                   // variable for reading the pin status
bool leftHigh, rightHigh;

void setup() {
  pinMode(LEFT_LED, OUTPUT);      // declare LED as output
  pinMode(RIGHT_LED, OUTPUT);     // declare LED as output
  pinMode(LEFT_SENSOR, INPUT);     // declare sensor as input
  pinMode(RIGHT_SENSOR, INPUT);    // declare sensor as input

  // initialize serial monitor
  Serial.begin(BAUD_RATE);
}
 
void loop(){
  // read sensor input values
  leftVal = analogRead(LEFT_SENSOR);
  rightVal = analogRead(RIGHT_SENSOR);

  // True if the reading value is over a threshold
  leftHigh = leftVal >= THRESHOLD;
  rightHigh = rightVal >= THRESHOLD;
  
  // print sensor input values for debugging
  Serial.print("left: ");
  Serial.print(leftHigh);
  Serial.print(", right: ");
  Serial.println(rightHigh);
  
  if (leftHigh and rightHigh) {            // check if both inputs are HIGH
    digitalWrite(LEFT_LED, HIGH);   // turn LED ON
    digitalWrite(RIGHT_LED, HIGH);  // turn LED ON
//    Serial.println("both");
    if (leftPirState == LOW and rightPirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected on LEFT and RIGHT!");
      // We only want to print on the output change, not state
      leftPirState = HIGH;
      rightPirState = HIGH;
    }
  } else if (leftHigh and not rightHigh) {            // check if left input is HIGH
    digitalWrite(LEFT_LED, HIGH);  // turn LED ON
//    Serial.println("left");
    if (leftPirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected on LEFT!");
      // We only want to print on the output change, not state
      leftPirState = HIGH;
    }
  } else if (rightHigh and not leftHigh) {            // check if right input is HIGH
    digitalWrite(RIGHT_LED, HIGH);  // turn LED ON
//    Serial.println("right");
    if (rightPirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected on RIGHT!");
      // We only want to print on the output change, not state
      rightPirState = HIGH;
    }
  } else if (not leftHigh and not rightHigh) {          // check if both inputs are LOW
    digitalWrite(LEFT_LED, LOW); // turn LED OFF
    digitalWrite(RIGHT_LED, LOW); // turn LED OFF
//    Serial.println("none");
    if (leftPirState == HIGH and rightPirState == HIGH){
      // we have just turned off
      Serial.println("Motion ended on LEFT and RIGHT!");
      // We only want to print on the output change, not state
      leftPirState = LOW;
      rightPirState = LOW;
    }
  }
}
