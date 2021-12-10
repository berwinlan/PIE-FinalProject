/*
 * PIR sensor base code from https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/using-a-pir-w-arduino
 * PID code from MP3
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <movingAvg.h>

// Define pins
#define LEFT_PIR A1      // Set left sensor pin to A1
#define RIGHT_PIR A2     // Set right sensor pin to A2
#define LEFT_MOTOR 3        // Set left motor pin to M3
#define RIGHT_MOTOR 2       // Set right motor pin to M2
#define SERVO_PIN 2             // D1
#define PHOTOTRANSISTOR_PIN A0

// Thresholds of HIGH v. LOW
#define PIR_THRESHOLD 800
#define LIGHT_THRESHOLD 10            // light/dark LIGHT_THRESHOLD

// Servo constants
#define DELAY 900
#define IN_POS 180
#define OUT_POS 0

// Serial baud rate
#define BAUD_RATE 115200

// Set initial speed of motors
// This also controls overall speed once it gets going
#define INITIAL_SPEED 50

// Initialize values
int leftPirState = LOW;             // we start, assuming no motion detected
int rightPirState = LOW;            // we start, assuming no motion detected
int leftVal = 0;                    // variable for reading the pin status
int rightVal = 0;                   // variable for reading the pin status
bool leftHigh, rightHigh;
bool head_in;      // toggle based on head position

// Initialize coefficients for tuning
double kp = 20; // Coefficient proportional gain - raw increase or decrease
double ki = 0.001; // Coefficient for integral gain
double kd = 100; // Coefficient for derivative gain - linear with amplitude

// Track times
uint32_t currentTime, previousTime, elapsedTime;

// Track error
int sensor[2] = {0, 0}; //    Initialize values for left and right sensors.
                        //    errors can be -1, 0, or 1 depending on which side
                        //    of the tape the robot is on.
                        //    sensor[0] is left wheel. sensor[1] is right wheel.
int error, previousError; // Track current and previous errors
double PID = 0.0; // Initialize PID for control loop 
double cumeError, rateError; 

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *leftMotor = AFMS.getMotor(LEFT_MOTOR);   // attach left motor
Adafruit_DCMotor *rightMotor = AFMS.getMotor(RIGHT_MOTOR);  // attach right motor

// Set up servo and phototransistor
Servo headServo;  // create servo object to control a servo
movingAvg phototransistor(10);      // use 10 data points for moving avg

void setup() {
  // Initialize serial monitor
  Serial.begin(BAUD_RATE);           
  Serial.setTimeout(1);
  head_in = true;

  // Initialize servo and phototransistor moving average
  headServo.attach(SERVO_PIN);
  headServo.write(OUT_POS);         // initial position of head out
  
  phototransistor.begin();
  phototransistor.reset();

  // Set up pins
  pinMode(LEFT_PIR, INPUT);     // declare sensor as input
  pinMode(RIGHT_PIR, INPUT);    // declare sensor as input

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  previousTime = millis(); // Get the current value of the millis timer
  previousError = 0;

  delay(5000); // Delay so that the robot does not start moving right away
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(INITIAL_SPEED);
  rightMotor->setSpeed(INITIAL_SPEED);

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void loop() {
  // variables to track phototransistor readings
  uint16_t phototransistorData;
  uint16_t phototransistorMovingAvg;

  // read sensor input values
  leftVal = analogRead(LEFT_PIR);
  rightVal = analogRead(RIGHT_PIR);

  phototransistorData = analogRead(PHOTOTRANSISTOR_PIN);
  phototransistorMovingAvg = phototransistor.reading(phototransistorData);

  if ((phototransistorMovingAvg <= LIGHT_THRESHOLD) && head_in) {
    headServo.write(OUT_POS);
    delay(DELAY);
    head_in = !head_in;
    Serial.println("head out");
  } else if ((phototransistorMovingAvg > LIGHT_THRESHOLD) && !head_in) {
    headServo.write(IN_POS);
    delay(DELAY);
    head_in = !head_in;
    Serial.println("head in");
  }

  // print sensor input values for debugging
  Serial.print("left: ");
  Serial.print(leftVal >= PIR_THRESHOLD);
  Serial.print(", right: ");
  Serial.println(rightVal >= PIR_THRESHOLD);

  //  Define sensor values as 1 if they are greater than the PIR_THRESHOLD
  //  meaning that the sensor sees infrared. Define sensor values as 0 if 
  //  they are less than the PIR_THRESHOLD meaning that the sensor is
  //  not seeing infrared.
  sensor[0] = leftVal >= PIR_THRESHOLD; 
  sensor[1] = rightVal >= PIR_THRESHOLD;

  if ((sensor[0] == 1) && (sensor[1] == 1)){ // If robot is on tape, error = 0.
    error = 0;
  } else if ((sensor[0] == 1) && (sensor[1] == 0)){ // If robot right of tape, error = -1.
    error = -1;
  } else if ((sensor[0] == 0) && (sensor[1] == 1)){ // If robot left of tape, error = 1.
    error = 1;
  }

  PID = computePID(error);

  Serial.println(PID);

  // If robot is to the right of the infrared, decrease left motor speed and increase
  // right motor speed. If robot is to the left of the infrared, increase left motor speed
  // and decrease right motor speed.
  leftMotor->setSpeed(INITIAL_SPEED + PID); 
  rightMotor->setSpeed(INITIAL_SPEED - PID);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD); 
}

// Compute PID
double computePID(double error) {
  currentTime = millis(); // Get the current value of the millis timer
  elapsedTime = double(currentTime - previousTime);

  cumeError += error*elapsedTime;                 // compute integral
  rateError = (error - previousError)/elapsedTime;    // compute derivative

  double diff = kp * error + ki * cumeError + kd * rateError;   // compute PID output

  previousError = error;
  previousTime = currentTime;

  return diff;        // return PID output
}
