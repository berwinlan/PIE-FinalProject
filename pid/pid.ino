/*
 * PIR sensor testing code from https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/using-a-pir-w-arduino
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define constants
#define LEFT_LED 12
#define RIGHT_LED 13
#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1
#define LEFT_MOTOR 2        // Set left motor pin to M2
#define RIGHT_MOTOR 3       // Set left motor pin to M3

#define BAUD_RATE 9600

#define INITIAL_SPEED 100

// Initialize values
int leftPirState = LOW;             // we start, assuming no motion detected
int rightPirState = LOW;            // we start, assuming no motion detected
int leftVal = 0;                    // variable for reading the pin status
int rightVal = 0;                   // variable for reading the pin status

// Initialize coefficients for tuning
double kp = 30; // Coefficient proportional gain
double ki = 0.002; // Coefficient for integral gain
double kd = 3000; // Coefficient for derivative gain.

// Track times
uint32_t currentTime, previousTime, elapsedTime;

// Track error
int sensor[2] = {0, 0}; // Intialize values for left and right sensors.
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

void setup() {
  Serial.begin(BAUD_RATE);           
  Serial.setTimeout(1);

  // Set up pins
  pinMode(LEFT_LED, OUTPUT);      // declare LED as output
  pinMode(RIGHT_LED, OUTPUT);     // declare LED as output
  pinMode(LEFT_SENSOR, INPUT);     // declare sensor as input
  pinMode(RIGHT_SENSOR, INPUT);    // declare sensor as input

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  previousTime = millis(); // Get the current value of the millis timer
  previousError = 0;

//  delay(5000); // Delay so that the robot does not start moving right away
  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(INITIAL_SPEED); // Subtract by 3 do to unequal motor speeds
  rightMotor->setSpeed(INITIAL_SPEED);

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void loop() {
  // read sensor input values
  leftVal = digitalRead(LEFT_SENSOR);
  rightVal = digitalRead(RIGHT_SENSOR);

  // print sensor input values for debugging
  Serial.print("left: ");
  Serial.print(leftVal);
  Serial.print(", right: ");
  Serial.println(rightVal);

  // Define sensor values as 1 if they are greater than the threshold
  //    meaning that the sensor is reading the tape. Define sensor values
  //    as 0 if they are less than the threshold meaning that the sensor is
  //    is reading the ground.
  sensor[0] = analogRead(LEFT_SENSOR) == HIGH; 
  sensor[1] = analogRead(RIGHT_SENSOR) == HIGH;

  if ((sensor[0] == 1) && (sensor[1] == 1)){ // If robot is on tape, error = 0.
    error = 0;
  } 
  if ((sensor[0] == 1) && (sensor[1] == 0)){ // If robot right of tape, error = -1.
    error = -1;
  }
  if ((sensor[0] == 0) && (sensor[1] == 1)){ // If robot left of tape, error = 1.
    error = 1;
  }

  PID = computePID(error);

  // If robot is to the right of the tape, decrease left motor speed and increase
  // right motor speed. If robot is to the left of the tape, increase left motor speed
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
