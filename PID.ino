#include <PID_v1.h>
#include <Servo.h>
 
// Defining X & Y outputs to be sent to servos
#define OutputServoX 7
#define OutputServoY 8
 
// Initializing corner pins of plate into an array
const byte cornerPins[] = {2, 3, 4, 5};
//Setting sensePin
const byte sensePin = A0;
 double tolerance = 50;
// Variable to track how long its been since plate has been touched
unsigned int NoTouchCount;
 
//Defining setpoint
double SetPointX = 490;
//545
double SetPointY = 512;
 //487
//Defining maximum and minimum X and Y values
double Xmax = 700;
double Ymax = 750;
double Xmin = 220;
double Ymin = 250;
 
// Defining Variables for PID
double OutputX;
double OutputY;
 
//defining variables for X & Y process varbiables to be stored into (Inputs)
double pvX;
double pvY;
 


float KpX = 0.5;//0,1
float KiX = 0.5; //0.1
float KdX = 0.00075; //0.0008
 
float KpY = 0.5;
float KiY = 0.5;
float KdY = 0.00075;

//Defining sampling time
int Ts = 1;
 
// PID from library
PID PIDX(&pvX, &OutputX, &SetPointX, KpX, KiX, KdX, REVERSE);
PID PIDY(&pvY, &OutputY, &SetPointY, KpY, KiY, KdX, REVERSE);
 
 // Initialize next execution time
unsigned long nextExecutionTime = millis();
 
 //--- Servo.h Instllation ---//
 
 // Introducing servo motors controlled
 Servo servoX; // servo object to control servo for X position
 Servo servoY; // servo object to control servo for Y position
 
 
 double thetaX;
 double thetaY;
 double thetaXflat = 115;
 double thetaYflat = 90;
 int servo_delay = 20;
 
 // Maximum angular displacement values of servo motors (mechanical limit)
 double thetaXmax = 90;
 double thetaYmax = 90;
 
 //define variables for tracking time and previous values for process variables
 unsigned long currentTime;
 unsigned long previousTime;
 int changeThreshold = 20;
 int resetTimeThreshold = 5000;
 int PrevpvX = 0;
 int PrevpvY = 0;
 
 
 // Defining sampling time
const unsigned long samplingInterval = 10; // in milliseconds, for example for 100 Hz operation
 
//-------SETUP-------//
 
 
 
// Looping through the array of pins
void setup() {
  for(int i = 0; i < 4; i++){
    pinMode (cornerPins[i], OUTPUT);
  }
  Serial.begin(9600);
 
 
// Attaching servo motors to pins
OutputY = OutputServoY;
OutputX = OutputServoX;
  servoX.attach(OutputServoX);
  servoY.attach(OutputServoY);
 
// setting flat position
  OutputX = thetaXflat;
  OutputY = thetaYflat;
 
 servoX.write(OutputX);
 servoY.write(OutputY);
 
//setting starting inputs for PID
  //SetPointX = 0;
  //SetPointY = 0;
 
  PIDX.SetMode(AUTOMATIC);
  PIDY.SetMode(AUTOMATIC);
 
 //setting limiting parameters for servos
PIDX.SetOutputLimits(thetaXflat-thetaXmax, thetaXflat+thetaXmax);
PIDY.SetOutputLimits(thetaYflat-thetaYmax, thetaYflat+thetaYmax);
 
PIDX.SetSampleTime(Ts);
PIDY.SetSampleTime(Ts);
 
thetaX = thetaXflat;
thetaY = thetaYflat;
 

 
}
 
//-------SENSING BALL AND COMPUTING PID VARIABLES-------//


 
void loop() {
 // Implement dynamic delay for fixed sampling rate
  unsigned long currentStartTime = millis();

  if (currentStartTime >= nextExecutionTime) {

  //setting pin 2 & 3 HIGH and pin 4 & 5 LOW on arduino to read X process variable
  digitalWrite(cornerPins[0], HIGH);
  digitalWrite(cornerPins[1], HIGH);
  digitalWrite(cornerPins[2], LOW);
  digitalWrite(cornerPins[3], LOW);
 
  //pvX = analogRead(sensePin);
 int numReadings = 20;
 int total =0;

for(int i = 0; i<numReadings; i++){
  total+=analogRead(sensePin);
  delay(1);
  
}
pvX = total/numReadings ;


//setting pin 2 & 3 HIGH and pin 4 & 5 LOW on arduino to read Y process variable
  digitalWrite(cornerPins[0], HIGH);
  digitalWrite(cornerPins[1], LOW);
  digitalWrite(cornerPins[2], HIGH);
  digitalWrite(cornerPins[3], LOW);
 
 
total = 0;
 for(int i = 0; i<numReadings; i++){
  total+=analogRead(sensePin);
  delay(1);
 }

  //pvY = analogRead(sensePin);
 pvY = total/numReadings;

 // Serial.print("Current: ");
  Serial.print(pvX);
  Serial.print(", ");
  Serial.println(pvY);
  //Serial.print(" | Previous: ");
  //Serial.print(PrevpvX);
  //Serial.print(", ");
  //Serial.println(PrevpvY);
  

 
//update previous values for next iteration
PrevpvX = pvX;
PrevpvY = pvY;

 
 
 if( pvX && pvY > 0) {
 
//activating servos
  servoX.attach(OutputServoX);
  servoY.attach(OutputServoY);
 
// resetting touch count
  NoTouchCount = 0;
 
PIDX.Compute();
PIDY.Compute();

servoX.write(OutputX);
servoY.write(OutputY);
 

 }
else {
 
  NoTouchCount++;
 
 //making the platform flat if plat hasn't been touched
 
 if(NoTouchCount > 30) {
  OutputX = thetaXflat;
  OutputY = thetaYflat;
 }


 // Update next execution time for the next loop iteration
    nextExecutionTime = currentStartTime + samplingInterval;

    // Dynamically adjust delay to maintain the sampling rate
    long timeSpent = millis() - currentStartTime;
    long dynamicDelay = samplingInterval - timeSpent;

    if (dynamicDelay > 0) {
      delay(dynamicDelay);
}

  }
  
  }
}

