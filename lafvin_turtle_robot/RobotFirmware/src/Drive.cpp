/* File Drive.c */

#include <Arduino.h>
#include "Drive.h"
#include "Pinout.h"

/* slightly modify these values if robot doesn't run straight on */
#define leftSpeedCorrection 0
#define rightSpeedCorrection 0

int  currentSpeed = DefaultSpeed;

void setCurrentSpeed(unsigned char speed)
{
  currentSpeed = speed;
}

void setSpeed(unsigned char speed)
{
  analogWrite(motorEnL, speed + leftSpeedCorrection);
  analogWrite(motorEnR, speed + rightSpeedCorrection);
}

void backward()     // move backward
{
  digitalWrite(motorR1, HIGH);  // left wheel moves forward
  digitalWrite(motorR2, LOW);
  digitalWrite(motorL1, HIGH);   // right wheel moves forward
  digitalWrite(motorL2, LOW);
  setSpeed(currentSpeed);
}
void stop()         // stop
{
  //don't set currentSpeed here - we want to continue with "currentSpeed" as soon as any new direction is set
  setSpeed(0);
  delay(300);
}
void right()        // turn right (single wheel)
{
  digitalWrite(motorR1, LOW); // wheel on the right moves backward
  digitalWrite(motorR2, HIGH); // wheel on the right moves backward
  digitalWrite(motorL1, LOW); // wheel on the right does not move
  digitalWrite(motorL2, LOW); // wheel on the right does not move
  setSpeed(currentSpeed);
}
void left()         // turn left (single wheel)
{
  digitalWrite(motorR1, LOW); // wheel on the left does not move
  digitalWrite(motorR2, LOW); // wheel on the left does not move
  digitalWrite(motorL1, LOW); // wheel on the left moves backward
  digitalWrite(motorL2, HIGH); // wheel on the left moves backward
  setSpeed(currentSpeed);
}

void rotateLeft()        // turn left (single wheel)
{
  digitalWrite(motorR1, LOW); // wheel on the right moves backward
  digitalWrite(motorR2, HIGH); // wheel on the right moves backward
  digitalWrite(motorL1, HIGH); // wheel on the right does not move
  digitalWrite(motorL2, LOW); // wheel on the right does not move
  setSpeed(currentSpeed);
}
void rotateRight()         // turn right (single wheel)
{
  digitalWrite(motorR1, HIGH); // wheel on the left does not move
  digitalWrite(motorR2, LOW); // wheel on the left does not move
  digitalWrite(motorL1, LOW); // wheel on the left moves backward
  digitalWrite(motorL2, HIGH); // wheel on the left moves backward
  setSpeed(currentSpeed);
}

void forward()          // move forward
{
  digitalWrite(motorR1, LOW);  // left wheel moves forward
  digitalWrite(motorR2, HIGH);
  digitalWrite(motorL1, LOW);   // right wheel moves forward
  digitalWrite(motorL2, HIGH);
  setSpeed(currentSpeed);
}

void speedUp()
{
  currentSpeed = currentSpeed + SpeedStep;
  if (currentSpeed > MaxSpeed)
  {
    currentSpeed = MaxSpeed;
  }
  setSpeed(currentSpeed);
}

void slowDown()
{
  currentSpeed = currentSpeed - SpeedStep;
  if (currentSpeed < MinSpeed)
  {
    currentSpeed = MinSpeed;
  }
  setSpeed(currentSpeed);
}
