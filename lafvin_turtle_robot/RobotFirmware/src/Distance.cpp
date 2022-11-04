/* File Distance.c */
#include <Arduino.h>
#include "Distance.h"
#include "Pinout.h"
#include "Servo.h"

#define servo_settle_time  750 // settling time in ms after steering servo motor moving

Servo myservo;        // set myservo
int currentServoPosition = -1;

unsigned long lastDistanceMeasureTime = 0;
int fDistance = 0;      // forward distance
int rDistance = 0;      // right distance
int lDistance = 0;      // left distance

void setServoPosition(int position)
{
  if(position != currentServoPosition)
  {
    myservo.attach(ServoPin);
    myservo.write(position);
    delay(servo_settle_time);
    myservo.detach();
    currentServoPosition = position;
  }
}

void measureDistanceFront()   // measure the distance ahead
{
  setServoPosition(90);
  float distance = measureDistance();
  if (0 < distance)
  {
    fDistance = distance;
  }
}
void measureDistanceLeft()   // measure distance on the left
{
  setServoPosition(10);
  float distance = measureDistance();
  if (0 < distance)
  {
    lDistance = distance;
  }
}
void measureDistanceRight()   //  measure distance on the right
{
  setServoPosition(170);
  float distance = measureDistance();
  if (0 < distance)
  {
    rDistance = distance;
  }
}

//returns distance from HC-SR04 ultrasonic module in cm
float measureDistance()
{
  float distance = -1;

  //according HC-SR04 datasheet, wait at least 60ms between two distance measurements
  if (millis() >= lastDistanceMeasureTime + 60)
  {
    //only start a new distance measurement when HC-SR04 module is ready
    if (digitalRead(EchoPin) == LOW)
    {
      /* The following trigPin/echoPin cycle is used to determine the
        distance of the nearest object by bouncing soundwaves off of it. */
      digitalWrite(TrigPin, LOW);
      delayMicroseconds(2);

      digitalWrite(TrigPin, HIGH);
      delayMicroseconds(10);

      digitalWrite(TrigPin, LOW);
      float duration = pulseIn(EchoPin, HIGH, 50000);

      //Calculate the distance (in cm) based on the speed of sound.
      distance = duration / 58.2;
    }
    lastDistanceMeasureTime = millis();
  }
  return distance;
}

float getDistanceFront()
{
    return fDistance;
}
float getDistanceLeft()
{
    return lDistance;
}

float getDistanceRight()
{
    return rDistance;
}

