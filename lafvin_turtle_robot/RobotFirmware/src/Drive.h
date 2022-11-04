/* File Drive.h */

#ifndef __DRIVE_H
#define __DRIVE_H

/* Speed definitions */
#define MinSpeed        115
#define MaxSpeed        255
#define DefaultSpeed    200
#define SpeedStep       10

void setCurrentSpeed(unsigned char speed);
void backward();
void stop();
void right();
void left();
void rotateLeft();
void rotateRight();
void forward();
void speedUp();
void slowDown();

#endif
