/* File Pinout.h */

#ifndef __PINOUT_H
#define __PINOUT_H

/* Pin usage definitions */

// I2C (Used for Display)
// ARDUINO UNO SDA:       A4  //Connect to Display "SDA"
// ARDUINO UNO SCL:       A5  //Connect to Display "SCK"

//Servo
#define ServoPin 7

//Motor 
#define motorEnR          5
#define motorEnL          6
#define motorL1           8
#define motorL2           9
#define motorR1           10
#define motorR2           A2

//Line sensors
#define PIN_LEFT_SENSOR   2
#define PIN_CENTER_SENSOR 4
#define PIN_RIGHT_SENSOR  3

//Distance sensor HC-SR04
#define EchoPin           12   
#define TrigPin           13    

//IR remote control
#define IRPin             A0

//Bluetooth Module
#define PIN_BT_TX         A3  //(Connect to BT Module RX with a Resistor)
#define PIN_BT_RX         11  //(Connect to BT Module TX)

#endif