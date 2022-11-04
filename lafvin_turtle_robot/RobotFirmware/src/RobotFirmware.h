/* File RobotFirmware.h */

#ifndef __ROBOT_FIRMWARE_H
#define __ROBOT_FIRMWARE_H

#include <Arduino.h>
#include "IRremote.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#include "Pinout.h"
#include "IrCodes.h"
#include "Drive.h"
#include "Distance.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_I2C_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

typedef enum _MODE {
  MODE_REMOTE_CONTROL,
  MODE_OBSTACLE_AVOIDANCE,
  MODE_SALSA,
  MODE_SQUARE,
  MODE_LINE_FOLLOWER
} MODE;

//Obstacle avoidance
typedef enum _DIRECTION {
  DIRECTION_IDLE,
  DIRECTION_FORWARD,
  DIRECTION_BACKWARD,
  DIRECTION_LEFT,
  DIRECTION_RIGHT,
  DIRECTION_ROTATE_LEFT,
  DIRECTION_ROTATE_RIGHT,
} DIRECTION;

#endif
