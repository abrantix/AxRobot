# 1 "C:\\Users\\DEFAUL~1.DES\\AppData\\Local\\Temp\\tmp1tjmkr5c"
#include <Arduino.h>
# 1 "C:/axgithub/AxRobot/lafvin_turtle_robot/RobotFirmware/src/RobotFirmware.ino"
# 11 "C:/axgithub/AxRobot/lafvin_turtle_robot/RobotFirmware/src/RobotFirmware.ino"
#include "RobotFirmware.h"

#define UserName "MeinName"

SoftwareSerial BtSerial(PIN_BT_RX, PIN_BT_TX);


#define LINE HIGH
#define NOLINE LOW

SSD1306AsciiWire display;
IRrecv irrecv(IRPin);
DIRECTION currentDirection = DIRECTION_IDLE;
MODE currentMode = MODE_REMOTE_CONTROL;
char receivedCharacter = 0;


void bluetoothRemoteControl();
void irRemoteControl();
void obstacleAvoidance();
void lineFollower();
void danceSalsa();
void funAction();
void lookLeft();
void lookRight();
void lookForward();
void sayNoNo();
void turnLeft90();
void turnRight90();
void squareDrive();
void setup();
void obstacleAvoidanceDetection();
void loop();
void action3();
void action1();
void action2();
void action4();
void action5();
void action6();
void action7();
void action8();
void action9();
void driveTurnRight();
#line 42 "C:/axgithub/AxRobot/lafvin_turtle_robot/RobotFirmware/src/RobotFirmware.ino"
void setup()
{
  Wire.begin();
  Wire.setClock(400000L);
  display.begin(&Adafruit128x64, SCREEN_I2C_ADDRESS);
  display.setFont(Verdana12_bold);
  display.clear();
  display.set2X();
  display.println(F("abrantix"));
  delay(2000);
  display.clear();
  display.set1X();
  display.setFont(Adafruit5x7);

  setServoPosition(90);
  BtSerial.begin(9600);
  pinMode(motorEnL, OUTPUT);
  pinMode(motorEnR,OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);
  pinMode(PIN_LEFT_SENSOR, INPUT);
  pinMode(PIN_CENTER_SENSOR, INPUT);
  pinMode(PIN_RIGHT_SENSOR, INPUT);
  digitalWrite(TrigPin, LOW);
  stop();
  delay(2000);
  irrecv.enableIRIn();
  while (BtSerial.available())
  {
    BtSerial.read();
  }
  display.set2X();
  display.println(F("Mir is"));
  display.println(F("gliich :)"));
}

void obstacleAvoidanceDetection()
{
  measureDistance();
  if (getDistanceFront() < 25)
  {
    stop();
    delay(700);
    backward();
    delay(300);
  }
  if (getDistanceFront() < 35)
  {
    stop();
    delay(700);
    measureDistanceLeft();
    measureDistanceRight();

    if (getDistanceLeft() > getDistanceRight())
    {
      currentDirection = DIRECTION_LEFT;
    }
    if (getDistanceLeft() <= getDistanceRight())
    {
      currentDirection = DIRECTION_RIGHT;
    }
    if (getDistanceLeft() < 10 && getDistanceRight() < 10)
    {
      currentDirection = DIRECTION_BACKWARD;
    }
  }
  else
  {
    currentDirection = DIRECTION_FORWARD;
  }
}

void loop()
{
  while (true)
  {
    if (currentMode != 0)
    {
      BtSerial.println(currentMode);
    }
    bluetoothRemoteControl();
    irRemoteControl();
    obstacleAvoidance();
    lineFollower();
    danceSalsa();

  }
}

void action3(){
  funAction();
}


void action1()
{

  setCurrentSpeed(MaxSpeed);


  forward();
  delay(1000);


  rotateRight();
  delay(1500);


  backward();
  delay(1000);


  left();
  delay(1500);


  stop();
}


void funAction()
{
  display.println(F(UserName));
  delay(3000);
  display.clear();

  display.println(F("Essen?"));
  delay(3000);
  display.clear();

  lookRight();
  delay(2000);

  lookLeft();
  delay(2000);

  lookForward();
  delay(4000);

  display.println(F("Gefunden?"));
  delay(3000);
  display.clear();

  turnLeft90();
  stop();


  forward();
  delay(1000);

  backward();
  delay(500);
  stop();

  display.set2X();
  display.println(F("Doch nicht"));
  delay(2000);
  display.clear();

  turnRight90();
  stop();

  display.println(F("Hunger"));
  delay(3000);
  display.clear();

  display.set1X();
  display.println(F("Bitcoins"));
  delay(3000);
  display.clear();

  display.set2X();
  display.println(F("Gefunden!!"));
  delay(3000);
  display.clear();
  display.set1X();

  danceSalsa();

}

void action2()
{
  currentMode = MODE_SQUARE;
  squareDrive();
}

void action4()
{
  danceSalsa();
}

void action5()
{

}

void action6()
{

}

void action7()
{

}

void action8()
{

}

void action9()
{

}

void irRemoteControl()
{
  if (irrecv.decode())
  {
    BtSerial.println(irrecv.decodedIRData.decodedRawData, HEX);
    bool enableRemoteControlMode = true;
    switch (irrecv.decodedIRData.decodedRawData)
    {
      case IR_Go:
        forward();
        break;

      case IR_Back:
        backward();
        break;

      case IR_Left:
        left();
        break;

      case IR_Right:
        right();
        break;

      case IR_Stop:
        stop();
        break;

      case IR_0:
        enableRemoteControlMode = false;
        slowDown();
        break;

      case IR_1:
        action1();
        break;

      case IR_2:
        action2();
        break;

      case IR_3:
        action3();
        break;

      case IR_4:
        currentMode = MODE_SALSA;
        enableRemoteControlMode = false;
        break;

      case IR_5:
        action5();
        break;

      case IR_6:
        enableRemoteControlMode = false;
        speedUp();
        break;

      case IR_7:
        lookLeft();
        break;

      case IR_8:
        lookForward();
        break;

      case IR_9:
        lookRight();
        break;

      case IR_Hash:
        currentMode = MODE_OBSTACLE_AVOIDANCE;
        enableRemoteControlMode = false;
        break;

      case IR_Star:
        enableRemoteControlMode = false;
        currentMode = MODE_LINE_FOLLOWER;
        BtSerial.println(F("MODE_LINE_FOLLOWER"));
        break;

      default:
        enableRemoteControlMode = false;
        break;
    }
    irrecv.resume();

    if (enableRemoteControlMode && MODE_REMOTE_CONTROL != currentMode)
    {

      stop();
      currentMode = MODE_REMOTE_CONTROL;
    }
  }
}

void bluetoothRemoteControl()
{

  if (BtSerial.available())
  {
    bool enableRemoteControlMode = true;

    receivedCharacter = BtSerial.read();


    switch (receivedCharacter)
    {
      case 'f':
        forward();
        break;

      case 'b':
        backward();
        break;

      case 'r':
        right();
        break;

      case 'l':
        left();
        break;

      case 's':
        stop();
        break;

      case '+':
        enableRemoteControlMode = false;
        speedUp();
        break;

      case '-':
        enableRemoteControlMode = false;
        slowDown();
        break;

      case 'o':

        enableRemoteControlMode = false;
        currentMode = MODE_OBSTACLE_AVOIDANCE;
        break;

      case 't':

        enableRemoteControlMode = false;
        setCurrentSpeed(MinSpeed);
        currentMode = MODE_LINE_FOLLOWER;
        break;

      case 'y':

        enableRemoteControlMode = false;
        measureDistanceRight();
        break;

      case 'x':

        enableRemoteControlMode = false;
        measureDistanceLeft();
        break;

      case 'c':

        enableRemoteControlMode = false;
        measureDistanceFront();
        break;

      case '1':
        action1();
        break;

      case '2':
        action2();
        break;

      case '3':
        action3();
        break;

      case '4':
        enableRemoteControlMode = false;
        currentMode = MODE_SALSA;
        break;

      case '5':
        action5();
        break;

      case '6':
        enableRemoteControlMode = false;
        sayNoNo();
        break;

      case '7':
        action7();
        break;

      case '8':
        action8();
        break;

      case '9':
        action9();
        break;

      default:

        break;
    }

    if (enableRemoteControlMode && MODE_REMOTE_CONTROL != currentMode)
    {

      stop();
      currentMode = MODE_REMOTE_CONTROL;
    }
  }
}

void sayNoNo()
{
  lookRight();
  lookLeft();
  lookRight();
  lookForward();
}

void lookLeft()
{
    measureDistanceLeft();
}

void lookRight()
{
    measureDistanceRight();
}

void lookForward()
{
    measureDistanceFront();
}


void lineFollower()
{
  if (MODE_LINE_FOLLOWER == currentMode)
  {
    unsigned char leftSensor = digitalRead(PIN_LEFT_SENSOR);
    unsigned char centerSensor = digitalRead(PIN_CENTER_SENSOR);
    unsigned char rightSensor = digitalRead(PIN_RIGHT_SENSOR);
    BtSerial.println(leftSensor);
    BtSerial.println(centerSensor);
    BtSerial.println(rightSensor);



    if (leftSensor == NOLINE && centerSensor == LINE && rightSensor == NOLINE)
      currentDirection = DIRECTION_FORWARD;


    else if (leftSensor == LINE && centerSensor == NOLINE && rightSensor == NOLINE)
      currentDirection = DIRECTION_RIGHT;


    else if (leftSensor == NOLINE && centerSensor == NOLINE && rightSensor == LINE)
      currentDirection = DIRECTION_LEFT;


    else if (leftSensor == LINE && centerSensor == LINE && rightSensor == LINE)
      currentDirection = DIRECTION_FORWARD;


    else if (leftSensor == LINE && centerSensor == LINE && rightSensor == NOLINE)
      currentDirection = DIRECTION_RIGHT;


    else if (leftSensor == NOLINE && centerSensor == LINE && rightSensor == LINE)
      currentDirection = DIRECTION_LEFT;

    setCurrentSpeed(MinSpeed);

    switch (currentDirection)
    {
      case DIRECTION_FORWARD:
        BtSerial.println(F("DIRECTION_FORWARD"));
        forward();
        break;

      case DIRECTION_RIGHT:
        BtSerial.println(F("DIRECTION_RIGHT"));
        right();
        break;

      case DIRECTION_LEFT:
        BtSerial.println(F("DIRECTION_LEFT"));
        left();
        break;

      case DIRECTION_ROTATE_RIGHT:
        BtSerial.println(F("DIRECTION_ROTATE_RIGHT"));
        rotateRight();
        break;

      case DIRECTION_ROTATE_LEFT:
        BtSerial.println(F("DIRECTION_ROTATE_LEFT"));
        rotateLeft();
        break;
      case DIRECTION_IDLE:
        BtSerial.println(F("DIRECTION_IDLE"));
        stop();
        break;
      default:
        BtSerial.println(F("DIRECTION_STOP"));
        backward();
        delay(100);
        break;
    }
  }
}

void obstacleAvoidance()
{
  if (MODE_OBSTACLE_AVOIDANCE == currentMode)
  {
    setCurrentSpeed(DefaultSpeed);
    obstacleAvoidanceDetection();

    switch (currentDirection)
    {
      case DIRECTION_FORWARD:
        forward();
        break;

      case DIRECTION_BACKWARD:
        backward();
        delay(200);
        break;

      case DIRECTION_ROTATE_RIGHT:
      case DIRECTION_RIGHT:
        rotateRight();
        delay(100);
        break;

      case DIRECTION_LEFT:
      case DIRECTION_ROTATE_LEFT:
        rotateLeft();
        delay(100);
        break;

      default:
        break;
    }
  }
}

void danceSalsa()
{
  if (MODE_SALSA == currentMode)
  {
    forward();
    delay(1000);
    getDistanceRight();
    backward();
    delay(1000);
    getDistanceFront();
    delay(1000);
    getDistanceLeft();
    forward();
    delay(1000);
    rotateRight();
    sayNoNo();
    delay(1000);
    rotateLeft();
    delay(1000);
    sayNoNo();
    stop();
  }
}

void turnRight90()
{
    right();
    delay(700);
}

void turnLeft90()
{
    left();
    delay(700);
}

void driveTurnRight()
{
    forward();
    delay(1000);
    turnRight90();
}

void squareDrive()
{
  if (MODE_SQUARE == currentMode)
  {
   driveTurnRight();
   driveTurnRight();
   driveTurnRight();
   driveTurnRight();
  }
}