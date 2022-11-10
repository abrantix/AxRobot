/*********************************************************************************
    Abrantix Arduino firmware for LAFVIN turtle robot
    Parts of this code are based on keyestudio sample code

   Main features:
   - Obstacle avoidance
   - Remote control over bluetooth dongle by using "axRobot" android app
   - Remote control by IR remote.
   - Display some text on a small I2C display
 *********************************************************************************/
#include "RobotFirmware.h"

#define UserName "MeinName"

SoftwareSerial BtSerial(PIN_BT_RX, PIN_BT_TX);      //RX, TX

/* Line follower: Invert these values depending of using ba right line on a dark background or vice versa */
#define LINE HIGH
#define NOLINE LOW

SSD1306AsciiWire display;
IRrecv irrecv(IRPin);
DIRECTION currentDirection = DIRECTION_IDLE;
MODE currentMode = MODE_REMOTE_CONTROL;
char receivedCharacter = 0;

/* forward declarations */
void bluetoothRemoteControl();
void irRemoteControl();
void obstacleAvoidance();
void lineFollower();
void danceSalsa();
void lookLeft();
void lookRight();
void lookForward();
void sayNoNo();
void turnLeft90();
void turnRight90();
void squareDrive();

void setup()
{
  Serial.begin(115200);
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
  BtSerial.begin(9600);    //bluetooth module
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
  irrecv.enableIRIn(); // Start the ir receiver
  while (BtSerial.available())
  {
    BtSerial.read();
  }
  display.set2X();
  display.println(F(UserName));
}

void obstacleAvoidanceDetection()        // measure 3 angles (0.90.179)
{
  measureDistance();            // read the distance ahead
  if (getDistanceFront() < 25)        // if distance ahead is <20cm
  {
    stop();               // clear data
    delay(700);
    backward();                // move backward for 0.2S
    delay(300);
  }
  if (getDistanceFront() < 35)        // if distance ahead is <30cm
  {
    stop();
    delay(700);             // clear data
    measureDistanceLeft();            // read distance on the left
    measureDistanceRight();            // read distance on the right

    if (getDistanceLeft() > getDistanceRight())  // if distance on the left is >distance on the right
    {
      currentDirection = DIRECTION_LEFT;      // move to the L
    }
    if (getDistanceLeft() <= getDistanceRight())  // if distance on the left is <= distance on the right
    {
      currentDirection = DIRECTION_RIGHT;      // move to the right
    }
    if (getDistanceLeft() < 10 && getDistanceRight() < 10)   // if distance on left and right are both <10cm
    {
      currentDirection = DIRECTION_BACKWARD;      // move backward
    }
  }
  else                      // if distance ahead is > x cm
  {
    currentDirection = DIRECTION_FORWARD;        // move forward
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
}
  
//perform a simple dance
void action1()
{
  //Set speed to maximum
  setCurrentSpeed(MaxSpeed);

  //go forward for 1 second
  turnRight90();
  backward();
  delay(2000);

  display.println(F("Ich habe"));
  display.println(F("einparkiert!"));
  //stop motors
  stop();
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
    Serial.print(F("IR command received:"));
    Serial.print(irrecv.decodedIRData.command, HEX);
    Serial.print(' ');
    Serial.println(irrecv.decodedIRData.command >> 8, HEX);
    bool enableRemoteControlMode = true;
    switch (irrecv.decodedIRData.command)
    {
      case IR_Go:
        Serial.println(F("IR_Go"));
        forward();
        break;

      case IR_Back:
        Serial.println(F("IR_Back"));
        backward();
        break;

      case IR_Left:
        Serial.println(F("IR_Left"));
        left();
        break;

      case IR_Right:
        Serial.println(F("IR_Right"));
        right();
        break;

      case IR_Stop:
      Serial.println(F("IR_Stop"));
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
    irrecv.resume(); // Receive the next value

    if (enableRemoteControlMode && MODE_REMOTE_CONTROL != currentMode)
    {
      //switch into remote control mode
      stop();
      currentMode = MODE_REMOTE_CONTROL;
    }
  }
}

void bluetoothRemoteControl()
{
  //check if serial data from bluetooth is available
  if (BtSerial.available())
  {
    bool enableRemoteControlMode = true;

    receivedCharacter = BtSerial.read();

    //perform action according received character
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
        /* Enter obstacle avoidance mode */
        enableRemoteControlMode = false;
        currentMode = MODE_OBSTACLE_AVOIDANCE;
        break;

      case 't':
        /* Enter line tracker mode */
        enableRemoteControlMode = false;
        setCurrentSpeed(MinSpeed);
        currentMode = MODE_LINE_FOLLOWER;
        break;

      case 'y':
        /* turn sensor to the right */
        enableRemoteControlMode = false;
        measureDistanceRight();
        break;

      case 'x':
        /* turn sensor to the left */
        enableRemoteControlMode = false;
        measureDistanceLeft();
        break;

      case 'c':
        /* turn sensor to the front */
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
        /* Unknown command */
        break;
    }

    if (enableRemoteControlMode && MODE_REMOTE_CONTROL != currentMode)
    {
      //switch into remote control mode
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

    // calculate direction
    // if only center sensor detects line, go straight ahead
    if (leftSensor == NOLINE && centerSensor == LINE && rightSensor == NOLINE)
      currentDirection = DIRECTION_FORWARD;

    // if only left sensor detects line, turn left
    else if (leftSensor == LINE && centerSensor == NOLINE && rightSensor == NOLINE)
      currentDirection = DIRECTION_RIGHT;

    // if only right sensor detects line, turn right
    else if (leftSensor == NOLINE && centerSensor == NOLINE  && rightSensor == LINE)
      currentDirection = DIRECTION_LEFT;

    // if all sensor detects line: we are either finished or out of control and will stop
    else if (leftSensor == LINE && centerSensor == LINE && rightSensor == LINE)
      currentDirection = DIRECTION_FORWARD;

    // if left and center sensor detect line, must be 90° turn to the left
    else if (leftSensor == LINE && centerSensor == LINE && rightSensor == NOLINE)
      currentDirection = DIRECTION_RIGHT;

    // if right and center sensor detect line, must be 90° turn to the right
    else if (leftSensor == NOLINE && centerSensor == LINE && rightSensor == LINE)
      currentDirection = DIRECTION_LEFT;

    setCurrentSpeed(MinSpeed);

    switch (currentDirection)
    {
      case DIRECTION_FORWARD:
        BtSerial.println(F("DIRECTION_FORWARD"));
        forward();       // move forward
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
    obstacleAvoidanceDetection();        // measure the distances and determine which direction to move

    switch (currentDirection)
    {
      case DIRECTION_FORWARD:
        forward();       // move forward
        break;

      case DIRECTION_BACKWARD:
        backward();
        delay(200);               //  go backward
        break;

      case DIRECTION_ROTATE_RIGHT:
      case DIRECTION_RIGHT:
        rotateRight();
        delay(100);                 // turn right
        break;

      case DIRECTION_LEFT:
      case DIRECTION_ROTATE_LEFT:
        rotateLeft();
        delay(100);                  // turn left
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

