/*********************************************************************************
    Abrantix Arduino firmware for LAFVIN turtle robot
    Parts of this code are based on keyestudio sample code

   Main features:
   - Obstacle avoidance
   - Remote control over bluetooth dongle by using "axRobot" android app
   - Remote control by IR remote.
   - Display some text on a small I2C display
 *********************************************************************************/
// Hint: IRremote, AltSoftSerial and SSD1306Ascii libraries have to be installed (see Sketch -> Include Library -> Manage libraries...)
#include <IRremote.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"


#define UserName "MeinName"

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



/* Speed definitions */
#define MinSpeed        115
#define MaxSpeed        255
#define DefaultSpeed    200
#define SpeedStep       10


SoftwareSerial BtSerial(PIN_BT_RX, PIN_BT_TX);      //RX, TX

/* slightly modify these values if robot doesn't run straight on */
#define leftSpeedCorrection 0
#define rightSpeedCorrection 0

/* Inverte these values depending of using ba right line on a dark background or vice versa */
#define LINE HIGH
#define NOLINE LOW

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_I2C_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
SSD1306AsciiWire display;

IRrecv irrecv(IRPin);
decode_results irResults;

#define IR_Go      0x00ff629d
#define IR_Back    0x00ffa857
#define IR_Left    0x00ff22dd
#define IR_Right   0x00ffc23d
#define IR_Stop    0x00ff02fd
#define IR_Hash    0x00ff52ad
#define IR_Star    0x00ff42bd
#define IR_0       0x00ff4ab5
#define IR_1       0x00ff6897
#define IR_2       0x00ff9867
#define IR_3       0x00ffb04f
#define IR_4       0x00ff30cf
#define IR_5       0x00ff18e7
#define IR_6       0x00ff7a85
#define IR_7       0x00ff10ef
#define IR_8       0x00ff38c7
#define IR_9       0x00ff5aa5

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

int fDistance = 0;      // forward distance
int rDistance = 0;      // right distance
int lDistance = 0;      // left distance
DIRECTION currentDirection = DIRECTION_IDLE;

MODE currentMode = MODE_REMOTE_CONTROL;
unsigned long lastDistanceMeasureTime = 0;

#define delay_time  500 // settling time in ms after steering servo motor moving

int  currentSpeed = DefaultSpeed;

Servo myservo;        // set myservo
int currentServoPosition = -1;

char receivedCharacter = 0;

void setCurrentSpeed()
{
  setSpeed(currentSpeed);
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
  setCurrentSpeed();
}
void stop()         // stop
{
  setSpeed(0);
  delay(300);
}
void right()        // turn right (single wheel)
{
  digitalWrite(motorR1, LOW); // wheel on the right moves backward
  digitalWrite(motorR2, HIGH); // wheel on the right moves backward
  digitalWrite(motorL1, LOW); // wheel on the right does not move
  digitalWrite(motorL2, LOW); // wheel on the right does not move
  setCurrentSpeed();
}
void left()         // turn left (single wheel)
{
  digitalWrite(motorR1, LOW); // wheel on the left does not move
  digitalWrite(motorR2, LOW); // wheel on the left does not move
  digitalWrite(motorL1, LOW); // wheel on the left moves backward
  digitalWrite(motorL2, HIGH); // wheel on the left moves backward
  setCurrentSpeed();
}

void rotateLeft()        // turn left (single wheel)
{
  digitalWrite(motorR1, LOW); // wheel on the right moves backward
  digitalWrite(motorR2, HIGH); // wheel on the right moves backward
  digitalWrite(motorL1, HIGH); // wheel on the right does not move
  digitalWrite(motorL2, LOW); // wheel on the right does not move
  setCurrentSpeed();
}
void rotateRight()         // turn right (single wheel)
{
  digitalWrite(motorR1, HIGH); // wheel on the left does not move
  digitalWrite(motorR2, LOW); // wheel on the left does not move
  digitalWrite(motorL1, LOW); // wheel on the left moves backward
  digitalWrite(motorL2, HIGH); // wheel on the left moves backward
  setCurrentSpeed();
}

void forward()          // move forward
{
  digitalWrite(motorR1, LOW);  // left wheel moves forward
  digitalWrite(motorR2, HIGH);
  digitalWrite(motorL1, LOW);   // right wheel moves forward
  digitalWrite(motorL2, HIGH);
  setCurrentSpeed();
}

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
  display.println(F("Mir is"));
  display.println(F("gliich :)"));
}

void obstacleAvoidanceDetection()        // measure 3 angles (0.90.179)
{
  ask_pin_F();            // read the distance ahead
  if (fDistance < 25)        // if distance ahead is <20cm
  {
    stop();               // clear data
    delay(700);
    backward();                // move backward for 0.2S
    delay(300);
  }
  if (fDistance < 35)        // if distance ahead is <30cm
  {
    stop();
    delay(700);             // clear data
    ask_pin_L();            // read distance on the left
    delay(delay_time);      // stabilizing time for servo motor
    ask_pin_R();            // read distance on the right
    delay(delay_time);      // stabilizing time for servo motor

    if (lDistance > rDistance)  // if distance on the left is >distance on the right
    {
      currentDirection = DIRECTION_LEFT;      // move to the L
    }
    if (lDistance <= rDistance)  // if distance on the left is <= distance on the right
    {
      currentDirection = DIRECTION_RIGHT;      // move to the right
    }
    if (lDistance < 10 && rDistance < 10)   // if distance on left and right are both <10cm
    {
      currentDirection = DIRECTION_BACKWARD;      // move backward
    }
  }
  else                      // if distance ahead is > x cm
  {
    currentDirection = DIRECTION_FORWARD;        // move forward
  }
}

//returns distance from HC-SR04 ultrasonic module in cm
float getDistance()
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

void setServoPosition(int position)
{
  if(position != currentServoPosition)
  {
    myservo.attach(ServoPin);
    myservo.write(position);
    delay(delay_time);
    myservo.detach();
    currentServoPosition = position;
  }
}

void ask_pin_F()   // measure the distance ahead
{
  setServoPosition(90);
  float distance = getDistance();
  if (0 < distance)
  {
    fDistance = distance;
  }
}
void ask_pin_L()   // measure distance on the left
{
  setServoPosition(10);
  float distance = getDistance();
  if (0 < distance)
  {
    lDistance = distance;
  }
}
void ask_pin_R()   //  measure distance on the right
{
  setServoPosition(170);
  float distance = getDistance();
  if (0 < distance)
  {
    rDistance = distance;
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
    BluetoothRemoteControl();
    IrRemoteControl();
    ObstacleAvoidance();
    LineFollower();
    DanceSalsa();

  }
}

void speedUp()
{
  currentSpeed = currentSpeed + SpeedStep;
  if (currentSpeed > MaxSpeed)
  {
    currentSpeed = MaxSpeed;
  }
  setCurrentSpeed();
}

void slowDown()
{
  currentSpeed = currentSpeed - SpeedStep;
  if (currentSpeed < MinSpeed)
  {
    currentSpeed = MinSpeed;
  }
  setCurrentSpeed();
}

void action3(){
  funAction();
}
  
//perform a simple dance
void action1()
{
  //Set speed to maximum
  currentSpeed = MaxSpeed;
  setCurrentSpeed();

  //go forward for 1 second
  forward();
  delay(1000);

  //rotate right for 1.5 second
  rotateRight();
  delay(1500);

  //run backward for 1 second
  backward();
  delay(1000);

  //turn left for 1 second
  left();
  delay(1500);

  //stop motors
  stop();
}

//perform a simple dance
void funAction()
{
  display.println(F(UserName));
  delay(3000);
  display.clear();

  display.println(F("Essen?"));
  delay(3000);
  display.clear();
  
  LookRight();
  delay(2000);
  
  LookLeft(); 
  delay(2000);

  LookForward(); 
  delay(4000);

  display.println(F("Gefunden"));
  delay(3000);
  display.clear();
  
  TurnLeft90();
  stop();
  
  //go forward for 1 second
  forward();
  delay(1000);

  backward();
  delay(500);
  stop();
  
  display.set2X();
  display.println(F("Doch nicht"));
  delay(2000);
  display.clear();

  TurnRight90();
  stop();

  display.println(F("Hunger"));
  delay(3000);
  display.clear();

  display.set1X();
  display.println(F("Bitcoins"));
  delay(3000);
  display.clear();
  
  display.set2X();
  display.println(F("Gefunden"));
  delay(3000);
  display.clear();
  display.set1X();

  DanceSalsa();
  
}

void action2()
{
  currentMode = MODE_SQUARE;
  SquareDrive();
}
void action3Old()
{
  //Set speed to maximum
  currentSpeed = MaxSpeed;
  setCurrentSpeed();

  forward();
  delay(500);
  stop();

  while (true)
  {
    delay(60);
    ask_pin_F();            // read the distance ahead
    if (fDistance < 10)
    {
      break;
    }
  }

  backward();
  delay(2000);
  rotateRight();
  delay(2000);
  stop();
}

void action4()
{
  DanceSalsa();
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

void IrRemoteControl()
{
  if (irrecv.decode(&irResults))
  {
    BtSerial.println(irResults.value, HEX);
    bool enableRemoteControlMode = true;
    switch (irResults.value)
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
        LookLeft();
        break;

      case IR_8:
        LookForward();
        break;

      case IR_9:
        LookRight();
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

void BluetoothRemoteControl()
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
        currentSpeed = MinSpeed;
        currentMode = MODE_LINE_FOLLOWER;
        break;

      case 'y':
        /* turn sensor to the right */
        enableRemoteControlMode = false;
        ask_pin_R();
        break;

      case 'x':
        /* turn sensor to the left */
        enableRemoteControlMode = false;
        ask_pin_L();
        break;

      case 'c':
        /* turn sensor to the front */
        enableRemoteControlMode = false;
        ask_pin_F();
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
        SayNoNo();
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

void SayNoNo()
{
  LookRight();
  LookLeft();
  LookRight();
  LookForward();
}

void LookLeft()
{
    ask_pin_L();
}

void LookRight()
{
    ask_pin_R();
}

void LookForward()
{
    ask_pin_F();
}


void LineFollower()
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


    currentSpeed = MinSpeed;
    setCurrentSpeed();

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

void ObstacleAvoidance()
{
  if (MODE_OBSTACLE_AVOIDANCE == currentMode)
  {
    currentSpeed = DefaultSpeed;
    setCurrentSpeed();
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

void DanceSalsa()
{
  if (MODE_SALSA == currentMode)
  {
    forward();
    delay(1000);
    ask_pin_R();
    backward();
    delay(1000);
    ask_pin_F();
    delay(1000);
    ask_pin_L();
    forward();
    delay(1000);
    rotateRight();
    SayNoNo();
    delay(1000);
    rotateLeft();
    delay(1000);
    SayNoNo();
    stop();
  }
}

void SquareDrive()
{
  if (MODE_SQUARE == currentMode)
  {
   DriveTurnRight();
   DriveTurnRight();
   DriveTurnRight();
   DriveTurnRight();
  }
}

void DriveTurnRight(){
    forward();
    delay(1000);
    TurnRight90();                
}
void TurnRight90(){
    right();
    delay(700);                
}

void TurnLeft90(){
    left();
    delay(700);                
}
