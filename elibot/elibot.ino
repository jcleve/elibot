
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

#define    STX          0x02
#define    ETX          0x03
#define    ledPin       13
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "xxxx";                          // message to Android device

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4
#define IDLE_MAX 20

int vehicleDirection = DIRECTION_STOP;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);           // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(150);
  myMotor1->run(FORWARD);
  // turn on motor
  myMotor1->run(RELEASE);

  myMotor2->setSpeed(150);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor2->run(RELEASE);

  myMotor3->setSpeed(150);
  myMotor3->run(FORWARD);
  // turn on motor
  myMotor3->run(RELEASE);

  myMotor4->setSpeed(150);
  myMotor4->run(FORWARD);
  // turn on motor
  myMotor4->run(RELEASE);

  Serial.println("setup complete");
}

void loop() {
  if (Serial1.available())  {                          // data received from smartphone
    delay(2);
    cmd[0] =  Serial1.read();
    if (cmd[0] == STX)  {
      int i = 1;
      while (Serial1.available())  {
        delay(1);
        cmd[i] = Serial1.read();
        if (cmd[i] > 127 || i > 7)                 break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))   break; // Button or Joystick data
        i++;
      }
      //if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      //else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
      //Serial.println(i);
      if (i == 7)          getJoystickState(cmd);
    }
  }

  uint8_t i;
}

void getJoystickState(byte data[8])
{
  Serial.println("getJoystickState");
  int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)     return; // commmunication error

  bool pivot = false;
  int speedY = 0;
  if (joyY > 0) // move forward
  {
    speedY = map(joyY, 0, 100, 0, 255);
    vehicleDirection = DIRECTION_FORWARD;
  }
  else if (joyY < 0) //move backward
  {
    speedY = map(joyY, -100, 0, 255, 0);
    vehicleDirection = DIRECTION_REVERSE;
  }

  int speedX_R = (map(joyX, -100, 100, speedY, 0));
  int speedX_L = (map(joyX, -100, 100, 0, speedY));

//  Serial.print("joyY: ");
//  Serial.println(joyY);
//
//  Serial.print("joyX: ");
//  Serial.println(joyX);
//
//  Serial.print("speedY: ");
//  Serial.println(speedY);
//
//  Serial.print("speedX_R: ");
//  Serial.println(speedX_R);
//
//  Serial.print("speedX_L: ");
//  Serial.println(speedX_L);

  if (joyY == 0 && joyX == 0)     // no controller input so stop motors
  {
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
  }
  else if (abs(joyY) < 20 && abs(joyX) > 20) // pivot
  {
    pivot = true;
    int pivotSpeed = 0;
    pivotSpeed = map(abs(joyX), 0, 100, 0, 255);
    if (joyX > 0) // pivot right
    {      
      myMotor1->run(FORWARD);
      myMotor2->run(FORWARD);
      myMotor3->run(FORWARD);
      myMotor4->run(FORWARD);
    }
    else
    {
      myMotor1->run(BACKWARD);
      myMotor2->run(BACKWARD);
      myMotor3->run(BACKWARD);
      myMotor4->run(BACKWARD);
    }

    myMotor1->setSpeed(pivotSpeed);
    myMotor2->setSpeed(pivotSpeed);
    myMotor3->setSpeed(pivotSpeed);
    myMotor4->setSpeed(pivotSpeed);

  }
  else if (joyY > 0) // go forward
  {
    myMotor1->run(FORWARD);
    myMotor2->run(BACKWARD);
    myMotor3->run(BACKWARD);
    myMotor4->run(FORWARD);
  }
  else if (joyY < 0) // go backward
  {
    myMotor1->run(BACKWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(BACKWARD);
  }

  if (!pivot)
  {
    myMotor1->setSpeed(speedX_L);
    myMotor2->setSpeed(speedX_R);
    myMotor3->setSpeed(speedX_R);
    myMotor4->setSpeed(speedX_L);
  }
}

