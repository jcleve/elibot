
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

// Differential Steering Joystick Algorithm
// ========================================
//   by Calvin Hass
//   http://www.impulseadventure.com/elec/
//
// Converts a single dual-axis joystick into a differential
// drive motor control, with support for both drive, turn
// and pivot operations.
//

// INPUTS
int     nJoyX;              // Joystick X input                     (-128..+127)
int     nJoyY;              // Joystick Y input                     (-128..+127)

// OUTPUTS
int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

// CONFIG
// - fPivYLimt  : The threshold at which the pivot action starts
//                This threshold is measured in units on the Y-axis
//                away from the X-axis (Y=0). A greater value will assign
//                more of the joystick's range to pivot actions.
//                Allowable range: (0..+127)

// TEMP VARIABLES
float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
int     nPivSpeed;      // Pivot Speed                          (-128..+127)
float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
float fPivYLimit = 50.0;

void getJoystickState(byte data[8])
{
  Serial.println("getJoystickState");
  int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  Serial.print("joyY: ");
  Serial.println(joyY);

  Serial.print("joyX: ");
  Serial.println(joyX);

  nJoyX = map(joyX, -100, 100, -128, 127);
  nJoyY = map(joyY, -100, 100, -128, 127);

  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX >= 0) ? 127.0 : (127.0 + nJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY / 128.0;
  nMotPremixR = nMotPremixR * nJoyY / 128.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  Serial.print("fPivScale: ");
  Serial.println(fPivScale);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  //nMotMixL = map(abs(nMotMixL), 0, 127, 0, 255);
  //nMotMixR = map(abs(nMotMixR), 0, 127, 0, 255);
  
  Serial.print("nMotMixL: ");
  Serial.println(nMotMixL);

  Serial.print("nMotMixR: ");
  Serial.println(nMotMixR);

  int speedL = map(abs(nMotMixL), 0, 127, 0, 255);
  int speedR = map(abs(nMotMixR), 0, 127, 0, 255);

  if(nMotMixL > 0) //forward
  {
    myMotor1->run(FORWARD);
    myMotor4->run(FORWARD);   
  }
  else if(nMotMixL < 0) //reverse
  {
    myMotor1->run(BACKWARD);
    myMotor4->run(BACKWARD);
  }
  else
  {
    myMotor1->run(RELEASE);
    myMotor4->run(RELEASE);
  }

  if(nMotMixR > 0) // forward
  {
    myMotor2->run(BACKWARD);
    myMotor3->run(BACKWARD);
  }
  else if(nMotMixR < 0) //reverse
  {
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
  }
  else
  {
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
  }

  myMotor1->setSpeed(speedL);
  myMotor2->setSpeed(speedR);
  myMotor3->setSpeed(speedR);
  myMotor4->setSpeed(speedL);

}









