////////////////////////////////////////////////////////////////////////////
//
//  This file is part of MPU9150Lib
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <MPU9150Lib.h>
#include <MPUQuaternion.h>
#include <MPUVector3.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69
#define  DEVICE_TO_USE    0

MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output
#define MPU_UPDATE_RATE  (40)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE
#define MAG_UPDATE_RATE  (25)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
#define MPU_LPF_RATE   40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  9600

//falling variables ////////////////////////////////////////////////////////
// acceleration values
float Ax, Ay, Az;
float Anorm;

// timing thresholds
unsigned long cur_time, old_time, time_diff;
const unsigned long FALLING_TIME = 3000;
const unsigned long DOWN_TIME = 6000;
const unsigned long RESET_TIME = 10000;

// acceleration measurements
const float POTENTIAL_HIT_THRESH_MAX = 22;
const float POTENTIAL_HIT_THRESH_MIN = 3;
const float FALLING_RANGE = 25;
float min_Anorm, max_Anorm, range_Anorm;

// states wearer is in
boolean standing, falling, down, fell, help;

// callibration vars
const float GRAV = 9.80665f;
// max voltage observed when x axis parallel to gravity
const float MAX_VOLT = 61200.0f;
// measurments were taken using: 0.00060535f;
float VOLT_TO_MS2 = GRAV/MAX_VOLT;

// gyro values 
float pitch, roll;

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  // only really necessary if using device 1
  MPU.selectDevice(DEVICE_TO_USE);
  // start the MPU
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   

  // set defaults for states and timers
  standing = false;
  falling = false;
  down = false;
  fell = false;
  help = false;

  old_time = millis();
}

void loop()
{  
  MPU.selectDevice(DEVICE_TO_USE);                         // only needed if device has changed since init but good form anyway
  if (MPU.read()) {                                        // get the latest data if ready yet

    // get accel vals
    Ax = MPU.m_calAccel[0] * 0.00060535f;
    Ay = MPU.m_calAccel[1] * 0.00060535f;
    Az = MPU.m_calAccel[2] * 0.00060535f;
    Anorm = sqrt(Ax*Ax + Ay*Ay + Az*Az);

    // get gyro vals
    // how bent over, (90 deg is straight up)
    pitch = MPU.m_dmpEulerPose[0] * (180/M_PI);
    // how bent to side, (0 deg is straight up)
    roll = MPU.m_dmpEulerPose[1] * (180/M_PI);

    // print raw data from acceleration
    Serial.print(Anorm);
    Serial.print("\t\t");

    // stance wearer is in, updates bools
    if (pitch > 70 && pitch < 110 && abs(roll) < 20) {
      Serial.print("standing\t\t");
      standing = true;
      down = false;
    }
    else if (pitch < 20 || pitch > 160 || abs(roll) > 70) {
      Serial.print("down    \t\t");
      down = true;
      standing = false;
    }
    else {
      // do nothing
      Serial.print("        \t\t");
      down = false;
      standing = false;
    }

    // acceleration check, has there been something more jarring than stair walking?
    if ((Anorm <= POTENTIAL_HIT_THRESH_MIN || Anorm >= POTENTIAL_HIT_THRESH_MAX) && !falling && !fell) {
      falling = true;
      // reset time
      old_time = cur_time;

      // initalize min == max
      min_Anorm = Anorm;
      max_Anorm = Anorm;
    }

    // TIMING update
    cur_time = millis();
    time_diff = cur_time - old_time;

    // calculate the max/mins if falling state and time threshold hasn't been reached
    if (falling && time_diff < FALLING_TIME) {
      Serial.print("falling timer\t\t");

      // max/mins range check
      if (Anorm < min_Anorm) {
        min_Anorm = Anorm;
      }
      if (Anorm > max_Anorm) {
        max_Anorm = Anorm;
      }

      // check range, if fell, might need help!!!
      range_Anorm = max_Anorm - min_Anorm;
      if (down && range_Anorm > FALLING_RANGE) {
        fell = true;
        falling = false;

        // reset timer
        old_time = cur_time;
      }
    }
    else {
      falling = false;    
      Serial.print("             \t\t");
    }

    // display help timer
    if (fell) {
      Serial.print("fell, help timer\t\t");
    }
    else {
      Serial.print("                \t\t");
    }

    // help! checking to see if they're down long enough
    if (down && fell && time_diff > DOWN_TIME) {
      // they need help!
      help = true;
    }

    // if they recovered by standing back up    
    if (fell && standing) {
      fell = false;
    }

    // display helping
    if (help) {
      Serial.print("help!!!!!!!!\t\t");
    }

    // reset states for another fall detection
    if (help && time_diff > RESET_TIME) {
      help = false;
      falling = false;
      fell = false;
      Serial.print("RESET TIMER\t\t");
    }

    Serial.println();
  }
}