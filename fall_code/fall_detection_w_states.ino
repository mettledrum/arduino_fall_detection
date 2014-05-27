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

// fall detection variables
boolean standing = false;
boolean down = false;
boolean falling = false;

const float GRAV = 9.80665f;
// max voltage observed when x axis parallel to gravity
const float MAX_VOLT = 61200.0f;
// approx 0.00060535f;
const float VOLT_TO_MS2 = GRAV/MAX_VOLT;

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  // only really necessary if using device 1
  MPU.selectDevice(DEVICE_TO_USE);
  // start the MPU
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);
}

void loop()
{  
  // only needed if device has changed since init but good form anyway
  MPU.selectDevice(DEVICE_TO_USE);
  // get the latest data if ready yet                          
  if (MPU.read()) {
    // how bent over, 90 deg is straight up
    float pitch = MPU.m_dmpEulerPose[0] * (180/M_PI);
    // how bent to side, 0 deg is straight up
    float roll = MPU.m_dmpEulerPose[1] * (180/M_PI);
    // regular is approx 9.8
    float x_accel = MPU.m_calAccel[1] * VOLT_TO_MS2;

    // current stance angles
    Serial.print(pitch);
    Serial.print("\t\t");
    Serial.print(roll);
    Serial.print("\t\t");

    // stance wearer is in, updates bools
    if (pitch > 70 && pitch < 110 && abs(roll) < 20) {
      Serial.print("standing\t\t");
      standing = true;
      down = false;
    }
    else if (pitch < 20 || pitch > 160 || abs(roll) > 70) {
      Serial.print("down\t\t");
      down = true;
      standing = false;
    }
    else {
      // do nothing
      Serial.print("\t\t");
      down = false;
      standing = false;
    }

    // while standing, did they start falling?
    if (abs(x_accel) < 2.0f && standing == true) {
      Serial.print("falling");
      falling = true;
    }
    else {
      falling = false;
    }

    // keep track of the timing of the fall

    // if down is reached within a certain amount of time,
    //  then indicate that there has been a fall!

    Serial.println();
  }
}