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

// vars for fall
#define AVG_ARR_LEN 25                                      // size of array to avg
unsigned long INTERVAL = 20;                                // ms for checking angular velocity
float ang_vels_roll[AVG_ARR_LEN];                           // holds degree differences for roll
float ang_vels_pitch[AVG_ARR_LEN];                          // holds degree differences for pitch
unsigned long old_time, cur_time, delta_time;               // used to hold millis()
float old_roll, cur_roll, delta_roll;
float old_pitch, cur_pitch, delta_pitch;
unsigned int IDX = 0;                                       // index for degree differences circular arrays

// averages values in arrays
float avg(float arr[]);

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  MPU.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU

  // setup first time variables
  old_time = millis();
  old_roll = 0;
  old_pitch = 0;
}

void loop()
{  
  MPU.selectDevice(DEVICE_TO_USE);                         // only needed if device has changed since init but good form anyway
  if (MPU.read()) {                                        // get the latest data if ready yet

    // get latest vals from MPU
    cur_pitch = abs(MPU.m_dmpEulerPose[0]);
    cur_roll = abs(MPU.m_dmpEulerPose[1]);

    // calculate abs() of differences
    delta_roll = abs(cur_roll - old_roll);
    delta_pitch = abs(cur_pitch - old_pitch);

    // update
    old_pitch = cur_pitch;
    old_roll = cur_roll;

    // fill circular arrays
    ang_vels_pitch[IDX] = delta_pitch;
    ang_vels_roll[IDX] = delta_roll;
    IDX = (IDX + 1) % AVG_ARR_LEN;
  }

  // if interval has passed, get average of roll and pitch
  cur_time = millis();
  delta_time = cur_time - old_time;
  if (delta_time > INTERVAL) {

    // update time
    old_time = cur_time; 

    float pitch = avg(ang_vels_pitch);
    float roll = avg(ang_vels_roll);

    Serial.print((roll/delta_time)*1000*(180/M_PI));
    Serial.print("\t\t");
    Serial.println((pitch/delta_time)*1000*(180/M_PI));
  }
}

// get average roll/pitch per second
float avg(float arr[]) {
  float sum = 0;
  for(int i=0;i<AVG_ARR_LEN;++i) {
    sum += arr[i];
  }
  return sum/AVG_ARR_LEN;
}


