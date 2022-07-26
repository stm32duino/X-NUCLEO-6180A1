/**
 ******************************************************************************
 * @file    X_NUCLEO_6180A1_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    15 January 2021
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-6180A1
 *          proximity sensor expansion board based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl6180_x_nucleo_6180a1_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Please uncomment the line below if you want also to use the satellites
//#define SATELLITES_MOUNTED

// Components
STMPE1600DigiOut xshutdown_top(&DEV_I2C, GPIO_12);
VL6180_X_NUCLEO_6180A1 sensor_vl6180_top(&DEV_I2C, &xshutdown_top);
#ifdef SATELLITES_MOUNTED
STMPE1600DigiOut xshutdown_left(&DEV_I2C, GPIO_14);
VL6180_X_NUCLEO_6180A1 sensor_vl6180_left(&DEV_I2C, &xshutdown_left);
STMPE1600DigiOut xshutdown_right(&DEV_I2C, GPIO_15);
VL6180_X_NUCLEO_6180A1 sensor_vl6180_right(&DEV_I2C, &xshutdown_right);
#endif

void setup()
{
   // Led
   pinMode(LedPin, OUTPUT);

   // Initialize serial for output
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Initialize I2C bus
   DEV_I2C.begin();

   // Configure VL6180 top component
   sensor_vl6180_top.begin();

   // Switch off VL6180 top component
   sensor_vl6180_top.VL6180_Off();

#ifdef SATELLITES_MOUNTED
   // Configure (if present) VL6180 left component
   sensor_vl6180_left.begin();

   // Switch off (if present) VL6180 left component
   sensor_vl6180_left.VL6180_Off();

   // Configure (if present) VL6180 right component
   sensor_vl6180_right.begin();

   // Switch off (if present) VL6180 right component
   sensor_vl6180_right.VL6180_Off();
#endif

   // Initialize all the sensors
   sensor_vl6180_top.InitSensor(0x10);

#ifdef SATELLITES_MOUNTED
   sensor_vl6180_left.InitSensor(0x12);
   sensor_vl6180_right.InitSensor(0x14);
#endif

   // Prepare Measurements
   sensor_vl6180_top.VL6180_Prepare();
#ifdef SATELLITES_MOUNTED
   sensor_vl6180_left.VL6180_Prepare();
   sensor_vl6180_right.VL6180_Prepare();
#endif
}

void loop()
{
   int status;
   VL6180_RangeData_t Range;
   char report[64];

   // Start Measurements in polling mode
   status = sensor_vl6180_top.VL6180_RangePollMeasurement(&Range);

   // Led on
   digitalWrite(LedPin, HIGH);

   if(!status)
   {
      snprintf(report, sizeof(report), "VL6180 Top: Distance=%ld, Range_status=%ld", Range.range_mm, Range.errorStatus);
      SerialPort.println(report);
   }

   // Led off
   digitalWrite(LedPin, LOW);

#ifdef SATELLITES_MOUNTED 
   status = sensor_vl6180_left.VL6180_RangePollMeasurement(&Range);

   // Led on
   digitalWrite(LedPin, HIGH);

   if(!status)
   {
      snprintf(report, sizeof(report), "VL6180 Left: Distance=%ld, Range_status=%ld", Range.range_mm, Range.errorStatus);
      SerialPort.println(report);
   }

   // Led off
   digitalWrite(LedPin, LOW);

   status = sensor_vl6180_right.VL6180_RangePollMeasurement(&Range);

   // Led on
   digitalWrite(LedPin, HIGH);

   if(!status)
   {
      snprintf(report, sizeof(report), "VL6180 Right: Distance=%ld, Range_status=%ld", Range.range_mm, Range.errorStatus);
      SerialPort.println(report);
   }

   // Led off
   digitalWrite(LedPin, LOW);
#endif
}
