/**
 ******************************************************************************
 * @file    X_NUCLEO_6180A1_Interrupt.ino
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

#ifndef USER_BTN
#define USER_BTN 5
#endif

int mode = 0;
const int buttonPin = USER_BTN;
int PushButtonState = LOW;
volatile bool button = false;

#define INTERRUPT_PIN A3
volatile bool interruptCount = false;

#define NUM_INTERRUPT_MODES 4

// Components
STMPE1600DigiOut xshutdown_top(&DEV_I2C, GPIO_12, (0x42 * 2));
VL6180_X_NUCLEO_6180A1 sensor_vl6180_top(&DEV_I2C, &xshutdown_top);

void button_pressed()
{
  button = true;
}

void INT_callback()
{
  interruptCount = true;
}

void setup()
{
   // Led
   pinMode(LedPin, OUTPUT);

   // Initialize serial for output
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Button Event Setting
   pinMode(buttonPin,INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(buttonPin), button_pressed, FALLING);

   pinMode(INTERRUPT_PIN, INPUT_PULLUP);
   attachInterrupt(INTERRUPT_PIN, INT_callback, RISING);

   // Initialize I2C bus
   DEV_I2C.begin();

   // Configure VL6180 top component
   sensor_vl6180_top.begin();

   // Switch off VL6180 top component
   sensor_vl6180_top.VL6180_Off();

   // Initialize VL6180 top component
   sensor_vl6180_top.InitSensor(0x10);

   // Disable Wrap Around Filter
   sensor_vl6180_top.VL6180_FilterSetState(0);

   // Prepare Measurements
   sensor_vl6180_top.VL6180_Prepare();

   // Increase convergence time to the max (this is because proximity config of API is used)
   sensor_vl6180_top.VL6180_RangeSetMaxConvergenceTime(63);

   // Set max upscale so we can work up to some  50cm
   sensor_vl6180_top.VL6180_UpscaleSetScaling(3);

   // Set inter measurement period
   sensor_vl6180_top.VL6180_RangeSetInterMeasPeriod(50);

   // Setup Interrupt
   sensor_vl6180_top.VL6180_SetupGPIO1(GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);

   // Make sure from now on all register in group are not fetched by device
   sensor_vl6180_top.VL6180_SetGroupParamHold(1);

   // Disable any threshold
   sensor_vl6180_top.VL6180_RangeSetThresholds(0, 0, 0 );

   // Set range interrupt reporting new sample ready
   sensor_vl6180_top.VL6180_RangeConfigInterrupt(CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);

   // Leave device peak up all new register in group
   sensor_vl6180_top.VL6180_SetGroupParamHold(0);

   // Clear any interrupt that should ensure a new edge get generated even if we missed it
   sensor_vl6180_top.VL6180_ClearAllInterrupt();

   // Start continuous mode
   sensor_vl6180_top.VL6180_RangeStartContinuousMode();
}

void loop()
{
   if(button)
   {
      // Debouncing
      delay(50);

      // Wait until the button is released
      while ((digitalRead(buttonPin) == PushButtonState));

      // Debouncing
      delay(50);

      // Change mode
      mode = ((mode + 1) % NUM_INTERRUPT_MODES);

      switch(mode)
      {
         case 0:     /* CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY */
         default:
         {
            // Make sure from now on all register in group are not fetched by device
            sensor_vl6180_top.VL6180_SetGroupParamHold(1);

            // Disable any threshold
            sensor_vl6180_top.VL6180_RangeSetThresholds(0, 0, 0 );

            // Set range interrupt reporting new sample ready
            sensor_vl6180_top.VL6180_RangeConfigInterrupt(CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);

            // Leave device peak up all new register in group
            sensor_vl6180_top.VL6180_SetGroupParamHold(0);

            // Clear any interrupt that should ensure a new edge get generated even if we missed it
            sensor_vl6180_top.VL6180_ClearAllInterrupt();
            break;
         }
         case 1:     /* CONFIG_GPIO_INTERRUPT_LEVEL_LOW */
         {
            // Make sure from now on all register in group are not fetched by device
            sensor_vl6180_top.VL6180_SetGroupParamHold(1);

            // Get interrupt whenever lower than 100mm
            sensor_vl6180_top.VL6180_RangeSetThresholds(100, 0, 0 );

            // Set range interrupt reporting low threshold
            sensor_vl6180_top.VL6180_RangeConfigInterrupt(CONFIG_GPIO_INTERRUPT_LEVEL_LOW);

            // Leave device peak up all new register in group
            sensor_vl6180_top.VL6180_SetGroupParamHold(0);

            // Clear any interrupt that should ensure a new edge get generated even if we missed it
            sensor_vl6180_top.VL6180_ClearAllInterrupt();
            break;
         }
         case 2:     /* CONFIG_GPIO_INTERRUPT_LEVEL_HIGH */
         {
            // Make sure from now on all register in group are not fetched by device
            sensor_vl6180_top.VL6180_SetGroupParamHold(1);

            // Get interrupt whenever higher than 200mm
            sensor_vl6180_top.VL6180_RangeSetThresholds(0, 200, 0 );

            // Set range interrupt reporting high threshold
            sensor_vl6180_top.VL6180_RangeConfigInterrupt(CONFIG_GPIO_INTERRUPT_LEVEL_HIGH);

            // Leave device peak up all new register in group
            sensor_vl6180_top.VL6180_SetGroupParamHold(0);

            // Clear any interrupt that should ensure a new edge get generated even if we missed it
            sensor_vl6180_top.VL6180_ClearAllInterrupt();
            break;
         }
         case 3:     /* CONFIG_GPIO_INTERRUPT_OUT_OF_WINDOW */
         {
            // Make sure from now on all register in group are not fetched by device
            sensor_vl6180_top.VL6180_SetGroupParamHold(1);

            // Get interrupt whenever out of  100mm  200mm  range
            sensor_vl6180_top.VL6180_RangeSetThresholds(100, 200, 0 );

            // Set range interrupt reporting out of window threshold
            sensor_vl6180_top.VL6180_RangeConfigInterrupt(CONFIG_GPIO_INTERRUPT_OUT_OF_WINDOW);

            // Leave device peak up all new register in group
            sensor_vl6180_top.VL6180_SetGroupParamHold(0);

            // Clear any interrupt that should ensure a new edge get generated even if we missed it
            sensor_vl6180_top.VL6180_ClearAllInterrupt();
            break;
         }
      }

      button = false;
      // Reset the interrupt flag because we changed the interrupt mode, so we need to synchronize to next event
      interruptCount = false;
   }

   if(interruptCount)
   {
      VL6180_RangeData_t Range;
      IntrStatus_t IntStatus;
      int status;
      char report[64];

      // Led on
      digitalWrite(LedPin, HIGH);

      status = sensor_vl6180_top.VL6180_RangeGetInterruptStatus(&IntStatus.val);
      if (!status)
      {
         status = sensor_vl6180_top.VL6180_RangeGetMeasurement(&Range);

         if(!status && (Range.errorStatus == 0 || Range.errorStatus == 7))
         {
            switch( IntStatus.status.Range )
            {
               case RES_INT_STAT_GPIO_LOW_LEVEL_THRESHOLD :
               SerialPort.print("L: ");
               break;
               case RES_INT_STAT_GPIO_HIGH_LEVEL_THRESHOLD :
               SerialPort.print("H: ");
               break;
               case RES_INT_STAT_GPIO_OUT_OF_WINDOW :
               SerialPort.print("O: ");
               break;
               case RES_INT_STAT_GPIO_NEW_SAMPLE_READY:
               SerialPort.print("N: ");
               break;
            }

            snprintf(report, sizeof(report), "%ld, %ld", Range.range_mm, Range.errorStatus);
            SerialPort.println(report);
         }
      }

      sensor_vl6180_top.VL6180_RangeClearInterrupt(); /* clear it */

      interruptCount = false;

      // Led off
      digitalWrite(LedPin, LOW);
   }
}
