# X-NUCLEO-6180A1

Arduino library to support the X-NUCLEO-6180A1 based on VL6180 ranging sensor.
This sensor uses I2C to communicate. An I2C instance is required to access to the sensor.
The APIs provide simple distance measure in both polling and interrupt modes.

## Examples

There are 3 example with the X-NUCLEO-6180A1 library.

* X_NUCLEO_6180A1_HelloWorld: This example code is to show how to get the proximity
  values of the onboard VL6180 sensor in polling mode.
* X_NUCLEO_6180A1_Asynchronous: This example code is to show how to get the proximity
  values of the onboard VL6180 sensor using the asynchronous mode.
* X_NUCLEO_6180A1_Interrupt: This example code is to show how to setup the onboard
  VL6180 sensor using the interrupt mode with continuous measurements. At the beginning
  the sensor is configured to generate the interrupt on the new sample ready event;
  pushing the user button, the sensor is configured to generate the interrupt when the
  measurement is lower than 100 mm; pushing the user button again, the sensor is configured
  to generate the interrupt when the measurement is higher than 200 mm; pushing the user
  button again, the sensor is configured to generate the interrupt when the measurement
  is out of the window range between 100 mm and 200 mm; pushing the user button again the
  sensor is configured in the initial state and so on.

## Dependencies

This package requires the following Arduino libraries:

* STM32duino VL6180: https://github.com/stm32duino/VL6180

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-6180A1

The VL6180 datasheet is available at  
https://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl6180.html
