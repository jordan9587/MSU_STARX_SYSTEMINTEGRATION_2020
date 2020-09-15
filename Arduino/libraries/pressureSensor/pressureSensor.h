/**
 * \file pressureSensor.h
 *
 * \author Sparty
 */


#ifndef pressureSensor_h
#define pressureSensor_h

#include "Arduino.h"

class pressureSensor
{
  public:
    /**
    * Constructor
    */
    pressureSensor(int pin0, int pin1);
    /*
    * Reads voltage pressure sensors and returns their pressure value
    * declared as void for now.
    */
    void readPressureSensor();

  private:
    int mPinSensor0;    // Analog pin 0 for left pressure sensor
    int mPinSensor1;    // Analog pin 1 for right pressure sensor
    volatile float mPressure0 = 0;       // Store the pressure value for analog pin 0
    volatile float mPressure1 = 0;       // Store the pressure value for analog pin 1
    // A calibration if true for Honeywell ASDXRRX100PGAA5 pressure sensor, else false and set to B calibration.
    bool mCalibrationA = true;
};

#endif
