/**
 * \file pressureSensor.cpp
 *
 * \author Sparty
 */


#include "pressureSensor.h"
#include "Arduino.h"

pressureSensor::pressureSensor(int pin0, int pin1)
{
  mPinSensor0 = pin0;
  mPinSensor1 = pin1;
}


void pressureSensor::readPressureSensor()
{
  // transfer function for sensor Honeywell ASDXRRX100PGAA5 (100 psi, 5V, A-calibration)
  // Vout = 0.8*Vsupply/(Pmax - Pmin)*(Papplied - Pmin) + 0.1*Vsupply
  // Rearrange to get: Papplied = (Vout/Vsupply - 0.1)*(Pmax - Pmin)/0.8 + Pmin;
  if (mCalibrationA)
  {
    // A calibration 10% to 90%
    // Output (volts) = [0.8 * V.(supply)]/[P.(max) - P.(min)] * (Pressure.(applied) - P.(min)) + 0.10 * V.(supply)
    // = [0.8 * (3.3 Volts)]/[100 psi - 0.36 psi] * (Pressure.(applied) - (0.36 psi)) + 0.10 * (3.3 Volts)
    // 0.36 psi is lowest 10 inches H20 conversion
    //float A_calibration = (0.8 * (3.3))/(100 - 0.36) * (psi_Pressure - 0.36) + 0.10 * 3.3;
    mPressure0 = (analogRead(mPinSensor0)/1024.0 - 0.1)*100.0/0.8;
    mPressure1 = (analogRead(mPinSensor0)/1024.0 - 0.1)*100.0/0.8;
  }
  else
  {
    // B calibration, 5% to 95%
    // Output (volts) = [0.9 * V.(supply)]/[P.(max) - P.(min)] * (Pressure.(applied) - P.(min)) + 0.05 * V.(supply)
    // = [0.9 * (3.3 Volts)]/[P.(max) - 0.36 psi] * (Pressure.(applied) - 0) + 0.05 * (3.3 Volts)
    // 0.36 psi is lowest 10 inches H20 conversion
    // float B_calibration = (0.9 * (3.3))/(100 - 0.36) * (psi_Pressure - 0.36) + 0.05 * 3.3;
     mPressure0 = (analogRead(mPinSensor0)/1024.0 - 0.1)*100.0/0.9;
     mPressure1 = (analogRead(mPinSensor0)/1024.0 - 0.1)*100.0/0.9;
  }
  
}
