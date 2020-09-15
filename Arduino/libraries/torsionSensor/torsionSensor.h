/**
 * \file torsionSensor.h
 *
 * \author Sparty
 */


#ifndef torsionSensor_h
#define torsionSensor_h

#include "Arduino.h"

class torsionSensor
{
  public:
    /*
    * Constructor
    */
    torsionSensor(int pin0, int pin1);

    /*
    * Loop required to read register and print current rotation.
    */
    void loopTorsion();

  private:
    // Assume mPinSensor0 is 13 and mPinSensor1 is 2.
    int mPinSensor0;    // Analog pin 0 for left torsion sensor
    int mPinSensor1;    // Analog pin 1 for right torsion sensor
};

#endif
