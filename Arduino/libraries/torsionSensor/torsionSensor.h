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
    void loopTorsionOutput();
    
  protected:
    // track last position so we know whether it's worth printing new output
    long previous_position = 0;
    
  private:
    // Registry Manipulation Constants:
    // Assume mPinSensor0 is 13 and mPinSensor1 is 2.
    int mPinSensor0;    // Analog pin 0 for left torsion sensor
    int mPinSensor1;    // Analog pin 1 for right torsion sensor



};

#endif
