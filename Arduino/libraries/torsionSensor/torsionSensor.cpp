/**
 * \file torsionSensor.cpp
 *
 * \author Sparty
 */


#include "torsionSensor.h"
#include "Arduino.h"

torsionSensor::torsionSensor(int pin0, int pin1)
{
   // Assume mPinSensor0 is 13 and mPinSensor1 is 2. Since registry 
   // Manipulation is for those ports.
   mPinSensor0 = pin0;
   mPinSensor1 = pin1;

   // Set pullups on torison inputs
   pinMode(mPinSensor1, OUTPUT);
   pinMode(mPinSensor0, OUTPUT);
   digitalWrite(mPinSensor1, 1);
   digitalWrite(mPinSensor0, 1);

   // long may be a 64-bit type, but 1 is still an int. You need to make 1 a long int using the UL suffix.
   unsigned long int REG_PMC_PCER0 = (1UL<<27); // Activate clock for TC0
   // Reset counter on TC_CV == RC
   int REG_TC0_CMR0 = (1<<0)|(1<<2)|(1<<8)|(1<<10)|(1 << 14);    // reset counter on index
   int REG_TC0_BMR = (1<<8)|(1<<9)|(1<<12);     // activate with no filters
   int REG_TC0_CCR0 = 5;    //enable clock and reset counter

   int REG_TC0_RC0 = 360;
   
   Serial.begin(9600);
}


void torsionSensor::loopTorsion()
{
    // put your main code here, to run repeatedly:
    Serial.println(REG_TC0_CV0); 
    Serial.print("   ");
    Serial.println(REG_TC0_QISR >> 8);
}