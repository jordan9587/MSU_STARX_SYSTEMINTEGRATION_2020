/**
 * \file debugMenu.cpp
 *
 * \author Sparty
 */


#include "debugMenu.h"
#include "Arduino.h"

debugMenu::debugMenu()
{

}

void debugMenu::controlMenu()
{ 
  Serial.println("STARX Control Menu:");
  Serial.println("\n");
  Serial.println("0. Forward");
  Serial.println("1. Backwards");
  Serial.println("2. Halt");
  Serial.println("3. Calibration save as");
  Serial.println("4. Send values to output");
  Serial.println("5. ++velocity");
  Serial.println("6. --velocity");
  Serial.println("7. ++delay");
  Serial.println("8. --delay");
  Serial.println("\n");
}


void debugMenu::startMenu()
{
  Serial.println("STARX Start Menu:");
  Serial.println("\n");
  Serial.println("0. Heirarchal State Machine");  
  Serial.println("1. Break");  
  Serial.println("2. Calibrate Suit");
  Serial.println("\n");
}

void debugMenu::forwardAction()
{
  // forward actuator at a specific rate.
  // Delay
  // halt actuator

}


void debugMenu::backwardsAction()
{
  // Reverse actuator at specific rate
  // Delay
  // halt actuator
}

void debugMenu::haltAction()
{
  // Stop actuator
}

void debugMenu::saveCalibrationAction()
{
  // save values for calibration
}

void debugMenu::readPin(int pin0)
{
  if (pin0 == A0 || A1 || A3 || A4 || A5 || A6 || A7 || A8 ||
   A9 || A10 ||  A11 )
   {
      // Analogue input pin
      Serial.println(analogRead(pin0));
   }
   else
   {
      // Digital input pin
      Serial.println(digitalRead(pin0));
   }
}

void debugMenu::increaseIntensityAction()
{
  // increase Actuator rate of change
}

void debugMenu::decreaseIntensityAction()
{
  // decrease Actuator rate of change

}


void debugMenu::increaseDelayAction()
{
  // increase Actuator rate of change
  delayCounter += 5;
  Serial.println("Delay increased by 5 ms.");
  Serial.println((delayCounter));
}

void debugMenu::decreaseDelayAction()
{
  // increase Actuator rate of change
  delayCounter -= 5;
  Serial.println("Delay decreased by 5 ms.");
  Serial.println((delayCounter));
}

void debugMenu::heirarchalStateMachine()
{
    // States
}

void debugMenu::calibrateAction()
{
  // Calibrate
}

