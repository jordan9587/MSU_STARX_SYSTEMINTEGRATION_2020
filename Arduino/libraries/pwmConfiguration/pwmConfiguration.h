/**
 * \file pwmConfiguration.h
 *
 * \author Sparty
 * 
 * The PWM pins digital pins 9 - 6 will be reserved for the solenoid valves.
 * PWML4-7 are the respective PWM channels for each digital pin DUE Datasheet
 * chapter 38 and 28. These functions should be placed in setup() for Arudino Due.
 * Designed for Arduino Due with Atmel SAM3X8E ARM Cortex-M3 CPU.
 * 
 */


#ifndef pwmConfiguration_h
#define pwmConfiguration_h

#include "Arduino.h"

class pwmConfiguration
{
  public:
    /*
    * Constructor for PWM configuration. Pins 9-6 reserved for solenoid values.
    */
    pwmConfiguration();

    /*
    * Configures channel by setting channel prescalers, setting period/frequency 
    * and setting duty cycles.
    */
    void channelConfiguration();



};

#endif
