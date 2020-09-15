/**
 * \file debugMenu.h
 *
 * \author Sparty
 */


#ifndef debugMenu_h
#define debugMenu_h

#include "debugMenu.h"

class debugMenu
{
  public:
    /**
    * Constructor
    */
    debugMenu();
    /**
    * Prints the options user has for starting exoSuit
    */
    void startMenu();
    /**
    * Prints the options user has for controlling exoSuit during runtime
    */
    void controlMenu();

    /**
    * Calls upon functions to move exoSuit forward.
    */
    void forwardAction();
    /**
    * Calls upon functions to move exoSuit backwards.
    */
    void backwardsAction();
    /**
    * Calls upon functions to halt exoSuit from further movement.
    */
    void haltAction();
    /**
    * Save calibrated variables.
    */
    void saveCalibrationAction();
    /**
    * Reads and outputs input pins from parameter (digital or analogue)
    * @param char pin0 as input pin value being printed with serial.println()
    */
    void readPin(int pin0);
    /**
    * Increase Actuator intensity
    */
    void increaseIntensityAction();
    /**
    * Decrease Actuator intensity
    */
    void decreaseIntensityAction();
    /**
    * Increase Delay count
    */
    void increaseDelayAction();
    /**
    * Decrease Delay count
    */
    void decreaseDelayAction();
    /**
    * Functions and PID that will drive State machine that mimics gait model.
    */
    void heirarchalStateMachine();
    /**
    * Calibrate exoSuit
    */
    void calibrateAction();

  private:
    // Delay counter 
    float delayCounter = 0;

};

#endif
