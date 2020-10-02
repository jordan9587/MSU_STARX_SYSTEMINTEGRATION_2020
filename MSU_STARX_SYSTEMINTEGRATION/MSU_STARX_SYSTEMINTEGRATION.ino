#include <pressureSensor.h>
#include <debugMenu.h>
#include <torsionSensor.h>
#include <PID_v1.h>
#include <Fsm.h>
#include <pwmConfiguration.h>

//PID
double setpoint;
double input;
double output;
double Kp=0;
double Ki=0;
double Kd=0;
// Declare direct and reverse PID instances.
PID myPID_direct(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_reverse(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

//Actuator variables come below:
int pressurePin = A3;

// Potentiometers/Torsion Sensors
int potentiometersHipPin = A1;
int potentiometersKneePin = A2;

// Pressure Sensor

// Three Tier Tyler Gait Setpoints
int hip_0degrees = 580; 
int hip_10degrees = hip_0degrees+35;
int hip_15degrees = hip_0degrees-42;
int hip_30degrees = hip_15degrees-30;
int knee_0degrees = 600;
int knee_30degrees = knee_0degrees-130;
int knee_60degrees = knee_0degrees-220;


// Examples to call onto custom libraries.
pressureSensor pressureSensor(A0, A1);
debugMenu debugMenu;
pwmConfiguration pwmConfiguration();
torsionSensor torsionSensor(13,2);

// Debug Menu
int debugStatus = 0;

// Example for State Machine example:
#define LED1_PIN 10
#define LED2_PIN 11

/* States functions and transition links*/
void on_led1_on_enter() 
{
    Serial.println("on_led1_on_enter");
    digitalWrite(LED1_PIN, HIGH);
}

void on_led1_off_enter()
{
    Serial.println("on_led1_off_enter");
    digitalWrite(LED1_PIN, LOW);
}

void on_led2_on_enter() 
{
    Serial.println("on_led2_on_enter");
    digitalWrite(LED2_PIN, HIGH);
}

void on_led2_off_enter() 
{
    Serial.println("on_led2_off_enter");
    digitalWrite(LED2_PIN, LOW);
}


/* States and State Machine Declaration*/
// Midstance
// HIP AND KNEE TO 0°
State state_led1_on(&on_led1_on_enter, NULL, NULL);
// PRE-SWING: HIP:0°, KNEE:0->(-60°)
State state_led1_off(&on_led1_off_enter, NULL, NULL);
// MID-SWING: HIP:0->15°, KNEE:(-60°)->(-30°)
State state_led2_on(&on_led2_on_enter, NULL, NULL);
 // TERMINAL SWING: HIP:15°->30°, KNEE:(-30°)->0°
State state_led2_off(&on_led2_off_enter, NULL, NULL);
Fsm fsm_led1(&state_led1_off);
Fsm fsm_led2(&state_led2_off);


void setup() 
{
  // put your setup code here, to run once:


  // Set pins for actuators, PWM for knee and hip for potentiometer, ect.
  pinMode(potentiometersHipPin, INPUT); 
  pinMode(potentiometersKneePin, INPUT); 

  Serial.begin(9600);  

  // PID Reverse
  //Initial setpoint
  setpoint = 75;
  //Turn the PID on
  myPID_reverse.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID_reverse.SetTunings(Kp, Ki, Kd);

  // PID Direct
  //Initial setpoint
  setpoint = 75;
  //Turn the PID on
  myPID_direct.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID_direct.SetTunings(Kp, Ki, Kd);

  /* State Machine transition links
  Depending on the FSM project comes up with transition link wise, implement like below
  */
  fsm_led1.add_timed_transition(&state_led1_off, &state_led1_on, 1000, NULL);
  fsm_led1.add_timed_transition(&state_led1_on, &state_led1_off, 3000, NULL);
  fsm_led2.add_timed_transition(&state_led2_off, &state_led2_on, 1000, NULL);
  fsm_led2.add_timed_transition(&state_led2_on, &state_led2_off, 2000, NULL);
}

void loop() 
{
  // Enable actuators

  debugMenu.startMenu();
  /*
  Serial.println("STARX Start Menu:");
  Serial.println("\n");
  Serial.println("0. Heirarchal State Machine");  
  Serial.println("1. Break");  
  Serial.println("2. Calibrate Suit");
  */

  // Debug Menu
  switch(debugStatus)
    {
        case '0':
            // State Machine
            fsm_led1.run_machine();
            fsm_led2.run_machine();
            break;
        case '1':
            // Break
            break;
        case '2':
            debugMenu.calibrateAction();
            break;
        // operator doesn't match any case debugStatus /
        default:
            printf("Error! operator is not correct");
            break;
    }
}
