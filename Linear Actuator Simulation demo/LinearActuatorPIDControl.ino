///////////////////////////////////////////////////////////
// PID control of DC motor with rotary dual phase encoder//
///////////////////////////////////////////////////////////

#include DCMotor.h
#include <PID_v1.h>
#define motorDirPin 5 //Dir pin
#define motorPWMPin 6 //PWM pin
#define EnablePin 9 //Enable pin
#define encoderPinA 2
#define encoderPinB 3
#define LED 13 
//#define Kp 1200   //PID defualts for Linear Actuator
//#define Ki 250000
//#define Kd 10000  
//#define setpoint 360    //defining setpoint for motor pos

// motor control initialization
DCMotor mtr_ctrl(motorPWMPin, motorDirPin, EnablePin); //pin 6: pwm; pin 5: Dir; pin 9: Enable


volatile double encoderPos = 0;

//float ratio = (360)/1024;
//const float ratio = (360/188.611)/48; //Ratio from reference code
//Reference code ratio values:
// 360 -> 1 rotation
// 188.611 -> Gear Ratio
// 48 -> Pulses Per Revolution from encoder 
//1024 -> Pulses Per Revolution from encoder (servo specs)



// PID values

double Kp = 1200;
double Kd = 25;
double Ki = 0.1;
double setpoint = 90;

double encoderDeg;
double motorDeg = 0;
float past_error;
unsigned long lastTime;
float error;
float D_error = 0;
float I_error = 0;
double control = 0;
int count=0;

PID myPID(&motorDeg, &control, &setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(9600);
  
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, runEncoderA, CHANGE);
  
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, runEncoderB, CHANGE);
  
  pinMode(LED, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  pinMode(EnablePin, OUTPUT);
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move 
}

void loop()
{
  count+=1;
  //EncoderPos mapped from 0 to 360deg (9061 value found from observing encoderPos change vs degree)
  motorDeg = map((encoderPos), 0, 9061, 0, 360);
  myPID.Compute();
  //motorDeg = (encoderPos)*ratio;  //Input from reference code
  
  digitalWrite(EnablePin, 255);
  
  
  if (control>=0){
    runMotor(HIGH, min(abs(control), 255)); //Forward Direction
  }
  else{
    runMotor(LOW, min(abs(control), 255));  //Reverse Direction
  }
  
  //Serial.print("encoderPos : ");
  //Serial.print(encoderPos);
  //Serial.print("   motorDeg : ");
  Serial.println(motorDeg);
  //Serial.print("   error : ");
  //Serial.print(error);
  //Serial.print("    control : ");
  //Serial.print(control);
  //Serial.print("    motorVel : ");
  //Serial.println(min(abs(control), 255)); 
  
}

void runEncoderA()
{
  if (digitalRead(encoderPinA)==digitalRead(encoderPinB)){
    encoderPos += 1;
  }
  else{
    encoderPos -= 1;
  }
}
void runEncoderB()
{  
  if (digitalRead(encoderPinA)==digitalRead(encoderPinB)){
    encoderPos -= 1;
  }
  else{
    encoderPos += 1;
  }
}


void runMotor(bool dir, int vel)
{
  //Sets motor direction
  mtr_ctrl.motorDir(dir);
  digitalWrite(LED, dir);
  //Sets velocity
  if (dir == true){
    mtr_ctrl.pwmMotor(255-vel);
  }
  else{
    mtr_ctrl.pwmMotor(vel);
  }
}
