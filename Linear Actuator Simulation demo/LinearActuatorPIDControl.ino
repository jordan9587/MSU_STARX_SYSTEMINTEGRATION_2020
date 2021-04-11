///////////////////////////////////////////////////////////
// PID control of DC motor with rotary dual phase encoder//
///////////////////////////////////////////////////////////
#define motorDirPin 5 //Dir pin
#define motorPWMPin 6 //PWM pin
#define EnablePin 9 //Enable pin
#define encoderPinA 2
#define encoderPinB 3
#define LED 13  //LED pin
#define Kp 1200   //PID defualts for Linear Actuator
#define Ki 250000
#define Kd 10000  
#define setpoint 360    //defining setpoint for motor pos
// motor control initialization
DCMotor mtr_ctrl(motorPWMPin, motorDirPin, EnablePin); //pin 6: pwm; pin 5: Dir; pin 9: Enable


volatile double encoderPos = 0;
//float ratio = (360)/1024;
//const float ratio = (360/188.611)/48;
// 360 -> 1 rotation
// 188.611 -> Gear Ratio
// 48 -> Pulses Per Revolution from encoder
//1024 -> Pulses Per Revolution from encoder (servo specs)



// PID control

//double Kp = 10; double Ki=0.00001; double Kd=0.4; //Most opitmal performance
//float Kp = 1200; float Ki=250000; float Kd=10000;
//double targetDeg = 360;   //Setpoint
double encoderDeg;
double motorDeg = 0;
float past_error;
unsigned long lastTime;
float error;
float D_error = 0;
float I_error = 0;
double control = 0;

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
}

void loop()
{
  //EncoderPos mapped from 0 to 360deg (9061 value found from observing encoderPos change vs degree)
  motorDeg = map((encoderPos), 0, 9061, 0, 360);
  //motorDeg = (encoderPos)*ratio;  //Input
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  
  error = setpoint - motorDeg;  //error calculation
  I_error = timeChange * error; //I calculation
  D_error = (past_error-error)/timeChange;  //D calculations
  
  control = Kp*error + Ki*I_error + Kd*D_error; //PID output
  
  digitalWrite(EnablePin, 255);
  
  if (control>=0){
    runMotor(HIGH, min(abs(control), 255));
  }
  else{
    runMotor(LOW, min(abs(control), 255));
  }
  past_error = error;
  lastTime = now;
  
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
  mtr_ctrl.motorDir(dir);
  digitalWrite(LED, dir);
  if (dir == true){
    mtr_ctrl.pwmMotor(255-vel);
  }
  else{
    mtr_ctrl.pwmMotor(vel);
  }
}
