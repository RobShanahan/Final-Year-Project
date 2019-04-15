/////////////////////////////////////
//Libraries & Classes & Definitions//
/////////////////////////////////////
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

Servo motor;
LSM9DS1 imu;

#define sampleTime 0.01                //Sample time in seconds

#define LSM9DS1_G 0x6B
#define LSM9DS1_A 0x6B

/////////////
//Variables//
/////////////
short int valRead = 0;                 //value of motor potentiometer. In later iterations change this to platform potentiometer
short int motorPWM = 0;                //PWM signal used to control motor
short int offset = 0;
short int sample = 0;
float integral = 0;
float alpha = 1.0;
float prevError = 0;

/////////////
//Constants//
/////////////
const short int offsetACW = 1600;      //Offset applied to motorPWM calculation to make motor turn ACW
const short int offsetCW = 1400;       //Offset applied to motorPWM calculation to make motor tune CW
const short int offsetN = 1500;        //Offset applied to motorPWM calculation to stop motor turning
const short int potMT = A0;            //Potentiometer for motor feedback
const short int potIn = A3;            //Potentiometer for platform feedback
const short int potHome = 220;         //Potentiometer value when spoon is level
float gyroVal = 0;
float kp = 10.0;
float ki = 2.0;
float kd = 0.6;

/////////
//Setup//
/////////
void setup() {
  Serial.begin(115200);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.agAddress = LSM9DS1_A;
  imu.settings.device.agAddress = LSM9DS1_G;
  
  if (!imu.begin()) {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double check wiring.");
    
    while (1)
      ;
  }
  
  motor.attach(9); //Attaches the control signal of the motor to PWM pin 9 on Arduino
  pinMode(potMT, INPUT);
  pinMode(potIn, INPUT);
}

/////////////////
//Motor control//
/////////////////
void runMotor(short int potSetpoint) {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  valRead = analogRead(potMT);
  sample = (alpha * valRead) + ((1 - alpha) * sample);
  gyroVal = -(imu.calcGyro(imu.gx));
  gyroVal *= PI/180;
  float error = potSetpoint - sample;
  integral = integral + error*sampleTime;
  float derivative = -gyroVal;
  if (error < -1) {
    offset = offsetCW;      //Adds the necessary offset for the motor to turn in the CW direction
  }
  else if (error > 1) {
    offset = offsetACW;     //Adds the necessary offset for the motor to turn in the ACW direction
  }
  else {
    offset = offsetN;       //Adds the necessary offset for the motor to stop turning
  }
  prevError = error;
  float output = (kp*error + ki*integral + kd*derivative); //Equation to generate PWM signal for the motor
  motorPWM = round(output) + offset;
  motor.writeMicroseconds(motorPWM);
  delay(5);
}

////////////////
//Program Loop//
////////////////
void loop() {
  int val = analogRead(potIn);
  runMotor(285 - val); //Executes the runMotor function (in this case ensuring the motor returns to the potentiometer setpoint)
}
