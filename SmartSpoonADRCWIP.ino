#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

Servo motor;
LSM9DS1 imu;

#define sampleTime 0.01

#define LSM9DS1_G 0x6B
#define LSM9DS1_A 0x6B

/* Variables */
float z1, z2, z3, z1_new, z2_new, z3_new, uLESO, yLESO;    //LESO variables
float gyroRead, gyroFB, potFB, uADRC;                      //LADRC Inputs/Outputs
int N, Km, R, Js, Jm;                                      //Physical system parameters
int offset = 0;                                            //Offset variable dictating motor rotation direction
int motorPWM = 0;                                          //PWM output to motor

/* Constants */
int offsetACW = 1600;                                      //Offset to make motor turn in ACW direction
int offsetCW = 1400;                                       //Offset to make motor turn in CW direction
int offsetN = 1500;                                        //Offset to stop motor turning

int potMotor = A0;                                         //Analog input pin for motor potentiometer
int potPlatform = A3;                                      //Analog input pin for platform potentiometer
int potHome = 220;                                         //Potentiometer value when platform and spoon are parallel

int b0 = 100;                                              //Input scaling factor
int Kp = 4;                                                //Proportional gain in PD controller
int Kd = 4;                                                //Derivative gain in PD controller
int Kp_M = 50; 
int Kphi = 1;                                              //Gain of the spoon potentiometer
int w0 = 800;                                              //System BW

/* Calculations */
int beta1 = 3*w0;                                          //Gain beta1 calculation
int beta2 = 3*(w0^2);                                      //Gain beta2 calculation
int beta3 = w0^3;                                          //Gain beta3 calculation
int alpha_M = (N*Km)/R;                                    //alpha calculation in physical system dynamics
int beta_M = (N^2*Km^2)/R;                                 //beta calculation in physical system dynamics
int inertia_M = (Js + Jm*N^2);                             //J calculation in physical system dynamics



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

  /* Pin Connections */
  motor.attach(9);
  pinMode(potMotor, INPUT);
  pinMode(potPlatform, INPUT);

  /* Initial System States */
  uADRC = 0;
  z1 = 0;
  z2 = 0;
  z3 = 0;
}

void runMotor(int potSetpoint) {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
  potFB = analogRead(potMotor);
  gyroFB = -(imu.calcGyro(imu.gx));
  gyroFB *= PI/180;
  uLESO = uADRC*b0;
  z1_new = (0.005703*z1) + (-1.006e-05*z2) + (1.677e-08*z3) + (1.677e-06*uLESO) + (0.9943*gyroFB);
  z2_new = (10.73*z1) + (-0.01845*z2) + (3.019e-05*z3) + (0.003019*uLESO) + (-10.73*gyroFB);
  z3_new = (5153*z1) + (-8.588*z2) + (0.01375*z3) + (-98.62*uLESO) + (-5153*gyroFB);
  float u0 = ((potFB - z1_new)*Kp) - (z2_new*Kd);
  uADRC = (u0 - z3_new)/b0;
  float error = potSetpoint - Kp_M*potFB + uADRC;
  if (error < -1) {
    offset = offsetCW; //Adds the necessary offset for the motor to turn in the CW direction
  }
  else if (error > 1) {
    offset = offsetACW; //Adds the necessary offset for the motor to turn in the ACW direction
  }
  else {
    offset = offsetN; //Adds the necessary offset for the motor to stop turning
  }
  //Update current system states
  z1 = z1_new;
  z2 = z2_new;
  z3 = z3_new;
  
  float output = error;
  motorPWM = round(output) + offset;
  motor.writeMicroseconds(motorPWM);
  delay(5);
}

void loop() {
  runMotor(potHome);
}
