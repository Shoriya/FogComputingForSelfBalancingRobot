#include "Wire.h"
#define PERIOD  4000    // loop period in micros



///////////////// MPU-6050 //////////////////////////


static int MPU_ADDR = 0x68; //AD0 is HIGH

// MPU6050 specific
#define MPU6050_FS_SEL0 3
#define MPU6050_FS_SEL1 4
#define MPU6050_AFS_SEL0 3
#define MPU6050_AFS_SEL1 4

// Combined definitions for the FS_SEL values.eg.  ±250 degrees/second
#define MPU6050_FS_SEL_250  (0)
#define MPU6050_FS_SEL_500  (bit(MPU6050_FS_SEL0))
#define MPU6050_FS_SEL_1000 (bit(MPU6050_FS_SEL1))
#define MPU6050_FS_SEL_2000 (bit(MPU6050_FS_SEL1) | bit(MPU6050_FS_SEL0))

// Combined definitions for the AFS_SEL values
#define MPU6050_AFS_SEL_2G  (0)
#define MPU6050_AFS_SEL_4G  (bit(MPU6050_AFS_SEL0))
#define MPU6050_AFS_SEL_8G  (bit(MPU6050_AFS_SEL1))
#define MPU6050_AFS_SEL_16G (bit(MPU6050_AFS_SEL1)|bit(MPU6050_AFS_SEL0))

// See page 12 & 13 of MPU-6050 datasheet for sensitivities config and corresponding output      
#define GYRO_FULL_SCALE_RANGE         MPU6050_FS_SEL_250    
#define GYRO_SCALE_FACTOR             131     // LSB / (degs per seconds)
#define ACC_FULL_SCALE_RANGE          MPU6050_AFS_SEL_4G    
#define ACC_SCALE_FACTOR              8192    // LSB / g



static float GYRO_RAW_TO_DEGS = 1.0 / (1000000.0 / PERIOD) / GYRO_SCALE_FACTOR;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t gyroX_calibration, gyroY_calibration, gyroZ_calibration;

float roll, pitch, rollAcc, pitchAcc;
float speeed;

//////////////////////////////////////////////////////////////////////////////

//////// PID //////////

#define MAX_PID_OUTPUT 500

//float BASE_Kp = 50, BASE_Ki = 0, BASE_Kd = 0;
float Kp = 13, Ki = 0, Kd = 0;
float angleSetpoint = 0, selfBalanceAngleSetpoint = -3.5;
float pidOutput, pidError, pidLastError, integralErr, positionErr, serialControlErr, prevSerialControlErr, errorDerivative;

float MAX_CONTROL_OR_POSITION_ERR = MAX_PID_OUTPUT / Kp;
float MAX_CONTROL_ERR_INCREMENT = MAX_CONTROL_OR_POSITION_ERR / 400;
#define MIN_CONTROL_ERR 1

int32_t currentPos = 0;

// Motor A
int m1_1 = 27;
int m1_2 = 26;
int pwm1 = 14;


// Motor B
int m2_1 = 18;
int m2_2 = 19;
int pwm2 = 12;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 0;
const int resolution = 8;




/////////////////////////MOTOR_SETUP/////////////////////////////////

void motor_setup()
{
  pinMode(m1_1, OUTPUT);
  pinMode(m1_2, OUTPUT);
  pinMode(m2_1, OUTPUT);
  pinMode(m2_2, OUTPUT);
 
  ledcSetup(pwmChannel1, freq, resolution);   // configure LED PWM functionalitites
  ledcSetup(pwmChannel2, freq, resolution);
 
  ledcAttachPin(pwm1, pwmChannel1);  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pwm2, pwmChannel2);  

  // set the motor control and PWM pins to output mode
  //pinMode(pwm1, OUTPUT);
  //pinMode(pwm2, OUTPUT);
  
  }

/////////////////////SENSOR_SETUP///////////////////////////////////
void sensor_setup()
{
  Wire.begin();
  Wire.setClock(400000L);

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(MPU_ADDR);     
  Wire.write(0x6B);                     //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                     //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();
         
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                     //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(GYRO_FULL_SCALE_RANGE);                    
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                     //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(ACC_FULL_SCALE_RANGE);                    
  Wire.endTransmission();
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);                     //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                     //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();


  calibrateGyro();
}


#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

void getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();  
  Wire.requestFrom(MPU_ADDR, 6);
  *x = constr((((int16_t)Wire.read()) << 8) | Wire.read(), -ACC_SCALE_FACTOR, ACC_SCALE_FACTOR);
  *y = constr((((int16_t)Wire.read()) << 8) | Wire.read(), -ACC_SCALE_FACTOR, ACC_SCALE_FACTOR);
  *z = constr((((int16_t)Wire.read()) << 8) | Wire.read(), -ACC_SCALE_FACTOR, ACC_SCALE_FACTOR);
}

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

void getRotation(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);  
  *x = ((((int16_t)Wire.read()) << 8) | Wire.read()) - gyroX_calibration;
  *y = ((((int16_t)Wire.read()) << 8) | Wire.read()) - gyroY_calibration;
  *z = ((((int16_t)Wire.read()) << 8) | Wire.read()) - gyroZ_calibration;
}


void calibrateGyro() {
  int32_t x, y, z;
  
  for(int i=0; i<500; i++){
    getRotation(&gyroX, &gyroY, &gyroZ);
    x += gyroX;
    y += gyroY;
    z += gyroZ;

    delayMicroseconds(PERIOD); // simulate the main program loop time ??
  }

  gyroX_calibration = x / 500;
  gyroY_calibration = y / 500;
  gyroZ_calibration = z / 500;
}


//////////////////////// on ESP32 Arduino constrain doesn't work
int16_t constr(int16_t value, int16_t mini, int16_t maxi) {
  if(value < mini) return mini;
  else if(value > maxi) return maxi;
  return value;
}
float constrf(float value, float mini, float maxi) {
  if(value < mini) return mini;
  else if(value > maxi) return maxi;
  return value;
}

///////////////////////////////////////////////////////////////////


//////////////////////SETUP////////////////////////////////////////
void setup() {
  //SETup SERVER CLIENT
 Serial.begin(500000);
  motor_setup();
  sensor_setup();
   
  Wire.begin();
  
  
 
}

//////////////////////////////////////////////////////////////////


void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
 
  if(leftMotorSpeed >= 0) {
   
    ledcWrite(pwmChannel1, leftMotorSpeed);
    digitalWrite(m1_1, HIGH);
    digitalWrite(m1_2, LOW);
  }
  else {
    
    ledcWrite(pwmChannel1, - leftMotorSpeed);
    digitalWrite(m1_1, LOW);
    digitalWrite(m1_2, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    
    ledcWrite(pwmChannel2, rightMotorSpeed);
    digitalWrite(m2_1, HIGH);
    digitalWrite(m2_2, LOW);
  }
  else {
    
    ledcWrite(pwmChannel2, - rightMotorSpeed);
    digitalWrite(m2_1, LOW);
    digitalWrite(m2_2, HIGH);
  }
}

/////////////////////////////////////////////////////////////
void emergency(){
  digitalWrite(pwm1, LOW);
  digitalWrite(pwm2, LOW);
  digitalWrite(m1_1, LOW);
  digitalWrite(m2_1, LOW);
  digitalWrite(m1_2, LOW);
  digitalWrite(m2_2, LOW);
}


void loop() {
  getAcceleration(&accX, &accY, &accZ);
  
  rollAcc = asin((float)accX / ACC_SCALE_FACTOR) * RAD_TO_DEG;
  pitchAcc = asin((float)accY / ACC_SCALE_FACTOR) * RAD_TO_DEG;

  getRotation(&gyroX, &gyroY, &gyroZ);
  roll -= gyroY * GYRO_RAW_TO_DEGS;
  pitch += gyroX * GYRO_RAW_TO_DEGS;
  // sin() has to be applied on radians
//  roll += pitch * sin((float)gyroZ * GYRO_RAW_TO_DEGS * DEG_TO_RAD);
//  pitch -= roll * sin((float)gyroZ * GYRO_RAW_TO_DEGS * DEG_TO_RAD);


  roll = roll * 0.999 + rollAcc * 0.001;
  pitch = pitch * 0.999 + pitchAcc * 0.001;
  
  //currentAngle = Angle_Y;
 Serial.print(roll);
 Serial.print("      ");
 Serial.print(pitch);

 positionErr = constrf(currentPos / (float)1000, -MAX_CONTROL_OR_POSITION_ERR, MAX_CONTROL_OR_POSITION_ERR);
   
  pidError = roll  - selfBalanceAngleSetpoint;
  pidError += positionErr;  
  integralErr = constrf(integralErr + Ki * pidError, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
  errorDerivative = pidError - pidLastError;
  
  pidOutput = Kp * pidError + integralErr + Kd * errorDerivative;
  pidOutput = constrf(pidOutput, -250, 250);
  
 Serial.print("      ");
  Serial.println(pidError);
  //Serial.print("      ");
  //Serial.print(pidOutput);
  //Serial.println("      ");
  setMotors(pidOutput, pidOutput);
  pidLastError = pidError;
  //}
}
