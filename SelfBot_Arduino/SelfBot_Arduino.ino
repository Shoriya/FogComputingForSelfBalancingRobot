#include "Wire.h"

#define MPU_addr 0x68

#define m1_left   10
#define m1_right   9

#define m2_left  8
#define m2_right  7

#define pwm1 3
#define pwm2 4

double AccelX,AccelY,AccelZ,Tmp,GyroX,GyroY,GyroZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int. We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double Angle_X, Angle_Y; //These are the angles in the complementary filter
#define degconvert 57.29577951//there are like 57 degrees in a radian.



#define Kp 40
#define Kd 0.05
#define Ki 40

#define sampleTime  0.005
#define targetAngle 0

volatile int output = 0;
volatile float currentAngle, prevAngle=0, error, prevError=0, errorSum=0;

void setup() {
   
  // set the motor control and PWM pins to output mode
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(m1_left, OUTPUT);
  pinMode(m1_right, OUTPUT);
  pinMode(m2_left, OUTPUT);
  pinMode(m2_right, OUTPUT);
 
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(57600);
  delay(100);
  //setup starting angle
  //1) collect the data
 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  AccelX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AccelY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccelZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyroX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyroY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyroZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
  double roll = atan2(AccelY, AccelZ)*degconvert;
  double pitch = atan2(-AccelX, AccelZ)*degconvert;
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double Angle_X = roll;
  double Angle_Y = pitch;
 
  //start a timer
  timer = micros();
}


void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  
  if(leftMotorSpeed >= 0) {
    analogWrite(pwm1, leftMotorSpeed);
    digitalWrite(m1_right, HIGH);
    digitalWrite(m1_left, LOW);
  }
  else {
    analogWrite(pwm1, - leftMotorSpeed);
    digitalWrite(m1_right, LOW);
    digitalWrite(m1_left, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(pwm2, leftMotorSpeed);
    digitalWrite(m2_right, HIGH);
    digitalWrite(m2_left, LOW);
  }
  else {
    analogWrite(pwm2, - leftMotorSpeed);
    digitalWrite(m2_right, LOW);
    digitalWrite(m2_left, HIGH);
  }
}

void emergency(){
  digitalWrite(pwm1, LOW);
  digitalWrite(pwm2, LOW);
  digitalWrite(m1_right, LOW);
  digitalWrite(m2_right, LOW);
  digitalWrite(m1_left, LOW);
  digitalWrite(m2_left, LOW);
}


void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  AccelX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AccelY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccelZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyroX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyroY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyroZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
  double roll = atan2(AccelY, AccelZ)*degconvert;
  double pitch = atan2(-AccelX, AccelZ)*degconvert;
  double gyroXrate = GyroX/131.0; 
  double gyroYrate = GyroY/131.0;  
  Angle_X = 0.99 * (Angle_X + gyroXrate * dt) + 0.01 * roll;
  Angle_Y = 0.99 * (Angle_Y + gyroYrate * dt) + 0.01 * pitch;

  currentAngle = Angle_Y;
  
  Serial.println(currentAngle);
  if(currentAngle >50 && currentAngle < -50){
      emergency();
     
  }
  /*
  else if(currentAngle >-0.3 && currentAngle <0.3){
      emergency();
      delay(50);
  }*/
  
  else{
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  output = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  output = constrain(output, -255, 255);
 
  setMotors(output, output);
  prevAngle = currentAngle;
 
  }
}
