#include "Mecanum_Movment.h"
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050_light.h>
#include <HUSKYLENS.h>
MPU6050 gyro(Wire);

LiquidCrystal_I2C lcd(0x27, 16, 2);
//Settings.|
//         |
Mecanum_Movment::Mecanum_Movment(int _m1a, int _m1b, int _m1e, int _m2a, int _m2b, int _m2e, int _m3a, int _m3b, int _m3e, int _m4a, int _m4b, int _m4e, int Kp, int Ki, int Kd)
{
  m1a = _m1a;
  m1b = _m1b;
  m1e = _m1e;
  m2a = _m2a;
  m2b = _m2b;
  m2e = _m2e;
  m3a = _m3a;
  m3b = _m3b;
  m3e = _m3e;
  m4a = _m4a;
  m4b = _m4b;
  m4e = _m4e;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m1e, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(m2e, OUTPUT);
  pinMode(m3a, OUTPUT);
  pinMode(m3b, OUTPUT);
  pinMode(m3e, OUTPUT);
  pinMode(m4a, OUTPUT);
  pinMode(m4b, OUTPUT);
  pinMode(m4e, OUTPUT);
}

void Mecanum_Movment::begin()
{
  setLCD();
  Serial.begin(115200);
  calibrateMPU();

}

void Mecanum_Movment::calibrateMPU() {
  // All kinds of things that need to be done.
  Serial.begin(115200);
  Wire.begin();
  // gyro.setGyroOffsets(-54, -36, 26);
	// gyro.setAccOffsets(-598,  -789, 2126);

  
  byte status = gyro.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while (status != 0) { } // Stop everything if could not connect to MPU6050.
  
  Serial.println("Calculating offsets, do not move MPU6050");
  delay(1000);
  // mpu.upsideDownMounting = true; // Uncomment this line if the MPU6050 is mounted upside-down.
  gyro.calcOffsets(); // Gyro and accelerometer.
  Serial.println("Done!\n");
}

int Mecanum_Movment::getYaw() {
  gyro.update();
  int yaw = gyro.getAngleZ();
  if (yaw >360){
    yaw -= 360;
  }
  else if (yaw<-360){
    yaw += 360;
  }

  return yaw;
}


//         ^
//Settings.|


//Gyro. |
//      |




void Mecanum_Movment::setLCD() {  
  lcd.begin();
  lcd.backlight();

}
void Mecanum_Movment::printYaw()
{
  Serial.print("Yaw: ");
  Serial.println(getYaw());
  lcd.print(getYaw());
}


void Mecanum_Movment::LcdYaw()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Yaw: ");
  lcd.println(getYaw());

}


void Mecanum_Movment::resetYaw()
{
  calibrateMPU();
}


//Gyro. ^
//      |


double Mecanum_Movment::calc(double sp, double pv)
{
  Time = millis()/1000.0;

  deltaTime = Time-prev_Time;
  if (deltaTime == 0) {
    deltaTime = 0.01;
  }

  error = sp - pv;
  P = kp * error;

  integral += error * deltaTime;
  I = ki * integral;

  derivative =(error - prev_Error)/deltaTime;
  D = kd * derivative;

  output = P + I + D;

  prev_Error = error;
  
  prev_Time = Time;

  return output;
}

void Mecanum_Movment:: FollowLine(int speed, int, color, float kp, float kifloat, kd){
  
}

void Mecanum_Movment::moveF(int speed)
{
  // Tf(speed);
  // THf(speed);
  // Of(speed);
  // Ff(speed);  
  integral = 0;
  prev_Error = 0;

  kp = Kp;
  ki = Ki;
  kd = Kd;

  while (true){ 
  moveFOut = calc(0, getYaw());
  moveFSR = abs(moveFOut) + speed;
  moveFSL = speed-abs(moveFOut);


  Tf(moveFSR);
  THf(moveFSL);
  Of(moveFSL);
  Ff(moveFSR);
  LcdYaw();}
}


void Mecanum_Movment::moveB(int speed)
{
  Tb(speed);
  THb(speed);
  Ob(speed);
  Fb(speed);
}

void Mecanum_Movment::turnR(int speed)
{
  Tb(speed);
  THf(speed);
  Of(speed);
  Fb(speed);
}

void Mecanum_Movment::turnL(int speed)
{
  Tf(speed);
  THb(speed);
  Ob(speed);
  Ff(speed);
}

void Mecanum_Movment::move(int degree, int speed)
{
  integral = 0;
  prev_Error = 0; 
  kp = Kp;
  ki = Ki;
  kd = Kd;

  moveOut = calc(degree, getYaw());
  moveSpeed = abs(moveOut) + speed;

  

}


void Mecanum_Movment::turn(int degree, int speed)
{
  integral = 0;
  prev_Error = 0;
  kp = Kp;
  ki = Ki;
  kd = Kd;
  while (getYaw() != degree) {
  LcdYaw();
  turnOut = calc(degree, getYaw());

  turnSpeed = speed + abs(turnOut);
  if (getYaw() == degree){
    break;
  }
  if (turnOut > 0) {
    turnR(turnSpeed);
    LcdYaw();


  }
  else {
    turnL(turnSpeed);
    LcdYaw();


  }
  
  LcdYaw();
  printYaw();
  }
  stopMotors();


}



void Mecanum_Movment::L(int speed)
{
  Tf(speed);
  THf(speed);
  Ob(speed);
  Fb(speed);
}

void Mecanum_Movment::R(int speed)
{
  Tb(speed);
  THb(speed);
  Of(speed);
  Ff(speed);
}
void Mecanum_Movment::stopMotors(){
  Of(0);
  Tf(0);
  THf(0);
  Ff(0);
}
void Mecanum_Movment::DfL(int speed) {
  Of(speed);
  Ff(speed);
  Tb(speed);
  THb(speed);
}

void Mecanum_Movment::DfR(int speed) {
  Ob(speed);
  Fb(speed);
  Tf(speed);
  THf(speed);
}
void Mecanum_Movment::DbL(int speed) {
  Ob(speed);
  Fb(speed);
  Tb(speed);
  THb(speed);
}

void Mecanum_Movment::DbR(int speed) {
  Of(speed);
  Ff(speed);
  Tf(speed);
  THf(speed);
}

//Basic wheels functions.
void Mecanum_Movment::Of(int speed){
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  analogWrite(m1e, speed);
}

void Mecanum_Movment::Ob(int speed){
  digitalWrite(m1b, HIGH);
  digitalWrite(m1a, LOW);
  analogWrite(m1e, speed);
}

void Mecanum_Movment::Tf(int speed){
  digitalWrite(m2b, HIGH);
  digitalWrite(m2a, LOW);
  analogWrite(m2e, speed);
}

void Mecanum_Movment::Tb(int speed){
  digitalWrite(m2b, LOW);
  digitalWrite(m2a, HIGH);
  analogWrite(m2e, speed);
}

void Mecanum_Movment::THf(int speed){
  digitalWrite(m3a, HIGH);
  digitalWrite(m3b, LOW);
  analogWrite(m3e, speed);
}

void Mecanum_Movment::THb(int speed){
  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  analogWrite(m3e, speed);
}

void Mecanum_Movment::Ff(int speed){
  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(m4e, speed);
}

void Mecanum_Movment::Fb(int speed){
  digitalWrite(m4a, HIGH);
  digitalWrite(m4b, LOW);
  analogWrite(m4e, speed);
}


