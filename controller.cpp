#include "controller.h"
#include "I2Cdev.h"

Servo moto1, moto2, moto3, moto4;
void Moto::engineInit()
{
  moto1.attach(9);
  moto2.attach(10);
  moto3.attach(11);
  moto4.attach(12);
}

void Moto::engineArm()
{
  Serial.println("Engine Armed...");

  //digitalWrite(13, HIGH);

  writeToMotor(ARM, ARM, ARM, ARM);
}

void Moto::engineStart()
{
  Serial.println("Engine Starting... ");
  // Write to Servo
  writeToMotor(STARTUP, STARTUP, STARTUP, STARTUP);
}

void Moto::engineLock()
{
  Serial.println("Engine Locked...");
  //void writeToMotor(int param);
  writeToMotor(LOCK, LOCK, LOCK, LOCK);
}

void Moto::writeToMotor(int moto1var, int moto2var, int moto3var, int moto4var)
{
  moto1.write(moto1var);
  delay(500);
  moto2.write(moto2var);
  delay(500);
  moto3.write(moto3var);
  delay(500);
  moto4.write(moto4var);
  delay(500);
}


