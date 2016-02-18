#include "system_stats.h"
#include "mpu.h"
#include "I2Cdev.h"
#include "math.h"
#include "Servo.h"
#include "controller.h"

Moto motos;

void setup() {
  Fastwire::setup(400, 0);
  Serial.begin(9600);
  mympu_open(200);

  Serial.print("System Stats: Free Ram:"); Serial.println(freeRam());

  motos.engineInit();
//  delay(500);
  motos.engineArm();
//  delay(500);
  motos.engineStart();
//  delay(500);
  Serial.print("Servo setup completely!");
}

void loop() {
  mympu_update();

  //    if (!(c % 25)) {
  Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
  Serial.print(" R: "); Serial.print(mympu.ypr[1]);
  Serial.print(" P: "); Serial.print(mympu.ypr[2]);
  Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
  Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
  Serial.print(" gr: "); Serial.println(mympu.gyro[2]);

  //    }

}


