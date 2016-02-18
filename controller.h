#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "Servo.h"

class Moto {
  public:
    Servo moto1, moto2, moto3, moto4;
    // Define throttle
#define LOCK 0
#define ARM 14
#define STARTUP 58
#define TAKEOFF 80
  public:
    void engineInit();
    void engineArm();
    void engineStart();
    void engineLock();
    void writeToMotor(int, int, int, int);
};
#endif
