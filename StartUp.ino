#include "freeram.h"

#include "mpu.h"
#include "I2Cdev.h"
#include "math.h"
#include "Servo.h"

int ret;
Servo moto1, moto2, moto3, moto4;

// Define throttle
int lock = 0;
int arm = 14;
int startup = 58;
int takeOff = 80;

void setup() {
  Fastwire::setup(400, 0);
  Serial.begin(9600);
  ret = mympu_open(200);
  Serial.print("MPU init: "); Serial.println(ret);
  Serial.print("Free mem: "); Serial.println(freeRam());

  moto1.attach(9);
  moto2.attach(10);
  moto3.attach(11);
  moto4.attach(12);

  moto1.write(0);
  delay(500);
  moto2.write(0);
  delay(500);
  moto3.write(0);
  delay(500);
  moto4.write(0);
  delay(500);

  engineArm();
  engineStart();



  Serial.print("Servo setup completely!");
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() {
  //  ret = mympu_update();
  mympu_update();
  //  switch (ret) {
  //    case 0: c++; break;
  //    case 1: np++; return;
  //    case 2: err_o++; return;
  //    case 3: err_c++; return;
  //    default:
  //      Serial.print("READ ERROR!  ");
  //      Serial.println(ret);
  //      return;
  //  }

  if (!(c % 25)) {
    //Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
    Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
    Serial.print(" R: "); Serial.print(mympu.ypr[1]);
    Serial.print(" P: "); Serial.print(mympu.ypr[2]);
    Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
    Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
    Serial.print(" gr: "); Serial.println(mympu.gyro[2]);
    CONTROL();
  }
}

float exp_angleX, exp_angleY, exp_angleZ;
float rc_roll, rc_pitch, rc_yaw;
float errorX, errorY, errorZ;
float p = 0.1, i = 0.1, d = 0.1;
float pid_rool_out, pid_pitch_out, pid_yaw_out;
unsigned int moto1thr, moto2thr, moto3thr, moto4thr;

void GET_EXPRAD(void)
{
  exp_angleX = (float)(-(rc_pitch - 1500) / 30.0f);
  exp_angleY = (float)(-(rc_roll - 1500) / 30.0f);
  exp_angleZ = (float)((rc_yaw - 1500) / 30.0f);
  //        printf("%f %f\n",MPU6050_ACC_LAST.Y*cos(Q_ANGLE.X/57.3)-MPU6050_ACC_LAST.Z*sin(Q_ANGLE.X/57.3),MPU6050_ACC_LAST.X*cos(-Q_ANGLE.Y/57.3)-MPU6050_ACC_LAST.Z*sin(-Q_ANGLE.Y/57.3));
  //        DIF_ANGLE.X = (ACC_AVG.Y*cos(Q_ANGLE.X/57.3)-ACC_AVG.Z*sin(Q_ANGLE.X/57.3))/500;
  //        DIF_ANGLE.Y = (ACC_AVG.X*cos(-Q_ANGLE.Y/57.3)-ACC_AVG.Z*sin(-Q_ANGLE.Y/57.3)/500);
  errorX = exp_angleX - mympu.ypr[2];
  errorY = exp_angleY - mympu.ypr[1];
  errorZ = exp_angleZ - mympu.ypr[0];

  //        DIF_ANGLE.Z = EXP_ANGLE.Z - GYRO_I[0].Z;
  //        DIF_ANGLE.X = EXP_ANGLE.X - GYRO_I[0].X;
  //        DIF_ANGLE.Y = EXP_ANGLE.Y - GYRO_I[0].Y;
  //        DIF_ANGLE.Z = EXP_ANGLE.Z - GYRO_I[0].Z;
}

void CONTROL(void)
{
  static float thr = 0, rool = 0, pitch = 0, yaw = 0;
  static float rool_i = 0, pitch_i = 0;
  static float rool_dif = 0, pitch_dif = 0;
  static float rool_speed_dif = 0, pitch_speed_dif = 0;
  float rool_out, pitch_out;
  unsigned int THROTTLE, throttle_in;

  GET_EXPRAD();

  rool         = p * errorY;
  rool_i += i * errorY * 0.002;
  rool_i = constrain(rool_i, -30, 30);
  rool += rool_i;
  rool += d * (errorY - rool_dif) * 500;
  rool_dif = errorY;
  ///////////
  pitch         = +p * errorX;
  pitch_i += i * errorX * 0.002;
  pitch_i = constrain(pitch, -30, 30);
  pitch += pitch_i;
  pitch += d * (errorX - pitch_dif) * 500;
  pitch_dif = errorX;
  ///////////
  //
  //rool -= GYRO_AVG.X * Gyro_G;
  rool_out = p * rool;
  rool_out += d * (rool - rool_speed_dif) * 500;
  rool_speed_dif = rool;

  //pitch -= GYRO_AVG.Y * Gyro_G;
  pitch_out = p * pitch;
  pitch_out += d * (pitch - pitch_speed_dif) * 500;
  pitch_speed_dif = pitch;

  //        rool=PID_PIT.I*(rool-GYRO_AVG.X* Gyro_G);
  //        pitch=PID_PIT.I*(pitch-GYRO_AVG.Y* Gyro_G);

  //        PID_YAW.dout = 20 * (MPU6050_GYRO_LAST.Z* Gyro_G-(Rc_Get.YAW-1500)/10);
  //PID_YAW.dout = 10 * (GYRO_AVG.Z * Gyro_G - (Rc_Get.YAW - 1500) / 10);

  pid_rool_out = rool_out;
  pid_pitch_out = pitch_out;
  //pid_yaw_out = PID_YAW.dout;

  /////////////
  //        GYRO_I[0].Z += EXP_ANGLE.Z/3000;
  //        yaw = -10 * GYRO_I[0].Z;
  //
  //        yaw -= 3 * GYRO_F.Z;
  THROTTLE = throttle_in;
  if (THROTTLE > 1050)
  {
    //                if(THROTTLE>1950)
    //                {
    //                        THROTTLE=1950;
    //                }
    //THROTTLE = THROTTLE / cos(Q_ANGLE.X / 57.3) / cos(Q_ANGLE.Y / 57.3);

    moto1thr = THROTTLE - 1000 + (int16_t)pid_rool_out - (int16_t)pid_pitch_out - (int16_t)pid_yaw_out;
    moto2thr = THROTTLE - 1000 + (int16_t)pid_rool_out + (int16_t)pid_pitch_out + (int16_t)pid_yaw_out;
    moto3thr = THROTTLE - 1000 - (int16_t)pid_rool_out + (int16_t)pid_pitch_out - (int16_t)pid_yaw_out;
    moto4thr = THROTTLE - 1000 - (int16_t)pid_rool_out - (int16_t)pid_pitch_out + (int16_t)pid_yaw_out;
    //Serial.print(pulseIn(90L, 1L));
    moto1.write(map(moto1thr, 1000, 2000, 59, 144));
    moto2.write(map(moto2thr, 1000, 2000, 59, 144));
    moto3.write(map(moto3thr, 1000, 2000, 59, 144));
    moto4.write(map(moto4thr, 1000, 2000, 59, 144));
  }
  //  else
  //  {
  //    moto1 = 0;
  //    moto2 = 0;
  //    moto3 = 0;
  //    moto4 = 0;
  //  }

  //  if (Q_ANGLE.X > 45 || Q_ANGLE.Y > 45 || Q_ANGLE.X < -45 || Q_ANGLE.Y < -45)
  //  {
  //    ARMED = 0;
  //    LED3_OFF;
  //  }
  Serial.print(" moto1="); Serial.print( map(moto1thr, 1000, 2000, 59, 144));
  Serial.print(" moto2="); Serial.print( map(moto2thr, 1000, 2000, 59, 144));
  Serial.print(" moto3="); Serial.print( map(moto3thr, 1000, 2000, 59, 144));
  Serial.print(" moto4="); Serial.print( map(moto4thr, 1000, 2000, 59, 144));
  //  if (ARMED)        MOTO_PWMRFLASH(moto1, moto2, moto3, moto4);
  //  else                        MOTO_PWMRFLASH(0, 0, 0, 0);
}

void engineArm()
{
  Serial.println("Engine Armed...");

  //digitalWrite(13, HIGH);

  writeToMotor(14,14,14,14);
}

void engineStart()
{
  Serial.println("Engine Starting... ");
  // Write to Servo
  writeToMotor(startup,startup,startup,startup);
}

void engineLock()
{
  Serial.println("Engine Locked...");
  //void writeToMotor(int param);
  writeToMotor(lock, lock, lock, lock);
}

void writeToMotor(int moto1, int moto2, int moto3, int moto4)
{
  moto1.write(moto1);
  delay(200);
  moto2.write(moto2);
    delay(200);
  moto3.write(moto3);
    delay(200);
      delay(200);
  moto4.write(moto4);
}


