#include <Wire.h>
#include "mpu.h"
#include "kalman.h"
#include "pid.h"
#include "wing.h"
#include "gps.h"

double roll_control, kalman_roll_control;
double pitch_control, kalman_pitch_control;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  servoPin();
  
  //IMU初始化
  setupMPU( MPU_9250_ADDRESS );
  processGryoFix( MPU_9250_ADDRESS );
  processAccelFix( MPU_9250_ADDRESS );
}
  
void loop() {
  //计时
  dt = (millis() - oldMillis) / 1000; // Divide by 1000 to get seconds [s]
  oldMillis = millis();        // Previous time is stored before the actual time read

  
  
  //IMU数据
  recordAccelRegister( MPU_9250_ADDRESS );    //获取加速度计数据
  recordGryoRegister( MPU_9250_ADDRESS );     //获取角速度计数据
  processAccelData();       //计算加速度计数据
  processGryoData();        //计算角速度计数据
  processAccAngle();        //通过加速度数据计算姿态角
  processGryoAngle();       //通过角速度数据计算姿态角
  processEulerAngle();      //融合姿态角

  //kalman滤波
  xk = kalman(dt, xk, ROT_X, accAngleX);
  yk = kalman(dt, yk, ROT_Y, accAngleY);
  //zk = kalman(dt, zk, ROT_Z, accAngleZ); //magAngleZ

  roll_control = pidServo(dt, 0, xk, ROT_X);
  kalman_roll_control = kalman(dt, kalman_roll_control, roll_control, xk);
  pitch_control = pidServo(dt, 0, yk, ROT_Y);
  kalman_pitch_control = kalman(dt, kalman_pitch_control, pitch_control, yk);
  
  //舵机控制
  flap_l.write(90 + kalman_pitch_control);//
  flap_r.write(90 + kalman_pitch_control);//
  horizontal.write(90 - kalman_roll_control);//
  
  //输出数据
  Serial.print(oldMillis);
  Serial.print("\t");
  Serial.print(xk);
  Serial.print("\t");
  Serial.print(yk);
  Serial.print("\t");
  //getGpsData(); 
  delayMicroseconds(50*1000-dt*1000);
  Serial.println(dt*1000);

}









/*
 * ========================================FUNCTIONS=================================================
 *
 *===> RESTART
 */
 

  


/*
 * ==>PRINT DATA
 */
void printRawData()
{
  Serial.print(ACCEL_X,6);
  Serial.print("\t");
  Serial.print(ACCEL_Y,6);
  Serial.print("\t");
  Serial.print(-ACCEL_Z,6);
  Serial.print("\t");
  Serial.print(GYRO_X,6);
  Serial.print("\t");
  Serial.print(GYRO_Y,6);
  Serial.print("\t");
  Serial.print(GYRO_Z,6);
  Serial.print("\t");
}
void printGForceData()    //真实加速度  [m/s^2]
{
  Serial.print(G_FORCE_X,6);
  Serial.print("\t");
  Serial.print(G_FORCE_Y,6);
  Serial.print("\t");
  Serial.print(-G_FORCE_Z,6);
  Serial.print("\t");
}
void printRotData()     //真实角速度  [deg/s]
{
  Serial.print(ROT_X,6);
  Serial.print("\t");
  Serial.print(ROT_Y,6);
  Serial.print("\t");
  Serial.print(ROT_Z,6);
  Serial.print("\t");
}
void printAccAngle()    //姿态角(加速度)  [deg]
{
  Serial.print(accAngleX);
  Serial.print("\t");
  Serial.print(accAngleY);
  Serial.print("\t");
}
void printGryoAngle()    //姿态角(角速度)  [deg]
{
  Serial.print(gyroAngleX);
  Serial.print("\t");
  Serial.print(gyroAngleY);
  Serial.print("\t");
}
void printEulerAngle()    //最终姿态角  [deg]
{
  Serial.print(roll_fin);
  Serial.print("\t");
  Serial.print(pitch_fin);
  Serial.print("\t");
  Serial.print(yaw_fin);
}
void printMagData()
{
  
}
void printvelocity()    //速度位移  [m/s][m]
{
  Serial.print(v_X);
  Serial.print("\t");
  Serial.print(v_Y);
  Serial.print("\t");
  Serial.print(v_Z);
  Serial.print("\t");
  Serial.print(s_X);
  Serial.print("\t");
  Serial.print(s_Y);
  Serial.print("\t");
  Serial.print(s_Z);
}
