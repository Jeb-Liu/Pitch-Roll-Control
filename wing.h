#include <Servo.h>
//舵机定义
Servo flap_l;
Servo flap_r;
Servo horizontal;
Servo vertical;

void servoPin()
{
  //舵机pin定义
  flap_l.attach(5);
  flap_r.attach(6);
  horizontal.attach(10);
  vertical.attach(11);
  }
