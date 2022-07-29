#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

//矩阵定义
//111111111111111111111111111111111111
BLA::Matrix<2,2> Fk;
BLA::Matrix<2,2> B;
BLA::Matrix<2,1> xk0;
BLA::Matrix<2,1> uk;
BLA::Matrix<2,1> xk_hat;
//222222222222222222222222222222222222
BLA::Matrix<2,2> Pk_;
BLA::Matrix<2,2> Pk0;
BLA::Matrix<2,2> FkT;
BLA::Matrix<2,2> Qk={0.001,0,0,0.003};
//333333333333333333333333333333333333
BLA::Matrix<2,2> Kk;
BLA::Matrix<2,2> H={1,0,0,1};
BLA::Matrix<2,2> HPk_HTR;
BLA::Matrix<2,2> R= {0.1, 0, 0, 0.1};
//444444444
BLA::Matrix<2,1> xkhat;
BLA::Matrix<2,1> Zk;
//5555555555
BLA::Matrix<2,2> Pk;

double kalman(double dt, double xk, double rot_x, double accAngel_x)
{
  //矩阵代入
  Fk = {1, dt, 0, 1};
  B = {dt, 0, 0, 0};
  xk0 = {xk, 0.01};
  uk = {rot_x, 0};
  Pk0 = {0.02,0,0,0.02};
  Zk = {accAngel_x, 0.01};

  //计算
  //先验估计
  xk_hat = Fk * xk0 + B * uk;
  //先验误差协方差
  Qk = {0.001*dt ,0, 0 , 0.003*dt};
  Pk_ = Fk * Pk0 * (~Fk) + Qk;
  //卡尔曼增益
  HPk_HTR = H * Pk_ *(~H) + R;
  bool is_nonsingular = Invert(HPk_HTR);
  Kk = (HPk_HTR) * (Pk_ * H);
  //后验估计
  xkhat = xk_hat + Kk*Zk - Kk*H*xk_hat;
  //更新误差协方差
  Pk = Pk_ - Kk*H*Pk_;
  
  //更新数据
  return xkhat(0);

  }
