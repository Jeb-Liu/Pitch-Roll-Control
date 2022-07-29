#define kp_outter 0.1
#define ki_outter 1
#define kp_inner 0.2
#define ki_inner 0.1
#define kd_inner 0.01
double err_outter, err_inner;
double ref_ang, ref_angRate, last_angRate, ref_angAcc;

double pidServo(double dt, double input_ang, double est_curr_ang, double est_curr_angRate)
{
  err_outter = ref_ang + input_ang - est_curr_ang;
  ref_angRate = kp_outter * err_outter;

   
  err_inner = ref_angRate - est_curr_angRate;
  ref_angAcc = (kp_inner + kp_inner * dt ) * err_inner + kd_inner * (est_curr_angRate - last_angRate);
  last_angRate = est_curr_angRate;

  return ref_angAcc;
}
