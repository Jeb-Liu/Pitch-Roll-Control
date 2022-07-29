
#define MPU_9250_ADDRESS  0x68
#define MPU_9250_ADDRESS2  0x69
#define NO1_MPU_9250  0
#define NO2_MPU_9250  1
#define SMPLRT_DIV  0x19 //陀螺仪采样率典型值为0X07 1000/(1+7)=125HZ
#define CONFIG 0x1A //低通滤波器  典型值0x06 5hz
#define GYRO_CONFIG 0x1B //陀螺仪测量范围 0X18 正负2000度
#define ACCEL_CONFIG 0x1C //加速度计测量范围 0X18 正负16g
#define ACCEL_CONFIG2 0x1D //加速度计低通滤波器 0x06 5hz
#define PWR_MGMT_1 0x6B        //电源管理1 
#define PWR_MGMT_2 0x6C        //电源管理2
#define ACCEL_XOUT_H 0x3B      //加速度计输出数据
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41        //温度计输出数据
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

int avgNumber = 1000;
double dt, oldMillis;


long ACCEL_X, ACCEL_Y, ACCEL_Z;
double G_FORCE_X, G_FORCE_Y, G_FORCE_Z;
double GFORCE_SUM[3]={0,0,-9.81*1000};//avgnumber=1000
double GFORCE_OFFSET[3]={0,0,0};

long GYRO_X, GYRO_Y, GYRO_Z;
double ROT_X, ROT_Y, ROT_Z;
double ROT_SUM[3]={0,0,0};
double ROT_OFFSET[3]={0,0,0};

double accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;
double roll_fin, pitch_fin, yaw_fin;
double xk, yk, zk;

double a0_X, a0_Y, a0_Z;
double v0_X, v0_Y, v0_Z, v_X, v_Y, v_Z;
double s0_X, s0_Y, s0_Z, s_X, s_Y, s_Z;

long MAG_X, MAG_Y, MAG_Z;


void setupMPU( int I2C_ADDRESS )  //初始化MPU9250
{

  Wire.beginTransmission( I2C_ADDRESS );  //向0x68开始一次传输数据，发送一个I2C开始字符
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);//MPU WAKE UP
  Wire.endTransmission();

  Wire.beginTransmission( I2C_ADDRESS );  // enable accelerometer and gyro
  Wire.write(PWR_MGMT_2);
  Wire.write(0x00);//MPU WAKE UP
  Wire.endTransmission();

  Wire.beginTransmission( I2C_ADDRESS );
  Wire.write(SMPLRT_DIV);
  Wire.write(0x07);
  Wire.endTransmission();

  Wire.beginTransmission( I2C_ADDRESS );
  Wire.write(CONFIG);
  Wire.write(0x06);
  Wire.endTransmission();

  Wire.beginTransmission( I2C_ADDRESS );
  Wire.write(ACCEL_CONFIG);
//Wire.write(0x00);//FS_SEL=0,FULL SCALE RANGE = +/-2g[DEGREE/SEC]
//Wire.write(0x08);//FS_SEL=1,FULL SCALE RANGE = +/-4g[DEGREE/SEC]
//Wire.write(0x10);//FS_SEL=2,FULL SCALE RANGE = +/-8g[DEGREE/SEC]
  Wire.write(0x18);//FS_SEL=3,FULL SCALE RANGE = +/-16g[DEGREE/SEC]
  Wire.endTransmission();

  Wire.beginTransmission( I2C_ADDRESS );
  Wire.write(GYRO_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission();
}

/*
 * ==> RECOED DATA
 */
 
void recordAccelRegister( int I2C_ADDRESS )  //获取加速度计数据
{
  Wire.beginTransmission( I2C_ADDRESS ); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write( ACCEL_XOUT_H );
  Wire.endTransmission();
  Wire.requestFrom( I2C_ADDRESS, 6);
  while( Wire.available()<6 );
  ACCEL_X = Wire.read()<<8|Wire.read();
  ACCEL_Y = Wire.read()<<8|Wire.read();
  ACCEL_Z = Wire.read()<<8|Wire.read(); 
}
void recordGryoRegister( int I2C_ADDRESS )  //获取角速度计数据
{
  Wire.beginTransmission( I2C_ADDRESS ); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write( GYRO_XOUT_H );
  Wire.endTransmission();
  Wire.requestFrom( I2C_ADDRESS, 6 );
  while( Wire.available()<6 );
  GYRO_X = Wire.read()<<8|Wire.read();
  GYRO_Y = Wire.read()<<8|Wire.read();
  GYRO_Z = Wire.read()<<8|Wire.read();
}
void recordMagRegister( int I2C_ADDRESS )  //获取地磁计数据
{
  Wire.beginTransmission( I2C_ADDRESS ); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write( 0x49 );
  Wire.endTransmission();
  Wire.requestFrom( I2C_ADDRESS, 6 );
  while( Wire.available()<6 );
  MAG_X = Wire.read()<<8|Wire.read();
  MAG_Y = Wire.read()<<8|Wire.read();
  MAG_Z = Wire.read()<<8|Wire.read();
}

/*
 * ==>PROCESS DATA
 */
void processAccelData() //计算真实加速度 m/s^2
{
  G_FORCE_X = 16*9.798*ACCEL_X / 32767.5 - GFORCE_OFFSET[0];
  G_FORCE_Y = 16*9.798*ACCEL_Y / 32767.5 - GFORCE_OFFSET[1];
  G_FORCE_Z = 16*9.798*ACCEL_Z / 32767.5 - GFORCE_OFFSET[2];
}

void processGryoData()  //计算真实角速度 deg/s
{
  ROT_X = 2000.0*GYRO_X / 32767.5 - ROT_OFFSET[0];
  ROT_Y = 2000.0*GYRO_Y / 32767.5 - ROT_OFFSET[1];
  ROT_Z = 2000.0*GYRO_Z / 32767.5 - ROT_OFFSET[2];
}

void processAccelFix( int I2C_ADDRESS )  //加速度计 数据修正项
{
  for (int i=0; i<avgNumber; i++){
    recordAccelRegister( I2C_ADDRESS );
    processAccelData();
    GFORCE_SUM[0] = GFORCE_SUM[0] + G_FORCE_X;
    GFORCE_SUM[1] = GFORCE_SUM[1] + G_FORCE_Y;
    GFORCE_SUM[2] = GFORCE_SUM[2] + G_FORCE_Z;
  }
  GFORCE_OFFSET[0] = GFORCE_SUM[0] / (double)avgNumber;
  GFORCE_OFFSET[1] = GFORCE_SUM[1] / (double)avgNumber;
  GFORCE_OFFSET[2] = GFORCE_SUM[2] / (double)avgNumber;
}
void processGryoFix( int I2C_ADDRESS )  //角速度计 数据修正项
{
  for (int i=0; i<avgNumber; i++){
    recordGryoRegister( I2C_ADDRESS );
    processGryoData();
    ROT_SUM[0] = ROT_SUM[0] + ROT_X;
    ROT_SUM[1] = ROT_SUM[1] + ROT_Y;
    ROT_SUM[2] = ROT_SUM[2] + ROT_Z;
  }
  ROT_OFFSET[0] = ROT_SUM[0] / (double)avgNumber;
  ROT_OFFSET[1] = ROT_SUM[1] / (double)avgNumber;
  ROT_OFFSET[2] = ROT_SUM[2] / (double)avgNumber;
}

void processAccAngle() //通过加速度数据计算姿态角
{
  accAngleX = (atan2(G_FORCE_Y,G_FORCE_Z) * 180 / PI);//
  accAngleY = (atan2(-1 * G_FORCE_X,sqrt(pow(G_FORCE_Y, 2) + pow(G_FORCE_Z, 2))) * 180 / PI);
  
}
void processGryoAngle()   //通过角速度数据计算姿态角
{
  gyroAngleX = roll_fin + ROT_X * dt; // deg/s * s = deg
  gyroAngleY = pitch_fin + ROT_Y * dt;
}

void processEulerAngle()  //complementary filter融合姿态角
{
  yaw_fin =  yaw_fin + ROT_Z*dt;    //没磁力计暂时修正不了
  if(abs(ROT_X)>0.3||abs(ROT_Y)>0.3||abs(ROT_Z)>0.3){
    roll_fin = 0.9*gyroAngleX + 0.1*accAngleX;    //highpass filter to gyro data
    pitch_fin = 0.9*gyroAngleY + 0.1*accAngleY;   //lowpass filter to acc data
  } else {
    roll_fin = 0.1*gyroAngleX + 0.9*accAngleX;
    pitch_fin = 0.1*gyroAngleY + 0.9*accAngleY;
  }
}
void velocityFix()  //速度位移初始化
{
  processAccelData();
  a0_X = G_FORCE_X;
  a0_Y = G_FORCE_Y;
  a0_Z = G_FORCE_Z;
  
  v0_X = 0;
  v0_Y = 0;
  v0_Z = 0;

  s0_X = 0;
  s0_Y = 0;
  s0_Z = 0;
  
}

void processVelocity() //计算速度位移 线性插值
{
  v_X = v0_X + a0_X*dt + 0.5 * ( G_FORCE_X - a0_X);
  v_Y = v0_Y + a0_Y*dt + 0.5 * ( G_FORCE_Y - a0_Y);
  v_Z = v0_Z + a0_Z*dt + 0.5 * ( (G_FORCE_Z + 9.81) - a0_Z);

  s_X = s0_X + v0_X*dt + 0.5 * ( v_X - v0_X);
  s_Y = s0_Y + v0_Y*dt + 0.5 * ( v_Y - v0_Y);
  s_Z = s0_Z + v0_Z*dt + 0.5 * ( v_Z - v0_Z);

  v0_X = v_X;
  v0_Y = v_Y;
  v0_Z = v_Z;

  s0_X = s_X;
  s0_Y = s_Y;
  s0_Z = s_Z;
}
