//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "IMU.h"
#include "msp430_math.h"
#include "I2C.h"
#include "UART.h"

//----------------------------------------------------------------------------------------------------
// Definitions

#define Kp 0.001f          	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0001f 		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f		// half the sample period

//---------------------------------------------------------------------------------------------------
// Variable definitions

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error
signed int ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw;
signed int ax_filted,ay_filted,az_filted,gx_filted,gy_filted,gz_filted;
signed int ax_bias,ay_bias,az_bias,gx_bias,gy_bias,gz_bias;

//====================================================================================================
// Function
//====================================================================================================

void IMUupdate() 
{
    float gx, gy, gz, ax, ay, az;  
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;         
    
    gx = gx_filted*0.00106526443603169529841533860381f;
    gy = gy_filted*0.00106526443603169529841533860381f;
    gz = gz_filted*0.00106526443603169529841533860381f;    
    
    ax = ax_filted;
    ay = ay_filted;
    az = -az_filted;    
    
    // normalise the measurements
    norm = sqrt(ax*ax + ay*ay + az*az);       
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;      
    
    // estimated direction of gravity
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    // error is sum of cross product between reference direction of field and direction measured by sensor
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    // integral error scaled integral gain
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    
    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
    
    // integrate quaternion rate and normalise
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    
    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
}

//====================================================================================================
//Reimplemented Functions by WuYingnan
//====================================================================================================
signed int abs(signed int a)
{
  if (a<0)
    return -a;
  else
    return a;
}

void IMU_init()
{
//HMC5883_init  
  I2C_write(HMC5883_Address,0x00,0x78);
  I2C_write(HMC5883_Address,0x01,0x40);
  I2C_write(HMC5883_Address,0x02,0x00);
  
//ADXL345_init
  I2C_write(ADXL345_Address,0x31,0x08);   //测量范围,正负2g，13位模式 分辨率4mg
  I2C_write(ADXL345_Address,0x2C,0x08);   //速率设定为12.5 参考pdf13页
  I2C_write(ADXL345_Address,0x2D,0x08);   //选择电源模式   参考pdf24页  
  
//L3G2440_init  
  I2C_write(L3G2440_Address,CTRL_REG1, 0x4f);   //
  I2C_write(L3G2440_Address,CTRL_REG2, 0x00);   //
  I2C_write(L3G2440_Address,CTRL_REG3, 0x08);   //
  I2C_write(L3G2440_Address,CTRL_REG4, 0xF0);   //+-2000dps
  I2C_write(L3G2440_Address,CTRL_REG5, 0x10);   //enable High Pass Filter
}

void IMU_getdata()
{
  
/*    
    I2C_read(0x1e,0x03,2,(unsigned char *)&mx);
    mx=(unsigned)mx/256+((unsigned)mx%256)*256;
//    UART_sendint(UCA1,mx+32768);        
//    UART_sendstr(UCA1," ");
    I2C_read(0x1e,0x05,2,(unsigned char *)&my);
    my=(unsigned)my/256+((unsigned)my%256)*256;
//    UART_sendint(UCA1,my+32768);        
//    UART_sendstr(UCA1," ");        
    I2C_read(0x1e,0x07,2,(unsigned char *)&mz);
    mz=(unsigned)mz/256+((unsigned)mz%256)*256;    
//    UART_sendint(UCA1,mz+32768);                
//    UART_sendstr(UCA1," ");    
*/    
        
    I2C_read(0x53,0x32,2,(unsigned char *)&ax_raw);
    ax_raw=(unsigned)ax_raw/256+((unsigned)ax_raw%256)*256-ax_bias;
  
    I2C_read(0x53,0x34,2,(unsigned char *)&ay_raw);
    ay_raw=(unsigned)ay_raw/256+((unsigned)ay_raw%256)*256-ay_bias;
    
    I2C_read(0x53,0x36,2,(unsigned char *)&az_raw);
    az_raw=(unsigned)az_raw/256+((unsigned)az_raw%256)*256-az_bias;    
    
    I2C_read(0x69,0x28|0x80,2,(unsigned char *)&gx_raw);
    gx_raw=(unsigned)gx_raw/256+((unsigned)gx_raw%256)*256-gx_bias;
    
    I2C_read(0x69,0x2a|0x80,2,(unsigned char *)&gy_raw);
    gy_raw=(unsigned)gy_raw/256+((unsigned)gy_raw%256)*256-gy_bias;

    I2C_read(0x69,0x2c|0x80,2,(unsigned char *)&gz_raw);
    gz_raw=(unsigned)gz_raw/256+((unsigned)gz_raw%256)*256-gz_bias;    

    ax_filted+=(ax_raw-ax_filted)/5;
    ay_filted+=(ay_raw-ay_filted)/5;
    az_filted+=(az_raw-az_filted)/5;

//    if (abs(gx_raw-gx_filted)<100)
      gx_filted+=(gx_raw-gx_filted)/2;
//    if (abs(gy_raw-gy_filted)<100)    
      gy_filted+=(gy_raw-gy_filted)/2;
//    if (abs(gz_raw-gz_filted)<100)    
      gz_filted+=(gz_raw-gz_filted)/2;   

    /*
    UART_sendint(UCA1,ax_filted+32768);                
    UART_sendstr(UCA1," ");          
    UART_sendint(UCA1,ay_filted+32768);                
    UART_sendstr(UCA1," ");     
    UART_sendint(UCA1,az_filted+32768);                
    UART_sendstr(UCA1," ");    
    UART_sendint(UCA1,gx_filted+32768);                
    UART_sendstr(UCA1," ");       
    UART_sendint(UCA1,gy_filted+32768);               
    UART_sendstr(UCA1," ");    
    UART_sendint(UCA1,gz_filted+32768);                
    UART_sendstr(UCA1," ");    */
}


void IMU_calibrate()
{
  signed long ax_sum=0,ay_sum=0,az_sum=0,gx_sum=0,gy_sum=0,gz_sum=0;
  ax_bias=0;
  ay_bias=0;
  az_bias=0;
  gx_bias=0;
  gy_bias=0;
  gz_bias=0;  
  for(unsigned int i=0;i<1000;i++)
  {
    IMU_getdata();
    ax_sum+=ax_raw;
    ay_sum+=ay_raw;
    az_sum+=az_raw;
    gx_sum+=gx_raw;
    gy_sum+=gy_raw;
    gz_sum+=gz_raw;
  }
  ax_bias=ax_sum/1000;
  ay_bias=ay_sum/1000;
  az_bias=az_sum/1000+8192;
  gx_bias=gx_sum/1000;
  gy_bias=gy_sum/1000;
  gz_bias=gz_sum/1000;
}

void IMU_update()
{
  static long roll=0,pitch=0,yaw=0;
  
  roll += (long)gx_filted*2000/32768;
  pitch+= (long)gy_filted*2000/32768;
  yaw  += (long)gz_filted*2000/32768;
  
  UART_sendint(UCA1,(signed int)(roll)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(pitch)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(yaw)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(gx_filted)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(gy_filted)+32768);
  UART_sendstr(UCA1," ");
  UART_sendint(UCA1,(signed int)(gz_filted)+32768);
  UART_sendstr(UCA1," ");
}

Euler_struct IMU_getEuler()
{
    Euler_struct euler;
    euler.pitch=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
    euler.roll=asin(2*(q0*q2-q1*q3));
    euler.yaw=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
    return euler;
}
//====================================================================================================
// END OF CODE
//====================================================================================================
