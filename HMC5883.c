#include "msp430x54x.h"
#include "UCS.h"
#include "HMC5883.h"
#include "I2C.h"

void HMC5883_init(void)
{
  I2C_write(0x1e,0x00,0x78);
  I2C_write(0x1e,0x01,0x40);
  I2C_write(0x1e,0x02,0x00);
}/*
void Multiple_read_HMC5883(unsigned char *BUF)
{
  unsigned char i;
  for (i=0; i<8; i++)                      //连续读取6个地址数据，存储中BUF
  {
      BUF[i] = I2C_read(0x1e,0x03+i);          //BUF[0]存储数据
  }
}
int HMC5883_Get_x(void)
{
  union
  {
    int x;
    unsigned char val[2];
  };
  while(!(I2C_read(0x1e,0x09) & BIT0)); 
  val[1]=I2C_read(0x1e,0x03);
  val[0]=I2C_read(0x1e,0x04);
  return x;
}
int HMC5883_Get_y(void)
{
  union
  {
    int y;
    unsigned char val[2];
  };
  while(!(I2C_read(0x1e,0x09) & BIT0)); 
  val[1]=I2C_read(0x1e,0x05);
  val[0]=I2C_read(0x1e,0x06);
  return y;
}
int HMC5883_Get_z(void)
{
  union
  {
    int z;
    unsigned char val[2];
  };
  while(!(I2C_read(0x1e,0x09) & BIT0)); 
  val[1]=I2C_read(0x1e,0x07);
  val[0]=I2C_read(0x1e,0x08);
  return z;
}

//数据溢出没有解决
unsigned int HMC5883_Get_angle(void)
{
  signed int X_val,Y_val,Z_val;
  X_val=(HMC5883_Get_x()-141)*100/78;
  Y_val=HMC5883_Get_y();//没有使用
  Z_val=HMC5883_Get_z()-1319;
  unsigned int angle;
  if (X_val!=0)
  {
    //theta=atan((double)dirZ/(double)dirX);
    double k=(double)Z_val/(double)X_val;
    double t1_k=k*k;
    double t2_k=t1_k*k;
    double angle_r=k-t2_k/3+t1_k*t2_k/5;
    int angle_d=(signed int)(angle_r*5729)/100;
    if (X_val<0)
      angle=angle_d+180;
      //theta+=3.141;
    else if (Z_val<0)
      angle=(unsigned int)angle_d+360;
      //theta+=6.283;
    else
      angle=(unsigned int)angle_d;
  }
  else
  {
    if (Z_val>0)
      angle=90;
      //theta=1.570;
    else
      angle=270;
      //theta=4.712;
  }
  return angle;
}
*/