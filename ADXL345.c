#include "msp430x54x.h"
#include "UCS.h"
#include "ADXL345.h"
#include "I2C.h"
void ADXL345_init(void)
{
  I2C_write(ADXL345_Address,0x31,0x08);   //测量范围,正负2g，13位模式 分辨率4mg
  I2C_write(ADXL345_Address,0x2C,0x08);   //速率设定为12.5 参考pdf13页
  I2C_write(ADXL345_Address,0x2D,0x08);   //选择电源模式   参考pdf24页
  //I2C_write(ADXL345_Address,0x2E,0x80);   //使能 DATA_READY 中断
  I2C_write(ADXL345_Address,0x1E,0x00);   //X 偏移量 根据测试传感器的状态写入pdf29页
  I2C_write(ADXL345_Address,0x1F,0x00);   //Y 偏移量 根据测试传感器的状态写入pdf29页
  I2C_write(ADXL345_Address,0x20,0x00);   //Z 偏移量 根据测试传感器的状态写入pdf29页
  I2C_write(ADXL345_Address,0x2F,0x00);		//配置freefall 为int1脚
  I2C_write(ADXL345_Address,0x28,0x07);
  I2C_write(ADXL345_Address,0x29,0x04);
  I2C_write(ADXL345_Address,0x2E,0x00+BIT2);//开启freefall int
  
  
  
  P1DIR&=~BIT0;
  P1IFG = 0;
  P1IES&=~BIT0;//上升中断
  P1IE |= BIT0;//17号  
  
  delay_ms(10);
}
/*
signed int ADXL345_GetAX()
{
  union
  {
  signed int ax;
  unsigned char n[2];
  };
  n[0]=I2C_read(ADXL345_Address,ACCEL_XOUT_L);
  n[1]=I2C_read(ADXL345_Address,ACCEL_XOUT_H);
  return ax;
}
signed int ADXL345_GetAY()
{
  union
  {
  signed int ay;
  unsigned char n[2];
  };
  n[0]=I2C_read(ADXL345_Address,ACCEL_YOUT_L);
  n[1]=I2C_read(ADXL345_Address,ACCEL_YOUT_H);
  return ay;
}
signed int ADXL345_GetAZ()
{
  union
  {
  signed int az;
  unsigned char n[2];
  };
  n[0]=I2C_read(ADXL345_Address,ACCEL_ZOUT_L);
  n[1]=I2C_read(ADXL345_Address,ACCEL_ZOUT_H);
  return az;
}

#pragma vector = PORT1_VECTOR
__interrupt void P1ISR(void)
{
  switch(P1IV)
  {
  case 0x00:
    break;
  case 0x02:
    _NOP();
    break;
  }
}
*/