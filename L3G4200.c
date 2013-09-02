#include "msp430x54x.h"
#include "L3G4200.h"
#include "I2C.h"//…Ë±∏µÿ÷∑0x69
void L3G4200_init()
{
   I2C_write(0x69,CTRL_REG1, 0x4f);   //
   I2C_write(0x69,CTRL_REG2, 0x00);   //
   I2C_write(0x69,CTRL_REG3, 0x08);   //
   I2C_write(0x69,CTRL_REG4, 0xF0);   //+-2000dps
   I2C_write(0x69,CTRL_REG5, 0x10);   //enable High Pass Filter
}
/*
signed int L3G4200_GetX()
{
  union
  {
    int x;
    unsigned char val[2];
  };
  val[0]=I2C_read(0x69,OUT_X_L);
  val[1]=I2C_read(0x69,OUT_X_H);
  return x;
}
signed int L3G4200_GetY()
{  
  union
  {
    int y;
    unsigned char val[2];
  };
  val[0]=I2C_read(0x69,OUT_Y_L);
  val[1]=I2C_read(0x69,OUT_Y_H);
  return y;
}
signed int L3G4200_GetZ()
{
  union
  {
    int z;
    unsigned char val[2];
  };
  val[0]=I2C_read(0x69,OUT_Z_L);
  val[1]=I2C_read(0x69,OUT_Z_H);
  return z;
}*/