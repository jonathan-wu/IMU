
#include "msp430f5438.h"
#include "msp430_math.h"
#include "Global.h"
#include "UCS.h"
#include "WDT.h"
#include "TimerA1.h"
#include "UART.h"

#include "I2C.h"
#include "ADXL345.h"
#include "L3G4200.h"
#include "HMC5883.h"

#include "IMU.h"

unsigned long nowTime,startTime;
unsigned char command;
Euler_struct a;
void main( void )
{
  WDT_init();
  
  UCS_init();
  
  TimerA1_init();
  
  I2C_init();
  
  UART_init(UCA1,115200);
  
  _EINT();
  
  IMU_init();  
  
  IMU_calibrate();
  
  UART_sendstr(UCA1,"Ready.");
  
  while(UCA1_GET_CHAR(&command));
  
  startTime=TimeBase;
  
  while(1)
  {
    if(nowTime!=TimeBase)
    {
      nowTime=TimeBase;
      
      if((nowTime % 5==0)&&(nowTime < 60000+startTime))
      {
        IMU_getdata();
        IMUupdate();
        a=IMU_getEuler();
        UART_sendlong(UCA1,(signed long)(a.yaw*100000)+1000000);                
        UART_sendstr(UCA1," ");            
        UART_sendlong(UCA1,(signed long)(a.pitch*100000)+1000000);                
        UART_sendstr(UCA1," ");            
        UART_sendlong(UCA1,(signed long)(a.roll*100000)+1000000);                
        UART_sendstr(UCA1," ");              /*
        UART_sendint(UCA1,(signed int)(q0*1000)+32768);
        UART_sendstr(UCA1," ");
        UART_sendint(UCA1,(signed int)(q1*1000)+32768);
        UART_sendstr(UCA1," ");
        UART_sendint(UCA1,(signed int)(q2*1000)+32768);
        UART_sendstr(UCA1," ");
        UART_sendint(UCA1,(signed int)(q3*1000)+32768);
        UART_sendstr(UCA1," ");*/
      }
    }

  }

}
