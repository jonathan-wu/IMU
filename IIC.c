#include "msp430x54x.h"
#include "UCS.h"
#include "IIC.h"
void IIC_init(void)
{
  scl_OUT;  
  DIR_OUT;  
  for( char i = 0 ; i < 9 ; i++ )
  {
    scl_1;
    __delay_cycles(8000);
    scl_0;
    __delay_cycles(8000);
  }
  scl_1;
  delay_us(5);
  sda_1;
  delay_us(5);
}
void IIC_start(void)
{
	scl_1; 
	delay_us(5);
	sda_1;
	delay_us(5); 
	sda_0; 
	delay_us(5); 
	scl_0; 
	delay_us(5);
}
void IIC_stop() 
{
	scl_0;
	delay_us(5);
	sda_0; 
	delay_us(5);
	scl_1;
	delay_us(5);
	sda_1;
	delay_us(5);
}
void IIC_writex(unsigned char j)
{  
	unsigned char i,temp;
	temp = j;
	for(i=0; i<8; i++)
	{
		scl_0; 
		delay_us(5); 
                if(temp & 0x80) sda_1;
                else sda_0;
		delay_us(5);
		scl_1; 
		delay_us(5);
		temp = temp << 1; 
	}
	scl_0;
	delay_us(5); 
}
unsigned char IIC_readx(void)
{
	unsigned char i, j, k = 0;
	for(i=0; i<8; i++)
	{
		scl_0;
		delay_us(5);	
                DIR_IN;
		if(sda_H)
		{
			j = 1;
		}
		else j = 0;
                DIR_OUT;
		k = (k << 1) | j; 
		scl_1;
		delay_us(5);
	} 
	return(k);
}
void IIC_ack(void)
{
	unsigned char i = 0;
	scl_1;
	delay_us(5);
        DIR_IN;
	while((sda_H) && (i < 255)) 
	{
		i++;
	}
        DIR_OUT;
	scl_0;
	delay_us(5);
}
signed int IIC_readword(unsigned char Address,unsigned char address)
{
        union
        {
          signed int x;
          unsigned char i[2];
        };
	IIC_start();
	IIC_writex(Address);
	IIC_ack();
	IIC_writex(address);
	IIC_ack();
	IIC_start();
	IIC_writex(Address+1);
	IIC_ack();
	i[0] = IIC_readx();
        sda_1;
        scl_1;
        delay_us(5);
        scl_0;
        delay_us(5);
        i[1] = IIC_readx();
	IIC_stop();
	return(x);
}

unsigned char IIC_read(unsigned char Address,unsigned char address)
{
	unsigned char i;
	IIC_start();
	IIC_writex(Address);
	IIC_ack();
	IIC_writex(address);
	IIC_ack();
	IIC_start();
	IIC_writex(Address+1);
	IIC_ack();
	i = IIC_readx();
	IIC_stop();
	return(i);
}
void IIC_write(unsigned char Address,unsigned char address, unsigned char info)
{
	IIC_start();
	IIC_writex(Address);
	IIC_ack();
	IIC_writex(address);
	IIC_ack();
	IIC_writex(info);
	IIC_ack();
	IIC_stop();
}