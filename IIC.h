#ifndef IIC_H_
#define IIC_H_
#define scl_1 P2OUT |= BIT1  //SCL
#define scl_0 P2OUT &=~BIT1
#define scl_OUT P2DIR |=BIT1
#define sda_1 P2OUT |= BIT2  //SDA
#define sda_0 P2OUT &=~BIT2
#define sda_H P2IN & BIT2
#define DIR_IN P2DIR &=~BIT2
#define DIR_OUT P2DIR |=BIT2
//#define Address 0xD0        //设备地址
extern void IIC_init(void);
extern void IIC_start(void);
extern void IIC_stop();
extern void IIC_writex(unsigned char j);
extern unsigned char IIC_readx(void);
extern void IIC_ack(void);
extern unsigned char IIC_read(unsigned char Address,unsigned char address);             //读地址为address的值
extern signed int IIC_readword(unsigned char Address,unsigned char address);
extern void IIC_write(unsigned char Address,unsigned char address, unsigned char info); //写值info到地址address处
#endif 
