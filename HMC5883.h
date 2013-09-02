#ifndef HMC5883_H_
#define HMC5883_H_
#define	SlaveAddress   0x3C	  //定义器件在IIC总线中的从地址
extern void HMC5883_init(void);
extern void Multiple_read_HMC5883(unsigned char *BUF);
extern int HMC5883_Get_x(void);//获取x方向值
extern int HMC5883_Get_y(void);//获取y方向值
extern int HMC5883_Get_z(void);//获取z方向值
unsigned int HMC5883_Get_angle(void);//获取角度值,没搞好
#endif 