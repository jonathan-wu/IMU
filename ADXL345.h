#ifndef ADXL345_H_
#define ADXL345_H_
#define	ACCEL_XOUT_H	0x33
#define	ACCEL_XOUT_L	0x32
#define	ACCEL_YOUT_H	0x35
#define	ACCEL_YOUT_L	0x34
#define	ACCEL_ZOUT_H	0x37
#define	ACCEL_ZOUT_L	0x36
#define ADXL345_Address 0x53        //设备地址
extern void ADXL345_init(void);
extern signed int ADXL345_GetAX();//分辨率定长4mg
extern signed int ADXL345_GetAY();
extern signed int ADXL345_GetAZ();
#endif 
