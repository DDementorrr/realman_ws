/*
 * CRTSerial.h
 * 知行电机硬件接口层程序
 * 日期: 2021.5.21
 * 作者: 
 */

#ifndef _CRTSERIAL_H
#define _CRTSERIAL_H

#include "CRTS.h"
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>

class CRTSerial : public CRTS
{
public:
	CRTSerial();
	CRTSerial(u8 End);
	CRTSerial(u8 End, u8 Level);

protected:
	virtual int writeCRTS(unsigned char *nDat, int nLen);//输出nLen字节
	virtual int readCRTS(unsigned char *nDat, int nLen);//输入nLen字节
	virtual int writeCRTS(unsigned char bDat);//输出1字节
	virtual void rFlushCRTS();//
	virtual void wFlushCRTS();//
public:
	unsigned long int IOTimeOut;//输入输出超时
	int Err;
public:
	virtual int getErr(){  return Err;  }
	virtual int setBaudRate(int baudRate);
	virtual bool begin(int baudRate, const char* serialPort);
	virtual void end();
protected:
    int fd;//serial port handle
    struct termios orgopt;//fd ort opt
	struct termios curopt;//fd cur opt
};

#endif