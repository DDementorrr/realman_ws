/*
 * CRTS.h
 * 知行电机通信层协议程序
 * 日期: 2021.5.21
 * 作者: 
 */

#ifndef _CRTS_H
#define _CRTS_H

#include "INST.h"

class CRTS{
public:
	CRTS();
	CRTS(u8 End);
	CRTS(u8 End, u8 Level);
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//普通写指令
	int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//异步写指令
	int RegWriteAction(u8 ID = 0xfe);//异步写执行指令
	void snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);//同步写指令
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);//写1个字节
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);//写2个字节
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);//读指令
	int readByte(u8 ID, u8 MemAddr);//读1个字节
	int readWord(u8 ID, u8 MemAddr);//读2个字节
	int Ping(u8 ID);//Ping指令
public:
	u8	Level;//电机返回等级
	u8	End;//处理器大小端结构
	u8	Error;//电机状态
protected:
	virtual int writeCRTS(unsigned char *nDat, int nLen) = 0;
	virtual int readCRTS(unsigned char *nDat, int nLen) = 0;
	virtual int writeCRTS(unsigned char bDat) = 0;
	virtual void rFlushCRTS() = 0;
	virtual void wFlushCRTS() = 0;
protected:
	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	void Host2CRTS(u8 *DataL, u8* DataH, u16 Data);//1个16位数拆分为2个8位数
	u16	CRTS2Host(u8 DataL, u8 DataH);//2个8位数组合为1个16位数
	int	Ack(u8 ID);//返回应答
	int checkHead();//帧头检测
};
#endif
