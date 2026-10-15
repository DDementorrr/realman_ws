/*
 * CTMINF.h
 * 知行CTM系列应用层程序
 * 日期: 2021.5.21
 * 作者: 
 */

#ifndef _CTM_H
#define _CTM_H

//波特率定义
#define	CTM_1M 0
#define	CTM_0_5M 1
#define	CTM_250K 2
#define	CTM_128K 3
#define	CTM_115200 4
#define	CTM_76800	5
#define	CTM_57600	6
#define	CTM_38400	7

//内存表定义
//-------EPROM(只读)--------
#define CTM_MODEL_L 3
#define CTM_MODEL_H 4

//-------EPROM(读写)--------
#define CTM_ID 5
#define CTM_BAUD_RATE 6
#define CTM_MIN_ANGLE_LIMIT_L 9
#define CTM_MIN_ANGLE_LIMIT_H 10
#define CTM_MAX_ANGLE_LIMIT_L 11
#define CTM_MAX_ANGLE_LIMIT_H 12
#define CTM_CW_DEAD 26
#define CTM_CCW_DEAD 27
#define CTM_OFS_L 31
#define CTM_OFS_H 32
#define CTM_MODE 33

//-------SRAM(读写)--------
#define CTM_TORQUE_ENABLE 40
#define CTM_ACC 41
#define CTM_GOAL_POSITION_L 42
#define CTM_GOAL_POSITION_H 43
#define CTM_GOAL_TIME_L 44
#define CTM_GOAL_TIME_H 45
#define CTM_GOAL_SPEED_L 46
#define CTM_GOAL_SPEED_H 47
#define CTM_LOCK 55

//-------SRAM(只读)--------
#define CTM_PRESENT_POSITION_L 56
#define CTM_PRESENT_POSITION_H 57
#define CTM_PRESENT_SPEED_L 58
#define CTM_PRESENT_SPEED_H 59
#define CTM_PRESENT_LOAD_L 60
#define CTM_PRESENT_LOAD_H 61
#define CTM_PRESENT_VOLTAGE 62
#define CTM_PRESENT_TEMPERATURE 63
#define CTM_MOVING 66
#define CTM_PRESENT_CURRENT_L 69
#define CTM_PRESENT_CURRENT_H 70

#include "CRTSerial.h"

class CTMINF : public CRTSerial
{
public:
    CTMINF();
    CTMINF(u8 End);
    CTMINF(u8 End, u8 Level);
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);//普通写单个电机位置指令
	virtual int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);//异步写单个电机位置指令(RegWriteAction生效)
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);//同步写多个电机位置指令
	virtual int WheelMode(u8 ID);//恒速模式
	virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);//恒速模式控制指令
	virtual int EnableTorque(u8 ID, u8 Enable);//扭力控制指令
	virtual int unLockEprom(u8 ID);//eprom解锁
	virtual int LockEprom(u8 ID);//eprom加锁
	virtual int CalibrationOfs(u8 ID);//中位校准
	virtual int FeedBack(int ID);//反馈电机信息
	virtual int ReadPos(int ID);//读位置
	virtual int ReadSpeed(int ID);//读速度
	virtual int ReadLoad(int ID);//读输出至电机的电压百分比(0~1000)
	virtual int ReadVoltage(int ID);//读电压
	virtual int ReadTemper(int ID);//读温度
	virtual int ReadMove(int ID);//读移动状态
	virtual int ReadCurrent(int ID);//读电流
private:
	u8 Mem[CTM_PRESENT_CURRENT_H-CTM_PRESENT_POSITION_L+1];
};

#endif