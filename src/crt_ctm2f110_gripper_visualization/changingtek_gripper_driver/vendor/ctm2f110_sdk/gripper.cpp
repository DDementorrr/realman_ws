#include "gripper.h"
#include <iostream>

GripperSDK::GripperSDK()
{

}

GripperSDK::~GripperSDK()
{
   closeSerial();
}

//open serial
int GripperSDK::openSerial(int baudRate, const char* serialPort)
{
    if(!ctmINF.begin(baudRate, serialPort))
    {
       std::cout<< "Failed to init serial device!"<<std::endl;
    return 0;
    }
    return 1;
}

//close serial
void GripperSDK::closeSerial()
{
    ctmINF.end();
}

//set gripper position
int GripperSDK::setpos(float p1,float p2)
{
    convert_pos[0] = OFFSET - p1 * SCALER;
    convert_pos[1] = OFFSET - p2 * SCALER;
    const int first = ctmINF.RegWritePosEx(1,convert_pos[0],60,100);
    const int second = ctmINF.RegWritePosEx(2,convert_pos[1],60,100);
    const int action = ctmINF.RegWriteAction();
    // CRTS::Ack uses zero for a valid acknowledgement and non-zero for a
    // protocol/transport error.
    return (first == 0 && second == 0 && action == 0) ? 1 : 0;
}

int GripperSDK::readpos(float &p1, float &p2)
{
    if (ctmINF.FeedBack(1) != 0) return 0;
    const int servo1 = ctmINF.ReadPos(-1);
    if (servo1 < 0) return 0;
    if (ctmINF.FeedBack(2) != 0) return 0;
    const int servo2 = ctmINF.ReadPos(-1);
    if (servo2 < 0) return 0;
    p1 = static_cast<float>((OFFSET - servo1) / SCALER);
    p2 = static_cast<float>((OFFSET - servo2) / SCALER);
    return 1;
}
