#ifndef SET_POSE_H
#define SET_POSE_H

#include <CRTServo.h>

#define SCALER 700.0
#define OFFSET 1700.0
using namespace std;
class GripperSDK
{
public:
  GripperSDK();
  ~GripperSDK();
  int openSerial(int baudRate, const char* serialPort); //open serial
  void closeSerial(); //close serial
  // Sends the calibrated position command to both actuator IDs.
  // Returns non-zero only when all protocol writes are acknowledged.
  int setpos(float p1,float p2);
  // Reads both actuator present positions and converts them to joint coordinates.
  int readpos(float &p1, float &p2);
private:
  CTMINF ctmINF;
  float convert_pos[2] = {0.0,0.0}; //temporary coordinate
  float p1, p2;
};
#endif
