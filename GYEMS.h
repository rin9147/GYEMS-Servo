/***************************************************************
* Arduino GYEMS RMD-L-70 Servo Motor Library - Version 1.0
* by rin9147 forked from Rasheed Kittinanthapanya 
* a simple RS485 communication between GYEMS servo and M5Stack Core2
***************************************************************/

#ifndef GYEMS_H
#define GYEMS_H

#include "Arduino.h"

#define MAXDPS 30000

class GYEMS
{
public:
    GYEMS(int ID, int RX, int TX, int EN);
    void Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]);
    void Int32ToByteData(unsigned long Data, unsigned char StoreByte[4]);
    void Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8]);
    void ReadReply13bit(byte commandByte, int replyData[3]);
    void MotorOff();
    void MotorStop();
    void MotorRun();
    void SetZero();
    void TorqueControl(unsigned int Torque,int replyData[3], bool reply_flag);
    void SpeedControl(float DPS, int replyData[3], bool reply_flag);
    void MultiPositionControlMode1(unsigned long long Deg, int replyData[3], bool reply_flag);
    void MultiPositionControlMode2(unsigned long long Deg, unsigned long DPS, int replyData[3], bool reply_flag);
    void SinglePositionControlMode1(unsigned long Deg, byte Direction, int replyData[3], bool reply_flag);
    void SinglePositionControlMode2(unsigned long Deg, unsigned long DPS, byte Direction, int replyData[3], bool reply_flag);
    void IncrementalControlMode1(unsigned long long Deg, unsigned long DPS, int replyData[3], bool reply_flag);
    void IncrementalControlMode2(unsigned long long Deg, unsigned long DPS, int replyData[3], bool reply_flag);

private:
    int _currentPosition;
    byte Header = 0x3E;
    byte _ID;
    int _EN;
    
    unsigned char ReadByte = 0;
    float OutputDeg;
    float CurrentDeg;

    float t2;
    float t1 = 0.0;
    float period;
    float theta1 = 0.0;
    float theta2;
    float CurrentDPS;
    float LastDPS;
};

#endif
