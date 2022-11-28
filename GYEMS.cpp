/***************************************************************
* Arduino GYEMS RMD-L-70 Servo Motor Library - Version 1.0
* by rin9147 forked from Rasheed Kittinanthapanya 
* a simple RS485 communication between GYEMS servo and Arduino
***************************************************************/

#include "Arduino.h"
#include "GYEMS.h"

// GYMES
//------------------------------------------------------------------//
GYEMS::GYEMS(int ID, int RX, int TX, int EN)
{
  _ID = (byte)ID;                                   // simply convert int to hex
  _EN = EN;
  pinMode(_EN, OUTPUT);                             // set enable pin for RS485 module as pin 2 of Arduino
  Serial1.begin(115200, SERIAL_8N1, RX, TX);
  digitalWrite(_EN,LOW);
}

// Convert int16_t to Byte
//------------------------------------------------------------------//
void GYEMS::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2])
{
  // unsigned int can store 16 bit int 
  StoreByte[0] = (Data & 0xFF00) >> 8;                  //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF);                       //Low byte, most left of HEX
}

// Convert int32_t to Byte
//------------------------------------------------------------------//
void GYEMS::Int32ToByteData(unsigned long Data, unsigned char StoreByte[4])
{
  // unsigned long can store 32 bit int
  StoreByte[0] = (Data & 0xFF000000) >> 24;             //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF0000) >> 16;
  StoreByte[2] = (Data & 0x0000FF00) >> 8;
  StoreByte[3] = (Data & 0x000000FF);                   //Low byte, most left of HEX
}

// Convert int62_t to Byte
//------------------------------------------------------------------//
void GYEMS::Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8])
{
  // unsigned long long can store 64 bit int
  StoreByte[0] = (Data & 0xFF00000000000000) >> 56;       //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF000000000000) >> 48;
  StoreByte[2] = (Data & 0x0000FF0000000000) >> 40;
  StoreByte[3] = (Data & 0x000000FF00000000) >> 32;     
  StoreByte[4] = (Data & 0x00000000FF000000) >> 24;     
  StoreByte[5] = (Data & 0x0000000000FF0000) >> 16;
  StoreByte[6] = (Data & 0x000000000000FF00) >> 8;
  StoreByte[7] = (Data & 0x00000000000000FF);             //Low byte, most left of HEX
}

// read Driver respond(13bit)
//------------------------------------------------------------------//
void GYEMS::ReadReply13bit(byte commandByte, int replyData[4])
{
  delayMicroseconds(800);
  byte FrameCheckSumReply = Header + commandByte + 0x07 + _ID;
  unsigned char EncoderReply[13] = {0};
  int EncoderReplySize = sizeof(EncoderReply);
  while (Serial1.available() > 0)
  {
    Serial1.readBytes(EncoderReply, EncoderReplySize);
    if (FrameCheckSumReply == EncoderReply[0] + EncoderReply[1] + EncoderReply[2] + EncoderReply[3])
    {
      replyData[0] = (int8_t)EncoderReply[5];
      replyData[1] = (int16_t)(EncoderReply[7] << 8) | (EncoderReply[6]);
      replyData[2] = (int16_t)(EncoderReply[9] << 8) | (EncoderReply[8]);
      replyData[3] = (int16_t)(EncoderReply[11] << 8) | (EncoderReply[10]);
    }
  }
  delayMicroseconds(800);
}

// Motor stop shutdown command
//------------------------------------------------------------------//
void GYEMS::MotorOff()
{
  byte OffCommand = 0x80;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + OffCommand + _ID + DataLength;

  digitalWrite(_EN,HIGH);     
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(OffCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  delayMicroseconds(800);
}

// Motor stop command
//------------------------------------------------------------------//
void GYEMS::MotorStop()
{

  byte StopCommand = 0x81;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + StopCommand + _ID + DataLength;

  digitalWrite(_EN,HIGH);     
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(StopCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  delayMicroseconds(800);
}

// Motor operation command
//------------------------------------------------------------------//
void GYEMS::MotorRun()
{
  byte RunCommand = 0x88;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + RunCommand + _ID + DataLength;

  digitalWrite(_EN,HIGH);     
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(RunCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  delayMicroseconds(800);
}

// Write the current opsition to ROM as the motor zero command (This command is not recommended for frequent use)
//------------------------------------------------------------------//
void GYEMS::SetZero()
{
  byte SetZeroCommand = 0x19;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + SetZeroCommand + _ID + DataLength;

  digitalWrite(_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SetZeroCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  delayMicroseconds(800);
}

// Torque closed loop control command
//------------------------------------------------------------------//
void GYEMS::TorqueControl(unsigned int Torque,int replyData[4], bool reply_flag)
{
  // Torque is a raw value, actual torque depends on the motor spec
  byte TorqueCommand = 0xA1;
  byte DataLength = 0x02;
  //byte DataLengthReplay = 0x07;
  byte FrameCheckSum = Header + TorqueCommand + DataLength + _ID;
  //byte FrameCheckSumReply = Header + TorqueCommand + DataLengthReplay + _ID;
  unsigned char TorqueByte[2];
  Int16ToByteData(Torque,TorqueByte);
  byte DataCheckByte = TorqueByte[1] + TorqueByte[0];

  digitalWrite(_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(TorqueCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(TorqueByte[1]);
  Serial1.write(TorqueByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(TorqueCommand, replyData);
  }
}

// Speed closed loop control command
//------------------------------------------------------------------//
void GYEMS::SpeedControl(float DPS, int replyData[3], bool reply_flag)
{
  // DPS is degree per second
  float SpeedLSB = DPS * 100;
  byte SpeedCommand = 0xA2;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + SpeedCommand + DataLength + _ID;
  unsigned char SpeedByte[4];
  Int32ToByteData((int)SpeedLSB,SpeedByte);
  byte DataCheckByte = SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SpeedCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(SpeedCommand, replyData);
  }
}

// Multi position closed loop control command 1
//------------------------------------------------------------------//
void GYEMS::MultiPositionControlMode1(unsigned long long Deg, int replyData[3], bool reply_flag)
{
  unsigned long long DegLSB = Deg * 100;
  byte MultiPositionCommand1 = 0xA3;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + MultiPositionCommand1 + _ID + DataLength;
  unsigned char PositionByte[8];
  Int64ToByteData(DegLSB,PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0];

  digitalWrite(_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(MultiPositionCommand1);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[7]);
  Serial1.write(PositionByte[6]);
  Serial1.write(PositionByte[5]);
  Serial1.write(PositionByte[4]);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(MultiPositionCommand1, replyData);
  }
}

// Multi position closed loop control command 2
//------------------------------------------------------------------//
void GYEMS::MultiPositionControlMode2(unsigned long long Deg, unsigned long DPS, int replyData[3], bool reply_flag)
{
  unsigned long long DegLSB = Deg * 100;
  unsigned long SpeedLSB = DPS * 100;
  byte MultiPositionCommand2 = 0xA4;
  byte DataLength = 0x0C;
  byte FrameCheckSum = Header + MultiPositionCommand2 + _ID + DataLength;
  unsigned char PositionByte[8];
  unsigned char SpeedByte[4];
  Int32ToByteData(SpeedLSB,SpeedByte);
  Int64ToByteData(DegLSB,PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] +  SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(MultiPositionCommand2);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[7]);
  Serial1.write(PositionByte[6]);
  Serial1.write(PositionByte[5]);
  Serial1.write(PositionByte[4]);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(MultiPositionCommand2, replyData);
  }
}

// Single position closed loop control command 1
//------------------------------------------------------------------//
void GYEMS::SinglePositionControlMode1(unsigned long Deg, byte Direction, int replyData[3], bool reply_flag)
{
  unsigned long DegLSB = Deg * 100;
  byte SinglePositionCommand1 = 0xA5;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + SinglePositionCommand1 + _ID + DataLength;
  unsigned char PositionByte[4];
  Int32ToByteData(DegLSB,PositionByte);
  byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1];

  digitalWrite(_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SinglePositionCommand1);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(Direction);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(SinglePositionCommand1, replyData);
  }
}

// Single position closed loop control command 2
//------------------------------------------------------------------//
void GYEMS::SinglePositionControlMode2(unsigned long Deg, unsigned long DPS, byte Direction, int replyData[3], bool reply_flag)
{
  unsigned long DegLSB = Deg * 100;
  unsigned long SpeedLSB = DPS * 100;
  byte SinglePositionCommand2 = 0xA6;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + SinglePositionCommand2 + _ID + DataLength;
  unsigned char PositionByte[4];
  unsigned char SpeedByte[4];
  Int32ToByteData(DegLSB,PositionByte);
  Int32ToByteData(SpeedLSB,SpeedByte);
  byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SinglePositionCommand2);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(Direction);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN,LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(SinglePositionCommand2, replyData);
  }
}

// Multi position closed loop control command 1
//------------------------------------------------------------------//
void GYEMS::IncrementalControlMode1(unsigned long long Deg, unsigned long DPS, int replyData[3], bool reply_flag)
{
  unsigned long long DegLSB = Deg * 100;
  unsigned long SpeedLSB = DPS * 100;
  byte IncrementalCommand1 = 0xA7;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + IncrementalCommand1 + _ID + DataLength;
  unsigned char PositionByte[4];
  unsigned char SpeedByte[4];

  Int32ToByteData(SpeedLSB, SpeedByte);
  Int64ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(IncrementalCommand1);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(IncrementalCommand1, replyData);
  }
}

// Multi position closed loop control command 2
//------------------------------------------------------------------//
void GYEMS::IncrementalControlMode2(unsigned long long Deg, unsigned long DPS, int replyData[3], bool reply_flag)
{
  unsigned long long DegLSB = Deg * 100;
  unsigned long SpeedLSB = DPS * 100;
  byte IncrementalCommand2 = 0xA8;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + IncrementalCommand2 + _ID + DataLength;
  unsigned char PositionByte[4];
  unsigned char SpeedByte[4];

  Int32ToByteData(SpeedLSB, SpeedByte);
  Int64ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(IncrementalCommand2);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(IncrementalCommand2, replyData);
  }
}
