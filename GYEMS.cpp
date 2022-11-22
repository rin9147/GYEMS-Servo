/***************************************************************
* Arduino GYEMS RMD-L-70 Servo Motor Library - Version 1.0
* by Rasheed Kittinanthapanya
* a simple RS485 communication between GYEMS servo and Arduino
***************************************************************/

#include "Arduino.h"
#include "GYEMS.h"

// GYMES
//------------------------------------------------------------------//
GYEMS::GYEMS(int ID)
{
  _ID = (byte)ID;                                       // simply convert int to hex
  pinMode(RS485_EN, OUTPUT);                            // set enable pin for RS485 module as pin 2 of Arduino

  Serial1.begin(115200, SERIAL_8N1, RS485_RX, RS485_TX); // M5Stack Core2 A-PORT
  pinMode(RS485_EN,OUTPUT);
  digitalWrite(RS485_EN,LOW);
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
void GYEMS::Int32ToByteData(unsigned long Data, unsigned char StoreByte[4]){
  
  // unsigned long can store 32 bit int 
  StoreByte[0] = (Data & 0xFF000000) >> 24;             //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF0000) >> 16;
  StoreByte[2] = (Data & 0x0000FF00) >> 8;
  StoreByte[3] = (Data & 0x000000FF);                   //Low byte, most left of HEX
}

// Convert int62_t to Byte
//------------------------------------------------------------------//
void GYEMS::Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8]){
  
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

// (NOTE: THIS PART WILL BE EDITED)
//------------------------------------------------------------------//
unsigned int GYEMS::Make12BitData(unsigned char loByte, unsigned char hiByte)
{
  unsigned int word;
  unsigned int Ang12Bit;

  word = (loByte & 0xFF) | ( (hiByte & 0xFF) << 8);       // construct 2 bytes data to a single 16 bits int
  Ang12Bit = map(word,0,16383,0,4095);                    // simply convert 16 bits to 12 bits data

  return Ang12Bit;
}

// (NOTE: THIS PART WILL BE EDITED)
//------------------------------------------------------------------//
unsigned int GYEMS::Make14BitData(unsigned char loByte, unsigned char hiByte)
{
  unsigned int Ang14Bit;

  Ang14Bit = (loByte & 0xFF) | ( (hiByte & 0xFF) << 8);       // construct 2 bytes data to a single 16 bits int

  return Ang14Bit;
}

// Read driver reply of Read encoder command (NOTE: THIS PART WILL BE EDITED)
//------------------------------------------------------------------//
float GYEMS::ReadReply()
{
  int i = 0;
  bool ReadOK = false;
  unsigned char EncoderReply[8] = {0,0,0,0,0,0,0,0};
  unsigned int EncoderData = 0;
  while ((Serial1.available() > 0))                         // wait for incoming data on Rx
  {
    ReadByte = Serial1.read();                            // read incoming byte
    EncoderReply[i] = ReadByte;                           // store it in array
    i++;
    
    ReadOK = true;
  }
  if (ReadOK == true)
  {
    EncoderData = Make14BitData(EncoderReply[5],EncoderReply[6]); // byte5 and byte 6 are lo-byte and hi-hyte of encoder data

    delayMicroseconds(2000);                      // THIS DELAY IS IMPORTANT to make reading two IDs not mix up
                                                  // similar with the one in GetCurrentDEG()
                                                  // above Serial.print() takes around 2.0~2.5ms and it solves the issue!
                                                  // so instead of print something out, i just mimic it as delay
    CurrentDeg = map((float)EncoderData,0.0,16383.0,0.0,360.0);       // simply convert 16bits data to degree
  }
  return CurrentDeg;
}

// Read encoder current degrees command (NOTE: THIS PART WILL BE EDITED)
//------------------------------------------------------------------//
float GYEMS::GetCurrentDEG()
{
  byte EncoderCommand = 0x90;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + EncoderCommand + _ID + DataLength;
  
  // send a command to servo to request a current position

  digitalWrite(RS485_EN,HIGH);                            // Pulling the pin 2 HIGH for sending via Tx
  delayMicroseconds(50);                                  // delay a bit before start sending...
  Serial1.write(Header);
  Serial1.write(EncoderCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(RS485_EN,LOW);                            // Pulling the pin 2 LOW for receving via Rx
  delayMicroseconds(800);                         // THIS DELAY IS IMPORTANT to make reading two IDs not mix up
                                                  // DO NOT CHANGE THIS VALUE!!
                                                  // above Serial.print() takes around 0.6~0.7ms and it solves the issue!
                                                  // so instead of print something out, i just mimic it as delay

  OutputDeg = ReadReply();                              // This function takes around 0.01 sec
  delayMicroseconds(800);                               // This delay will make the next incoming data not crash, when reading more than one servo 

  return OutputDeg;
}

// Read encoder estimate dps (NOTE: THIS PART WILL BE EDITED)
//------------------------------------------------------------------//
float GYEMS::EstimateDPS()
{
  byte EncoderCommand = 0x90;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + EncoderCommand + _ID + DataLength;
  
  // send a command to servo to request a current position
  digitalWrite(RS485_EN,HIGH);                        // Pulling the pin 2 HIGH for sending via Tx
  delayMicroseconds(50);                              // delay a bit before start sending...
  Serial1.write(Header);
  Serial1.write(EncoderCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(RS485_EN,LOW);                         // Pulling the pin 2 LOW for receving via Rx

  theta2 = ReadReply();                               // current position
  t2 = micros();                                      // current time
  period = (t2 - t1) * 0.000001;                      // difference of time 
  CurrentDPS = (theta2 - theta1) / period;            // velocity = difference of distance/difference of time

  if (abs(CurrentDPS) > MAXDPS)
    CurrentDPS = LastDPS;                             // if there is a spike of velocity, reset it to previous value

  t1 = t2;                                            // update previous time
  theta1 = theta2;                                    // update previous angle
  LastDPS = CurrentDPS;                               // udpate previous velocity

  return CurrentDPS;
}

// Read encoder average speed (NOTE: THIS PART WILL BE EDITED)
//------------------------------------------------------------------//
float GYEMS::GetAverageSpeed()
{
  float Speed;
  float AccumSpeed;
  int sampler = 250;                                // a total sampling data for doing average
  float aveDPS;
  int i = 0;

  for (i=0; i < sampler;i++)
  { 
    Speed = EstimateDPS();                          // estimate current speed in degree per second

    if (i == 0)
      AccumSpeed = Speed;
    else
      AccumSpeed = AccumSpeed + Speed;              // accumulating value
    delayMicroseconds(1000);                        // a small delay for stabilize the loop ***Without this, aveDPS will show 0.0, still don't understand why??***
  }
    
  aveDPS = AccumSpeed / sampler;                      // average speed
  return aveDPS;
}

// Motor stop shutdown command
//------------------------------------------------------------------//
void GYEMS::MotorOff()
{

  byte OffCommand = 0x80;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + OffCommand + _ID + DataLength;

  digitalWrite(RS485_EN,HIGH);                            // Pulling the pin 2 HIGH for sending via Tx
  delayMicroseconds(50);                                  // delay a bit before start sending...
  Serial1.write(Header);
  Serial1.write(OffCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(RS485_EN,LOW);                            // Pulling the pin 2 LOW for receving via Rx

  delayMicroseconds(800);
}

// Motor stop command
//------------------------------------------------------------------//
void GYEMS::MotorStop()
{

  byte StopCommand = 0x81;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + StopCommand + _ID + DataLength;

  digitalWrite(RS485_EN,HIGH);                            // Pulling the pin 2 HIGH for sending via Tx
  delayMicroseconds(50);                                  // delay a bit before start sending...
  Serial1.write(Header);
  Serial1.write(StopCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(RS485_EN,LOW);                            // Pulling the pin 2 LOW for receving via Rx

  delayMicroseconds(800);
}

// Motor operation command
//------------------------------------------------------------------//
void GYEMS::MotorRun()
{
  byte RunCommand = 0x88;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + RunCommand + _ID + DataLength;

  digitalWrite(RS485_EN,HIGH);                            // Pulling the pin 2 HIGH for sending via Tx
  delayMicroseconds(50);                                  // delay a bit before start sending...
  Serial1.write(Header);
  Serial1.write(RunCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(RS485_EN,LOW);                            // Pulling the pin 2 LOW for receving via Rx

  delayMicroseconds(800);
}

// Write the current opsition to ROM as the motor zero command (This command is not recommended for frequent use)
//------------------------------------------------------------------//
void GYEMS::SetZero()
{
  byte SetZeroCommand = 0x19;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + SetZeroCommand + _ID + DataLength;

  digitalWrite(RS485_EN,HIGH);                            // Pulling the pin 2 HIGH for sending via Tx
  delayMicroseconds(50);                                  // delay a bit before start sending...
  Serial1.write(Header);
  Serial1.write(SetZeroCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(RS485_EN,LOW);                            // Pulling the pin 2 LOW for receving via Rx

  delayMicroseconds(800);
}

// Torque closed loop control command
//------------------------------------------------------------------//
void GYEMS::TorqueControl(unsigned int Torque,int replyData[3], bool reply_flag)
{
  // Torque is a raw value, actual torque depends on the motor spec
  byte TorqueCommand = 0xA1;
  byte DataLength = 0x02;
  byte DataLengthReplay = 0x07;
  byte FrameCheckSum = Header + TorqueCommand + DataLength + _ID;
  byte FrameCheckSumReply = Header + TorqueCommand + DataLengthReplay + _ID;
  unsigned char TorqueByte[2];
  Int16ToByteData(Torque,TorqueByte);
  byte DataCheckByte = TorqueByte[1] + TorqueByte[0];

  digitalWrite(RS485_EN,HIGH);
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
  digitalWrite(RS485_EN,LOW);

  if (reply_flag == true)
  {
    delayMicroseconds(800);
    unsigned char EncoderReply[13] = {0};
    int EncoderReplySize = sizeof(EncoderReply);
    int64_t headCheck = 0;
    while (Serial1.available() > 0)
    {
      Serial1.readBytes(EncoderReply, EncoderReplySize);
      headCheck = (byte) EncoderReply[4];
      if (FrameCheckSumReply == EncoderReply[0] + EncoderReply[1] + EncoderReply[2] + EncoderReply[3])
      {
        replyData[0] = (int8_t)EncoderReply[5];
        replyData[1] = (int16_t)(EncoderReply[7] << 8) | EncoderReply[6];
        replyData[2] = (int16_t)(EncoderReply[9] << 8) | EncoderReply[8];
        replyData[3] = (int16_t)(EncoderReply[11] << 8) | EncoderReply[10];
      }
    }
    delayMicroseconds(800);
  }
}

// Speed closed loop control command
//------------------------------------------------------------------//
void GYEMS::SpeedControl(float DPS)
{
  // DPS is degree per second
  float SpeedLSB = DPS*100;
  byte SpeedCommand = 0xA2;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + SpeedCommand + DataLength + _ID;
  unsigned char SpeedByte[4];
  Int32ToByteData((int)SpeedLSB,SpeedByte);
  byte DataCheckByte = SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(RS485_EN,HIGH);
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
  digitalWrite(RS485_EN,LOW);
  //ReadReply();
}

// Multi position closed loop control command 1
//------------------------------------------------------------------//
void GYEMS::PositionControlMode1(unsigned long long Deg)
{ 
  unsigned long long DegLSB = Deg*100;
  byte PositionCommand1 = 0xA3;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + PositionCommand1 + _ID + DataLength ;
  unsigned char PositionByte[8];
  Int64ToByteData(DegLSB,PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0];

  digitalWrite(RS485_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(PositionCommand1);
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
  digitalWrite(RS485_EN,LOW);
  //ReadReply();
}

// Multi position closed loop control command 2
//------------------------------------------------------------------//
void GYEMS::PositionControlMode2(unsigned long long Deg, unsigned long DPS)
{
  unsigned long long DegLSB = Deg*100;
  unsigned long SpeedLSB = DPS*100;
  byte PositionCommand2 = 0xA4;
  byte DataLength = 0x0C;
  byte FrameCheckSum = Header + PositionCommand2 + _ID + DataLength ;
  unsigned char PositionByte[8];
  unsigned char SpeedByte[4];
  Int32ToByteData(SpeedLSB,SpeedByte);
  Int64ToByteData(DegLSB,PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] +  SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(RS485_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(PositionCommand2);
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
  digitalWrite(RS485_EN,LOW);
  //ReadReply(); 
}

// Multi position closed loop control command 3
//------------------------------------------------------------------//
void GYEMS::PositionControlMode3(unsigned long Deg, byte Direction)
{
  unsigned long DegLSB = Deg*100;
  byte PositionCommand3 = 0xA5;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + PositionCommand3 + _ID + DataLength ;
  unsigned char PositionByte[4];
  Int32ToByteData(DegLSB,PositionByte);
  byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1];

  digitalWrite(RS485_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(PositionCommand3);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(Direction);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(RS485_EN,LOW);
  //ReadReply();
}

// Incremental closed-loop control command 2
//------------------------------------------------------------------//
void GYEMS::PositionControlMode4(unsigned long Deg, unsigned long DPS, byte Direction)
{
  unsigned long DegLSB = Deg*100;
  unsigned long SpeedLSB = DPS*100;
  byte PositionCommand4 = 0xA6;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + PositionCommand4 + _ID + DataLength ;
  unsigned char PositionByte[4];
  unsigned char SpeedByte[4];
  Int32ToByteData(DegLSB,PositionByte);
  Int32ToByteData(SpeedLSB,SpeedByte);
  byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(RS485_EN,HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(PositionCommand4);
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
  digitalWrite(RS485_EN,LOW);
  //ReadReply();
}

// Multi position closed loop control command 1
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

  digitalWrite(RS485_EN, HIGH);
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

  digitalWrite(RS485_EN, LOW);
  if (reply_flag == true)
  {
    int i = 0;
    bool ReadOK = false;
    unsigned char EncoderReply[13] = {0};

    while (Serial1.available() > 0)
    {
      ReadByte = Serial1.read();
      EncoderReply[i] = ReadByte;
      i++;

      ReadOK = true;
    }
    if (ReadOK == true)
    {
      replyData[0] = EncoderReply[5];
      replyData[1] = ((EncoderReply[7] << 8 & 0xFF) | (EncoderReply[6] & 0xFF)) * 33 / 2048;
      replyData[2] = ((EncoderReply[9] << 8 & 0xFF) | (EncoderReply[8] & 0xFF));
      replyData[3] = ((EncoderReply[11] << 8 & 0xFF) | (EncoderReply[10] & 0xFF)) / 16383 * 360;
    }
  }
}
