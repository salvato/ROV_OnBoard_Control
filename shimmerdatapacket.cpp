#include <qmath.h>

#include "shimmer.h"
#include "shimmer3.h"
#include "shimmerSensor.h"
#include "shimmerdatapacket.h"

ShimmerDataPacket::ShimmerDataPacket(QList<quint8> packet, int numAdcChans, int num1ByteDigiChans, int num2ByteDigiChans, int shimmerVersion, int numberofChannels, ShimmerSensor* pShimmer)
  : isFilled(false)
{
  Q_UNUSED(numAdcChans)
  Q_UNUSED(num1ByteDigiChans)
  Q_UNUSED(num2ByteDigiChans)
  Q_UNUSED(shimmerVersion)
  Q_UNUSED(numberofChannels)
  timeStamp = (int)packet.at(0);
  timeStamp += ((int)packet.at(1) << 8) & 0xFF00;
  numChannels = pShimmer->GetNumChannels();
  packet.removeFirst(); // remove timestamp
  packet.removeFirst();
  //packet.RemoveRange(0, 2);
  QVector<quint8>newPacketByte = packet.toVector();
  //quint8 newPacketByte[] = packet.ToArray();
  QVector<long> newPacket = ParseData(newPacketByte, pShimmer->signalDataTypeArray);
  foreach (int l, newPacket) {
    channels.append((int)l);
  }
  isFilled = true;
}


int
ShimmerDataPacket::GetTimeStamp() {
  return timeStamp;
}


int
ShimmerDataPacket::GetNumChannels() {
  return numChannels;
}


int
ShimmerDataPacket::calculatetwoscomplement(int signedData, int bitLength) {
  int newData = signedData;
  if (signedData >= (1 << (bitLength - 1))) {
    newData = -((signedData ^ (int)(qPow(2, bitLength) - 1)) + 1);
  }
  return newData;
}


// Converts the raw packet byte values, into the corresponding calibrated and uncalibrated sensor values, the Instruction String determines the output
// @param newPacket a byte array containing the current received packet
// @param Instructions an array string containing the commands to execute. It is currently not fully supported
// @return

QVector<long>
ShimmerDataPacket::ParseData(QVector<quint8> data, QVector<QString> dataType) {
  int iData = 0;
  QVector<long> formattedData;
  formattedData.resize(dataType.count());

  for (int i = 0; i < dataType.count(); i++) {
    if (dataType[i] == "u8") {
      formattedData[i] = (int)data[iData];
      iData = iData + 1;
    } else if (dataType[i] == "i8") {
      formattedData[i] = calculatetwoscomplement((int)((int)0xFF & data[iData]), 8);
      iData = iData + 1;
    } else if (dataType[i] == "u12") {
      formattedData[i] = (int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8));
      iData = iData + 2;
    } else if (dataType[i] == "i12>") {
      formattedData[i] = calculatetwoscomplement((int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8)), 16);
      formattedData[i] = formattedData[i] >> 4; // shift right by 4 bits
      iData = iData + 2;
    } else if (dataType[i] == "u16") {
      formattedData[i] = (int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8));
      iData = iData + 2;
    } else if (dataType[i] == "u16r") {
      formattedData[i] = (int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData + 0] & 0xFF) << 8));
      iData = iData + 2;
    } else if (dataType[i] == "i16") {
      formattedData[i] = calculatetwoscomplement((int)((int)(data[iData] & 0xFF) + ((int)(data[iData + 1] & 0xFF) << 8)), 16);
      //formattedData[i]=ByteBuffer.wrap(arrayb).order(ByteOrder.LITTLE_ENDIAN).getShort();
      iData = iData + 2;
    } else if (dataType[i] == "i16*") {
      formattedData[i] = calculatetwoscomplement((int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData] & 0xFF) << 8)), 16);
      //formattedData[i]=ByteBuffer.wrap(arrayb).order(ByteOrder.LITTLE_ENDIAN).getShort();
      iData = iData + 2;
    } else if (dataType[i] == "i16r") {
      formattedData[i] = calculatetwoscomplement((int)((int)(data[iData + 1] & 0xFF) + ((int)(data[iData] & 0xFF) << 8)), 16);
      //formattedData[i]=ByteBuffer.wrap(arrayb).order(ByteOrder.LITTLE_ENDIAN).getShort();
      iData = iData + 2;
    } else if (dataType[i] == "u24r") {
      long xmsb = ((long)(data[iData + 0] & 0xFF) << 16);
      long msb = ((long)(data[iData + 1] & 0xFF) << 8);
      long lsb = ((long)(data[iData + 2] & 0xFF));
      formattedData[i] = xmsb + msb + lsb;
      iData = iData + 3;
    } else if (dataType[i] == "i24r") {
      long xmsb = ((long)(data[iData + 0] & 0xFF) << 16);
      long msb = ((long)(data[iData + 1] & 0xFF) << 8);
      long lsb = ((long)(data[iData + 2] & 0xFF));
      formattedData[i] = xmsb + msb + lsb;
      formattedData[i] = calculatetwoscomplement((int)formattedData[i], 24);
      iData = iData + 3;
    }
  }
  return formattedData;
}


int
ShimmerDataPacket::GetNumAdcChannels() {
  return numAdcChannels;
}


int
ShimmerDataPacket::GetNum1ByteDigiChannels() {
  return num1ByteDigiChannels;
}


int
ShimmerDataPacket::GetNum2ByteDigiChannels() {
  return num2ByteDigiChannels;
}


int
ShimmerDataPacket::GetChannel(int channelNum) {
  // channelNum is indexed from 0
  if (channelNum >= numChannels)
    return -1;
  else
    return channels[channelNum];
}


void
ShimmerDataPacket::SetChannel(int channelNum, int val) {
  // channelNum is indexed from 0
  if (channelNum < numChannels)
    channels[channelNum] = val;
}


bool
ShimmerDataPacket::GetIsFilled() {
  return isFilled;
}
