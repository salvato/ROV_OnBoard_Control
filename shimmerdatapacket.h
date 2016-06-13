#ifndef SHIMMERDATAPACKET_H
#define SHIMMERDATAPACKET_H


#include <QString>
#include <QList>


/*
 * Copyright (c) 2010, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:

 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Shimmer Research, Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mike Healy
 * @date   January, 2011
 */

/*
 * ChangeLog
 *
 */

QT_FORWARD_DECLARE_CLASS(ShimmerSensor)


class ShimmerDataPacket
{
public:
  ShimmerDataPacket(QList<quint8> packet, int numAdcChans, int num1ByteDigiChans, int num2ByteDigiChans, int shimmerVersion, int numberofChannels, ShimmerSensor* pShimmer);

private:
  int timeStamp;
  QList<int> channels;
  QList<QString> channelName;
  int numAdcChannels;
  int num1ByteDigiChannels;
  int num2ByteDigiChannels;
  int numChannels;
  bool isFilled;

protected:
  QVector<long> ParseData(QVector<quint8>data,QVector<QString> dataType);


public:
  int GetTimeStamp();
  int GetNumChannels();
  int calculatetwoscomplement(int signedData, int bitLength);
  int GetNumAdcChannels();
  int GetNum1ByteDigiChannels();
  int GetNum2ByteDigiChannels();
  int GetChannel(int channelNum);
  void SetChannel(int channelNum, int val);
  bool GetIsFilled();

};

#endif // SHIMMERDATAPACKET_H
