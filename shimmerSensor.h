#pragma once

#include <QObject>
#include <QDateTime>
#include <QTimer>
#include <QByteArray>
#include <QQuaternion>
#include <QBluetoothSocket>
#include <QBluetoothAddress>
#include <qbluetoothglobal.h>
#include <qbluetoothlocaldevice.h>
#include <qbluetoothserviceinfo.h>
#include <QJsonObject>
#include <QtSerialPort/QtSerialPort>


#include "shimmerdatapacket.h"
#include "shimmer3box.h"

enum connectionStatus {
  unconnectedStatus,
  connectingStatus,
  connectedStatus,
  initializingStatus,
  waitingFirmwVersStatus,
  waitingShimmerVersStatus,
  waitingSamplingRateStatus,
  waitingGeneralInquireStatus,
  waitingExgRegs1Status,
  waitingExgRegs2Status,
  waitingCalibrationStatus,
  waitingSettingSensorsStatus,
  waitingSettingAccRangeStatus,
  waitingSettingGyroRangeStatus,
  waitingSettingMagGainStatus,
  waitingSettingAdcRateStatus,
  canStartStreamingStatus,
  startingStreamingStatus,
  restartingStreamingStatus,
  stoppingStreamingStatus,
  streamingStatus
};


QT_FORWARD_DECLARE_CLASS(QBluetoothSocket)
QT_FORWARD_DECLARE_CLASS(GradDes3DOrientation)


class
ShimmerSensor : public QObject
{
private:
  Q_OBJECT

public:
  ShimmerSensor(QObject *parent, const QBluetoothAddress &remoteAddress, QString sComPort, qint32 number);
  ~ShimmerSensor();

  bool    BtSetup();
  bool    ComSetup();
  void    closeConnection();
  qint64  writeCommand(const quint8* data, qint32 nbytes);
  void    startWatchDogTimer(qint32 msec);
  void    stopWatchDogTimer();

  void    SetFirmVersionFullName(QString val);
  QString GetFirmwareVersionFullName();
  void    SetFirmwareVersion(double val);
  double  GetFirmwareVersion();
  qint32  GetShimmerVersion();
  void    SetFirmwareInternal(qint32 val);
  qint32  GetFirmwareInternal();
  double  GetFirmwareIdentifier();
  void    SetFirmwareIdentifier(double val);
  void    SetShimmerVersion(qint32 val);
  void    ResetOrientation();

  void    fillProfileShimmer3(QList<quint8> packet);

  qint32  GetNumChannels();
  void    SetNumChannels(qint32 num);
  qint32  GetChannel(qint32 channelNum);
  qint32  GetNumAdcChannels();
  qint32  GetNum1ByteDigiChannels();
  qint32  GetNum2ByteDigiChannels();
  void    SetAdcSamplingRate(qint32 rate);
  qint32  GetAdcSamplingRate();
  void    UpdateSensorsFromChannelsShimmer3();
  void    setSensors(qint32 val);
  qint32  GetSensors();
  bool    GetIsFilled();
  qint32  GetAccelRange();
  void    setGyroOnTheFlyCalibration(bool enable, qint32 bufferSize, double threshold);

  bool    ToggleMagnetometerUse();

  void    jsonRead(const QJsonObject &json);
  void    jsonWrite(QJsonObject &json) const ;

private:
  void    decodeCommand();
  qint32  calculatetwoscomplement(qint32 signedData, qint32 bitLength);
  void    retrievecalibrationparametersfrompacket(quint8 *bufferCalibrationParameters, quint8 packetType);
  QVector<qint32> formatdatapacketreverse(quint8 *data, QString* dataType, qint32 nElem);
  void    processPacket(ShimmerDataPacket* pDataPacket);
  double  getAverage(QList<double> doubleList);
  double  getStandardDeviation(QList<double> doubleList);
  void    calibrateInertialSensorData(double data[3], double AM[3][3], double SM[3][3], double OV[3], double cal[]);

private slots:
  void    readSocket();
  void    connected();
  void    disconnected();
  void    socketError(QBluetoothSocket::SocketError);
  void    serialRead();
  void    serialiWantToClose();
  void    serialError(QSerialPort::SerialPortError);
  void    onWatchDogTimerTimeout();

signals:
  void    sendDebugMessage(QString sDebugMessage);
  void    shimmerConnected(ShimmerSensor *me);
  void    shimmerFailedToConnect(ShimmerSensor *me);
  void    shimmerDisconnected(ShimmerSensor *me);
  void    readDone(const QString &s);
  void    ackReceived(ShimmerSensor *me);
  void    shimmerStatusReceived(ShimmerSensor *me, quint8 status);
  void    shimmerDirReceived(ShimmerSensor *me, QString sDirname);
  void    firmwareRead(ShimmerSensor *me);
  void    versionRead(ShimmerSensor *me);
  void    samplingRateRead(ShimmerSensor *me);
  void    exgRegs1Read(ShimmerSensor *me);
  void    exgRegs2Read(ShimmerSensor *me);
  void    allCalibrationsRead(ShimmerSensor *me);
  void    generalInquiryRead(ShimmerSensor *me);
  void    comunicationError(ShimmerSensor *me);
  void    retryRead();
  void    watchDogTimerTimeout(ShimmerSensor *me);

private:
  QTextStream       sInformation;
  QString           sDebugMessage;
  QString           sString;
  QDateTime         dateTime;
  QObject*          pParent;
  QBluetoothSocket* socket;
  QSerialPort*      serialPort;
  QString           comPort;
  QByteArray        readBuffer;
  volatile bool     bCompleteCommand;

  qint32 sensors;
  qint32 adcSamplingRate;
  qint32 configSetupByte0;
  qint32 accelRange;
  qint32 gyroRange;
  qint32 accelSamplingRate;
  qint32 mpu9150SamplingRate;
  qint32 magSamplingRate;
  qint32 magGain;
  qint32 pressureResolution;
  qint32 gsrRange;
  qint32 internalExpPower;
  qint32 numChannels;
  qint32 numAdcChannels;
  QList<qint32> channels;
  qint32 bufferSize;
  bool isFilled;
  double samplingRate;
  qint32 num1ByteDigiChannels;
  qint32 num2ByteDigiChannels;

  double firmwareIdentifier;
  double firmwareVersion;
  qint32 firmwareInternal;
  QString firmwareVersionFullName;
  qint32 shimmerVersion;

  double dataAccelRaw[3];
  double dataGyroRaw[3];
  double dataMagRaw[3];
  double dataAccelCal[3];
  double dataGyroCal[3];
  double dataGyroCalRad[3];
  double dataMagCal[3];

  bool enableGyroOnTheFlyCalibration;
  qint32 listSizeGyroOnTheFly;
  double thresholdGyroOnTheFly;
  QList<double> gyroXCalList;
  QList<double> gyroYCalList;
  QList<double> gyroZCalList;
  QList<double> gyroXRawList;
  QList<double> gyroYRawList;
  QList<double> gyroZRawList;

  QQuaternion updatedOrientation;
  bool bUseMagnetometers;

public:
  Shimmer3Box shimmerBox; // The graphical objects
  ShimmerDataPacket* keepPacket;
  QQuaternion initialOrientation;
  QQuaternion currentOrientation;
  qint32 chipID;
  bool isStreaming;
  bool isWaitingOrientation;
  QBluetoothAddress myRemoteAddress;
  connectionStatus currentStatus;
  GradDes3DOrientation* mOrientation;
  double acceleration;
  qint32 sensorNumber;
  QTimer watchDogTimer;
  bool enableLowPowerMag;
  bool enableLowPowerAccel;
  bool enableLowPowerGyro;
  QVector<QString> signalDataTypeArray;
  QVector<QString> signalNameArray;
  bool enable3DOrientation;
  bool mFirstTimeCalTime;
  double mLastReceivedCalibratedTimeStamp;
  double mPacketReceptionRate;
  //EXG
  quint8 Exg1RegArray[10];
  quint8 Exg2RegArray[10];

  bool DefaultAccelParams;
  bool DefaultDAccelParams;
  bool DefaultGyroParams;
  bool DefaultMagParams;

  double AlignmentMatrixAccel[3][3];
  double SensitivityMatrixAccel[3][3];
  double OffsetVectorAccel[3];
  double AlignmentMatrixGyro[3][3];
  double SensitivityMatrixGyro[3][3];
  double OffsetVectorGyro[3];
  double AlignmentMatrixMag[3][3];
  double SensitivityMatrixMag[3][3];
  double OffsetVectorMag[3];
  double AlignmentMatrixAccel2[3][3];
  double SensitivityMatrixAccel2[3][3];
  double OffsetVectorAccel2[3];

protected:
  QVector<QString>interpretdatapacketformat(qint32 nC, quint8 *signalid);
};

