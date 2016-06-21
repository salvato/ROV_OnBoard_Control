#include <QBluetoothServiceInfo>
#include <QBluetoothDeviceInfo>
#include <QBluetoothSocket>
#include <QtMath>
#include <QJsonArray>
#include <unistd.h>

#include "graddes3dorientation.h"
#include "matrixUtil.h"
#include "shimmer.h"
#include "shimmer3.h"
#include "shimmerSensor.h"


#define I_WANT_SEE_MAGNETOMETER   0


ShimmerSensor::ShimmerSensor(QObject* parent, const QBluetoothAddress &remoteAddress, QString sComPort, qint32 number)
  : QObject(parent)
  , pParent(parent)
  , socket(NULL)
  , serialPort(NULL)
  , enableGyroOnTheFlyCalibration(false)
  , listSizeGyroOnTheFly(100)
  , thresholdGyroOnTheFly(1.2)
  , keepPacket(NULL)
  , mOrientation(NULL)
  , sensorNumber(number)
{
  sInformation.setString(&sDebugMessage);
  myRemoteAddress = remoteAddress;
  currentStatus = unconnectedStatus;
  comPort = sComPort;

  isFilled             = false;
  isStreaming          = false;
  isWaitingOrientation = false;
  bUseMagnetometers    = true;

  SetFirmwareIdentifier(1);
  SetFirmwareVersion(0.1);
  SetFirmwareInternal(0);
  SetFirmVersionFullName("BoilerPlate 0.1.0");
  SetShimmerVersion(Shimmer::SHIMMER1);

  initialOrientation = QQuaternion();
  currentOrientation = initialOrientation;
}


ShimmerSensor::~ShimmerSensor() {
  watchDogTimer.stop();
  if(socket) {
    if(socket->state() == QBluetoothSocket::ConnectedState)
      socket->disconnectFromService();
    delete socket;
  }
  if(serialPort) {
    if(serialPort->isOpen())
      serialPort->close();
    delete serialPort;
  }
  if(mOrientation) delete mOrientation;
}


void
ShimmerSensor::ResetOrientation() {
  initialOrientation = QQuaternion();
}


bool
ShimmerSensor::ToggleMagnetometerUse() {
  bUseMagnetometers = !bUseMagnetometers;
  return bUseMagnetometers;
}


bool
ShimmerSensor::GetIsFilled() {
  return isFilled;
}


void
ShimmerSensor::SetFirmwareVersion(double val) {
  firmwareVersion = val;
}


double
ShimmerSensor::GetFirmwareVersion() {
  return firmwareVersion;
}


void
ShimmerSensor::SetFirmVersionFullName(QString val) {
  firmwareVersionFullName = val;
}


QString
ShimmerSensor::GetFirmwareVersionFullName() {
  return firmwareVersionFullName;
}


void
ShimmerSensor::SetFirmwareInternal(qint32 val) {
  firmwareInternal = val;
}


qint32
ShimmerSensor::GetFirmwareInternal() {
  return firmwareInternal;
}


void
ShimmerSensor::SetFirmwareIdentifier(double val) {
  firmwareIdentifier = val;
}


double
ShimmerSensor::GetFirmwareIdentifier() {
  return firmwareIdentifier;
}


qint32
ShimmerSensor::GetShimmerVersion() {
  return shimmerVersion;
}


void
ShimmerSensor::SetShimmerVersion(qint32 val) {
  shimmerVersion = val;
}


void
ShimmerSensor::setSensors(qint32 val) {
  sensors = val;
}


qint32
ShimmerSensor::GetSensors() {
  return sensors;
}


qint32
ShimmerSensor::GetNumChannels() {
  return numChannels;
}


void
ShimmerSensor::SetNumChannels(qint32 num) {
  numChannels = num;
}


qint32
ShimmerSensor::GetNum1ByteDigiChannels() {
  return num1ByteDigiChannels;
}


qint32
ShimmerSensor::GetNum2ByteDigiChannels() {
  return num2ByteDigiChannels;
}


qint32
ShimmerSensor::GetChannel(qint32 channelNum) {
  if (channelNum >= numChannels)
    return -1;
  else
    return channels[channelNum];
}


qint32
ShimmerSensor::GetNumAdcChannels() {
  return numAdcChannels;
}


void
ShimmerSensor::SetAdcSamplingRate(qint32 rate) {
  if(Shimmer3::IsSamplingRateInRange(rate) &&
    GetShimmerVersion() == (qint32)Shimmer::SHIMMER3)
  {
    adcSamplingRate = rate;
    samplingRate = 32768.0 / rate;
  }
}


qint32
ShimmerSensor::GetAdcSamplingRate() {
  return adcSamplingRate;
}


qint32
ShimmerSensor::GetAccelRange() {
  return accelRange;
}


void
ShimmerSensor::startWatchDogTimer(qint32 msec) {
  watchDogTimer.setInterval(msec);
  watchDogTimer.setSingleShot(true);
  watchDogTimer.start();
}


void
ShimmerSensor::stopWatchDogTimer() {
  watchDogTimer.stop();
}


void
ShimmerSensor::onWatchDogTimerTimeout() {
  emit watchDogTimerTimeout(this);
  //watchDogTimer.start();
}


bool
ShimmerSensor::BtSetup() {
  if(socket) {
    if(socket->state() == QBluetoothSocket::ConnectedState) {
      socket->disconnectFromService();
      socket->close();
      socket->disconnect();
      delete socket;
      socket = NULL;
    }
    currentStatus = unconnectedStatus;
  }

  socket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol, this);

  connect(socket, SIGNAL(error(QBluetoothSocket::SocketError)), this, SLOT(socketError(QBluetoothSocket::SocketError)));
  connect(socket, SIGNAL(readyRead()),    this, SLOT(readSocket()));
  connect(socket, SIGNAL(connected()),    this, SLOT(connected()));
  connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected()));
  connect(&watchDogTimer, SIGNAL(timeout()), this, SLOT(onWatchDogTimerTimeout()));

  socket->connectToService(myRemoteAddress, 1);
  return true;
}


bool
ShimmerSensor::ComSetup() {
  if(serialPort) {
    if(serialPort->isOpen()) {
      serialPort->clear();
      serialPort->clearError();
      readBuffer.clear();
      serialPort->close();
      delete serialPort;
      serialPort = NULL;
    }
    currentStatus = unconnectedStatus;
  }
  if(!serialPort) {
    serialPort = new QSerialPort(this);
    serialPort->setPortName(comPort);
  }

  connect(serialPort, SIGNAL(readyRead()),    this, SLOT(serialRead()));
  connect(serialPort, SIGNAL(aboutToClose()), this, SLOT(serialiWantToClose()));
  connect(&watchDogTimer, SIGNAL(timeout()), this, SLOT(onWatchDogTimerTimeout()));

  for(int i=0; i<9; i++) {
    serialPort->clearError();
    if(serialPort->open(QSerialPort::ReadWrite)) break;
    QThread::msleep(500);
  }
  if(!serialPort->isOpen()) {
    emit shimmerFailedToConnect(this);
    return false;
  } else {
    connect(serialPort, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serialError(QSerialPort::SerialPortError)));
    serialPort->setBaudRate(QSerialPort::Baud115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);
    emit shimmerConnected(this);
  }
  return true;
}


void
ShimmerSensor::serialError(QSerialPort::SerialPortError error) {
  if(error==QSerialPort::NoError) return;
  if(serialPort) {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << " " << comPort << " "
                 << serialPort->errorString();
    if(pParent) {
      emit sendDebugMessage(sDebugMessage);
    } else {
       qDebug() << sDebugMessage;
    }
  }
  if(currentStatus == streamingStatus) {
    if(serialPort) serialPort->close();
    emit shimmerDisconnected(this);
  } else {
    emit shimmerFailedToConnect(this);
  }
  currentStatus = unconnectedStatus;
}


void
ShimmerSensor::serialiWantToClose() {
  currentStatus = unconnectedStatus;
  emit shimmerDisconnected(this);
}



void
ShimmerSensor::serialRead() {
  if(!serialPort->isOpen()) {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << myRemoteAddress.toString()
                 << "socket->errorString() ";
    if(pParent) {
      emit sendDebugMessage(sDebugMessage);
    } else {
       qDebug() << sDebugMessage;
    }
    emit shimmerDisconnected(this);
    return;
  }
  while(serialPort->bytesAvailable()) {
    readBuffer.append(serialPort->readAll());
  }
  if(watchDogTimer.isActive()) {
    watchDogTimer.start();
  }
  decodeCommand();
}




void
ShimmerSensor::socketError(QBluetoothSocket::SocketError error) {
  Q_UNUSED(error)
  if(socket) {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << myRemoteAddress.toString()
                 << socket->errorString() ;
    if(pParent) {
      emit sendDebugMessage(sDebugMessage);
    } else {
       qDebug() << sDebugMessage;
    }
  }
  if(currentStatus == streamingStatus) {
    if(socket) socket->disconnect();
    emit shimmerDisconnected(this);
  } else {
    emit shimmerFailedToConnect(this);
  }
  currentStatus = unconnectedStatus;
}


void
ShimmerSensor::readSocket() {
  if(!socket) return;
  while(socket->bytesAvailable()) {
    readBuffer.append(socket->readAll());
  }
  if(watchDogTimer.isActive()) {
    watchDogTimer.start();
  }
  decodeCommand();
}


void
ShimmerSensor::connected() {
  emit shimmerConnected(this);
}


void
ShimmerSensor::closeConnection() {
  if(!socket) return;
  if(socket->isOpen())
    socket->close();
}


void
ShimmerSensor::disconnected() {
  delete socket;
  socket = NULL;
  currentStatus = unconnectedStatus;
  emit shimmerDisconnected(this);
}


void
ShimmerSensor::setGyroOnTheFlyCalibration(bool enable, qint32 bufferSize, double threshold) {
  if(enable) {
    enableGyroOnTheFlyCalibration = true;
    listSizeGyroOnTheFly  = bufferSize;
    thresholdGyroOnTheFly = threshold;
  } else {
    enableGyroOnTheFlyCalibration = false;
  }
}


qint32
ShimmerSensor::calculatetwoscomplement(qint32 signedData, qint32 bitLength) {
  qint32 newData = signedData;
  if (signedData >= (1 << (bitLength - 1))) {
    newData = -((signedData ^ (qint32)(qPow(2, bitLength) - 1)) + 1);
  }
  return newData;
}


QVector<qint32>
ShimmerSensor::formatdatapacketreverse(quint8* data, QString* dataType, qint32 nElem) {
  qint32 iData = 0;
  QVector<qint32> formattedData;
  formattedData.resize(nElem);

  for (qint32 i=0; i<nElem; i++) {
    if (dataType[i] == "u8") {
      formattedData[i] = (qint32)data[iData];
      iData = iData + 1;
    } else if (dataType[i] == "i8") {
      formattedData[i] = calculatetwoscomplement((qint32)((qint32)0xFF & data[iData]), 8);
      iData = iData + 1;
    } else if (dataType[i] == "u12") {
      formattedData[i] = (qint32)((qint32)(data[iData + 1] & 0xFF) + ((qint32)(data[iData] & 0xFF) << 8));
      iData = iData + 2;
    } else if (dataType[i] == "u16") {
      formattedData[i] = (qint32)((qint32)(data[iData + 1] & 0xFF) + ((qint32)(data[iData] & 0xFF) << 8));
      iData = iData + 2;
    } else if (dataType[i] == "i16") {
      formattedData[i] = calculatetwoscomplement((qint32)((qint32)(data[iData + 1] & 0xFF) + ((qint32)(data[iData] & 0xFF) << 8)), 16);
      iData = iData + 2;
    }
  }
  return formattedData;
}


void
ShimmerSensor::retrievecalibrationparametersfrompacket(quint8 *bufferCalibrationParameters, quint8 packetType) {
  try {
    QString dataType[15] = { "i16", "i16", "i16", "i16", "i16", "i16", "i8", "i8", "i8", "i8", "i8", "i8", "i8", "i8", "i8" };
    // using the datatype the calibration parameters are converted
    QVector<qint32> formattedPacket = formatdatapacketreverse(bufferCalibrationParameters, dataType, 15);
    double OffsetVector[3] = { (double)formattedPacket[0],
                               (double)formattedPacket[1],
                               (double)formattedPacket[2]
                             };
    double SensitivityMatrix[3][3] =  { { double(formattedPacket[3]), 0.0, 0.0 },
                                        { 0.0, double(formattedPacket[4]), 0.0 },
                                        { 0.0, 0.0, double(formattedPacket[5]) }
                                      };
    double AM[9];
    for (qint32 i=0; i<9; i++) {
      AM[i] = ((double)formattedPacket[6+i]) / 100.0;
    }

    double AlignmentMatrix[3][3] = { { AM[0], AM[1], AM[2] },
                                     { AM[3], AM[4], AM[5] },
                                     { AM[6], AM[7], AM[8] }
                                   };

    if (packetType == (quint8)Shimmer3::LSM303DLHC_ACCEL_RANGE_RESPONSE) {

      CopyMatrix3x3((double *)AlignmentMatrixAccel, (double *)AlignmentMatrix);
      CopyMatrix3x3((double *)SensitivityMatrixAccel, (double *)SensitivityMatrix);
      CopyVector3((double *)OffsetVectorAccel, (double *)OffsetVector);
      DefaultAccelParams = false;

    } else if (packetType == (quint8)Shimmer3::LSM303DLHC_ACCEL_CALIBRATION_RESPONSE) {

      CopyMatrix3x3((double *)AlignmentMatrixAccel2, (double *)AlignmentMatrix);
      CopyMatrix3x3((double *)SensitivityMatrixAccel2, (double *)SensitivityMatrix);
      CopyVector3((double *)OffsetVectorAccel2, (double *)OffsetVector);
      DefaultDAccelParams = false;

    } else if (packetType == (quint8)Shimmer3::MPU9150_GYRO_CALIBRATION_RESPONSE) {

      CopyMatrix3x3((double *)AlignmentMatrixGyro, (double *)AlignmentMatrix);
      CopyVector3((double *)OffsetVectorGyro, (double *)OffsetVector);
      CopyMatrix3x3((double *)SensitivityMatrixGyro, (double *)SensitivityMatrix);
      SensitivityMatrixGyro[0][0] /= 100.0;
      SensitivityMatrixGyro[1][1] /= 100.0;
      SensitivityMatrixGyro[2][2] /= 100.0;
      DefaultGyroParams = false;

    } else if (packetType == (quint8)Shimmer3::LSM303DLHC_MAG_CALIBRATION_RESPONSE) {

      CopyMatrix3x3((double *)AlignmentMatrixMag, (double *)AlignmentMatrix);
      CopyVector3((double *)OffsetVectorMag, (double *)OffsetVector);
      CopyMatrix3x3((double *)SensitivityMatrixMag, (double *)SensitivityMatrix);
      DefaultMagParams = false;

    }
  } catch(QString sError) {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << myRemoteAddress.toString()
                 << sError
                 << "in retrievecalibrationparametersfrompacket()";
    if(pParent) {
      emit sendDebugMessage(sDebugMessage);
    } else {
       qDebug() << sDebugMessage;
    }
  }
}


qint64
ShimmerSensor::writeCommand(const quint8 *data, qint32 nbytes) {
  qint64 bytesWritten;
  if(!socket) {
    if(!serialPort) return -1;
    bytesWritten = serialPort->write((char*)data, nbytes);
    if(bytesWritten != nbytes) {
      sDebugMessage = QString();
      sInformation << dateTime.currentDateTime().toString()
                   << myRemoteAddress.toString()
                   << " Not all bytes were written";
      if(pParent) {
        emit sendDebugMessage(sDebugMessage);
      } else {
         qDebug() << sDebugMessage;
      }
    }
    return bytesWritten;
  }
  if(socket->state() == QBluetoothSocket::ConnectedState) {
    qint64 bytesWritten = socket->write((char *)data, nbytes);
    if(bytesWritten != nbytes) {
      sDebugMessage = QString();
      sInformation << dateTime.currentDateTime().toString()
                   << myRemoteAddress.toString()
                   << " Not all bytes were written";
      if(pParent) {
        emit sendDebugMessage(sDebugMessage);
      } else {
         qDebug() << sDebugMessage;
      }
    }
    return bytesWritten;
  } else {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << myRemoteAddress.toString()
                 << " Impossible to write to ";
    if(pParent) {
      emit sendDebugMessage(sDebugMessage);
    } else {
       qDebug() << sDebugMessage;
    }
    return -1;
  }
}


void
ShimmerSensor::processPacket(ShimmerDataPacket* pDataPacket) {

  for (qint32 i=0; i<pDataPacket->GetNumChannels(); i++) {
    if (GetChannel(i) == (qint32)Shimmer3::XMag ||
        GetChannel(i) == (qint32)Shimmer3::YMag ||
        GetChannel(i) == (qint32)Shimmer3::ZMag)
    {
      // The magnetometer gives a signed 16 bit integer per channel
      qint16 intVal = pDataPacket->GetChannel(i) & 0xFFFF;
      pDataPacket->SetChannel(i, intVal);
    }
    //run through and get all the data required for orientation calculation
    if (((GetChannel(i) == (qint32)Shimmer3::XWRAccel))) {
      dataAccelRaw[0] = (double)pDataPacket->GetChannel(i);
    } else if (((GetChannel(i) == (qint32)Shimmer3::YWRAccel))) {
      dataAccelRaw[1] = (double)pDataPacket->GetChannel(i);
    } else if (((GetChannel(i) == (qint32)Shimmer3::ZWRAccel))) {
      dataAccelRaw[2] = (double)pDataPacket->GetChannel(i);
    } else if (GetChannel(i) == (qint32)Shimmer3::XGyro) {
      dataGyroRaw[0] = (double)pDataPacket->GetChannel(i);
    } else if (GetChannel(i) == (qint32)Shimmer3::YGyro) {
      dataGyroRaw[1] = (double)pDataPacket->GetChannel(i);
    } else if (GetChannel(i) == (qint32)Shimmer3::ZGyro) {
      dataGyroRaw[2] = (double)pDataPacket->GetChannel(i);
    } else if (GetChannel(i) == (qint32)Shimmer3::XMag) {
      dataMagRaw[0] = (double)pDataPacket->GetChannel(i);
    } else if (GetChannel(i) == (qint32)Shimmer3::YMag) {
      dataMagRaw[1] = (double)pDataPacket->GetChannel(i);
    } else if (GetChannel(i) == (qint32)Shimmer3::ZMag) {
      dataMagRaw[2] = (double)pDataPacket->GetChannel(i);
    }
  }// for (qint32 i=0; i<pDataPacket->GetNumChannels(); i++)

  //calculate quartenion here
  if ((((GetSensors() & (qint32)Shimmer3::SensorAAccel) > 0 &&
        (GetSensors() & (qint32)Shimmer3::SensorDAccel) == 0)) ||
      (((GetSensors() & (qint32)Shimmer3::SensorAAccel) > 0 &&
        GetAccelRange() == 0)))
  {
    calibrateInertialSensorData(dataAccelRaw,
                                AlignmentMatrixAccel,
                                SensitivityMatrixAccel,
                                OffsetVectorAccel,
                                dataAccelCal);
  } else {
    calibrateInertialSensorData(dataAccelRaw,
                                AlignmentMatrixAccel2,
                                SensitivityMatrixAccel2,
                                OffsetVectorAccel2,
                                dataAccelCal);
  }

  if (enableGyroOnTheFlyCalibration) {
    gyroXRawList.append(dataGyroRaw[0]);
    gyroYRawList.append(dataGyroRaw[1]);
    gyroZRawList.append(dataGyroRaw[2]);
    if (gyroXRawList.count() > listSizeGyroOnTheFly) {
      gyroXRawList.removeFirst();
      gyroYRawList.removeFirst();
      gyroZRawList.removeFirst();
    }
  }
  calibrateInertialSensorData(dataGyroRaw,
                              AlignmentMatrixGyro,
                              SensitivityMatrixGyro,
                              OffsetVectorGyro,
                              dataGyroCal);

  if (enableGyroOnTheFlyCalibration && bUseMagnetometers) {
    gyroXCalList.append(dataGyroCal[0]);
    gyroYCalList.append(dataGyroCal[1]);
    gyroZCalList.append(dataGyroCal[2]);
    if (gyroXCalList.count() > listSizeGyroOnTheFly) {
      gyroXCalList.removeFirst();
      gyroYCalList.removeFirst();
      gyroZCalList.removeFirst();
      // Compute new offset only if the averaged rotational speed
      // is less than the given threshold (no rotations...)
      if (getStandardDeviation(gyroXCalList) < thresholdGyroOnTheFly &&
          getStandardDeviation(gyroYCalList) < thresholdGyroOnTheFly &&
          getStandardDeviation(gyroZCalList) < thresholdGyroOnTheFly)
      {
        OffsetVectorGyro[0] = getAverage(gyroXRawList);
        OffsetVectorGyro[1] = getAverage(gyroYRawList);
        OffsetVectorGyro[2] = getAverage(gyroZRawList);
      }
    }
  }// if (enableGyroOnTheFlyCalibration)

  calibrateInertialSensorData(dataMagRaw,
                              AlignmentMatrixMag,
                              SensitivityMatrixMag,
                              OffsetVectorMag,
                              dataMagCal);
#if I_WANT_SEE_MAGNETOMETER
  sString.sprintf("Mx=%+012.2f\tMy=%+012.2f\tMz=%+012.2f",
                  dataMagRaw[0], dataMagRaw[1], dataMagRaw[2]);
  qDebug() << myRemoteAddress.toString() << sString;
#endif

  for(int i=0; i<3; i++)
    dataGyroCalRad[i] = dataGyroCal[i] * M_PI / 180.0;

  // >>>>>>>>>>>>>>>>>>>>>>>>
  // Gradient descent update!
  // >>>>>>>>>>>>>>>>>>>>>>>>

  updatedOrientation = mOrientation->MadgwickAHRSupdate(dataAccelCal[0],
                                                        dataAccelCal[1],
                                                        dataAccelCal[2],
                                                        dataGyroCalRad[0],
                                                        dataGyroCalRad[1],
                                                        dataGyroCalRad[2],
                                                        dataMagCal[0],
                                                        dataMagCal[1],
                                                        dataMagCal[2]);

  if(isWaitingOrientation) {// set initial orientation
    initialOrientation = updatedOrientation;
    isWaitingOrientation = false;
  }
  currentOrientation = initialOrientation.conjugate() * updatedOrientation;

  // Calculate acceleration component different from gravity
  acceleration = 0.0;
  for(qint32 i=0; i<3; i++) {
    acceleration += dataAccelCal[i]*dataAccelCal[i];
  }
  acceleration = qSqrt(acceleration) - 9.81;

  // Update sensor orientation for the graphical presentation
  if(currentOrientation.scalar()>1.0) currentOrientation.normalize();
  // assuming quaternion normalized then scalar() is less than 1, so term always positive.
  double s = qSqrt(1.0-currentOrientation.scalar()*currentOrientation.scalar());
  if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
    shimmerBox.setAxisAngle(360.0*acos(currentOrientation.scalar())/M_PI,
                            currentOrientation.x(),
                            currentOrientation.y(),
                            currentOrientation.z());
  } else {
    shimmerBox.setAxisAngle(360.0*acos(currentOrientation.scalar())/M_PI,
                            currentOrientation.x()/s,
                            currentOrientation.y()/s,
                            currentOrientation.z()/s);
  }

}


void
ShimmerSensor::calibrateInertialSensorData(double data[3], double AM[3][3], double SM[3][3], double OV[3], double cal[3]) {
  //  Based on the theory outlined by Ferraris F, Grimaldi U, and Parvis M. in:
  //   "Procedure for effortless in-field calibration of three-axis rate gyros and accelerometers"
  //   Sens. Mater. 1995; 7: 311-30.
  //
  //   C = [R^(-1)] .[K^(-1)] .([U]-[B])
  //
  // where.....
  //
  // [C]      -> [3 x n] Calibrated Data Matrix
  // [U]      -> [3 x n] Uncalibrated Data Matrix
  // [B]      -> [3 x n] Replicated Sensor Offset Vector fip
  // [R^(-1)] -> [3 x 3] Inverse Alignment Matrix
  // [K^(-1)] -> [3 x 3] Inverse Sensitivity Matrix
  //
  // n = Number of Samples

  double data2d[3];
  double data2d_OV[3];
  for(qint32 i=0; i<3; i++) data2d[i]= data[i];
  double SM_1[3][3];
  double AM_1[3][3];
  double M1[3][3];
  matrixinverse3x3(AM, AM_1);
  matrixinverse3x3(SM, SM_1);
  matrixmultiplication((double *)AM_1, (double *)SM_1, (double *)M1, 3, 3, 3, 3);
  matrixminus((double*)data2d, (double*)OV, (double*)data2d_OV, 3, 1, 3, 1);
  matrixmultiplication((double *)M1, (double*)data2d_OV, (double*)cal, 3 ,3, 3, 1);
}


double
ShimmerSensor::getAverage(QList<double> doubleList) {
  try {
    if(doubleList.count() == 0) throw QString("Empty List: can't calculate average");
    double average = 0.0;
    for (qint32 i=0; i<doubleList.count(); i++) {
      average += doubleList.at(i);
    }
    return average / doubleList.count();
  } catch(QString sError) {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << myRemoteAddress.toString()
                 << sError;
    if(pParent) {
      emit sendDebugMessage(sDebugMessage);
    } else {
       qDebug() << sDebugMessage;
    }
    exit(-1);
    return 0.0;
  }
}


double
ShimmerSensor::getStandardDeviation(QList<double> doubleList) {
  try {
    if(doubleList.count() == 0) throw QString("Empty List: can't calculate standard deviation");
    double average = getAverage(doubleList);
    double sumOfDerivation = 0;
    for (qint32 i=0; i<doubleList.count(); i++) {
      sumOfDerivation += (doubleList.at(i)) * (doubleList.at(i));
    }
    double sumOfDerivationAverage = sumOfDerivation / doubleList.count();
    return qSqrt(sumOfDerivationAverage - (average * average));
  } catch (QString sError) {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << myRemoteAddress.toString()
                 << sError;
    if(pParent) {
      emit sendDebugMessage(sDebugMessage);
    } else {
       qDebug() << sDebugMessage;
    }
    return 0.0;
  }
}


void
ShimmerSensor::fillProfileShimmer3(QList<quint8> packet) {
  //check if this packet is sane, and not just random
  if ((packet.count() >= 8) // minimum size
      && (packet.count() < (qint32)Shimmer3::ResponsePacketSize))      // max number of channels currently allowable
  {
    adcSamplingRate     = (qint32)packet[0] + ((((qint32)packet[1]) << 8) & 0xFF00);
    configSetupByte0    = (qint32)packet[2] + (((qint32)packet[3]) << 8) + (((qint32)packet[4]) << 16) + (((qint32)packet[5]) << 24);
    accelRange          = (configSetupByte0 >>  2) & 0x03;
    gyroRange           = (configSetupByte0 >> 16) & 0x03;
    magGain             = (configSetupByte0 >> 21) & 0x07;
    accelSamplingRate   = (configSetupByte0 >>  4) & 0xF;
    mpu9150SamplingRate = (configSetupByte0 >>  8) & 0xFF;
    magSamplingRate     = (configSetupByte0 >> 18) & 0x07;
    pressureResolution  = (configSetupByte0 >> 28) & 0x03;
    gsrRange            = (configSetupByte0 >> 25) & 0x07;
    internalExpPower    = (configSetupByte0 >> 24) & 0x01;

    if((magSamplingRate == 4 && adcSamplingRate < 3200) ) {
      enableLowPowerMag = true;
    }

    if((accelSamplingRate == 2 && adcSamplingRate < 3200)) {
      enableLowPowerAccel = true;
    }

    if((mpu9150SamplingRate == 0xFF && adcSamplingRate < 3200)) {
      enableLowPowerGyro = true;
    }

    numChannels = (qint32)packet[6];
    bufferSize  = (qint32)packet[7];
    channels.clear();
    if(packet.count() < 8+numChannels) {
      sDebugMessage = QString();
      sInformation << dateTime.currentDateTime().toString()
                   << myRemoteAddress.toString()
                   << " ShimmerProfile::fillProfileShimmer3() incomplete data Packet";
      if(pParent) {
        emit sendDebugMessage(sDebugMessage);
      } else {
         qDebug() << sDebugMessage;
      }
      return;
    }
    quint8* channelContents= new quint8[numChannels];

    for (qint32 i=0; i<numChannels; i++) {
      channels.append((qint32)packet[8 + i]);
      channelContents[i] = packet[8 + i];
    }
    isFilled = true;
    UpdateSensorsFromChannelsShimmer3();
    signalDataTypeArray = interpretdatapacketformat(numChannels, channelContents);
    delete[] channelContents;
  }
}


QVector<QString>
ShimmerSensor::interpretdatapacketformat(qint32 nC, quint8* signalid) {
  signalNameArray.resize(nC);
  signalDataTypeArray.resize(nC);
  qint32 packetSize = 0;
  qint32 enabledSensors = 0x00;

  for (qint32 i=0; i<nC; i++) {
    if ((quint8)signalid[i] == (quint8)0x00) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Low Noise Accelerometer X";
        signalDataTypeArray[i] = "i16";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorAAccel);
      }
    } else if ((quint8)signalid[i] == (quint8)0x01) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Low Noise Accelerometer Y";
        signalDataTypeArray[i] = "i16";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorAAccel);
      }
    } else if ((quint8)signalid[i] == (quint8)0x02) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Low Noise Accelerometer Z";
        signalDataTypeArray[i] = "i16";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorAAccel);
      }
    } else if ((quint8)signalid[i] == (quint8)0x03) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "VSenseBatt"; //should be the battery but this will do for now
        signalDataTypeArray[i] = "i16";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorVBatt);
      }
    } else if ((quint8)signalid[i] == (quint8)0x04) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i16";
        packetSize = packetSize + 2;
        signalNameArray[i] = "Wide Range Accelerometer X";
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorDAccel);
      }
    } else if ((quint8)signalid[i] == (quint8)0x05) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i16";
        packetSize = packetSize + 2;
        signalNameArray[i] = "Wide Range Accelerometer Y";
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorDAccel);
      }
    } else if ((quint8)signalid[i] == (quint8)0x06) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i16";
        packetSize = packetSize + 2;
        signalNameArray[i] = "Wide Range Accelerometer Z";
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorDAccel);
      }
    } else if ((quint8)signalid[i] == (quint8)0x07) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Magnetometer X";
        signalDataTypeArray[i] = "i16*";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorMag);
      }
    } else if ((quint8)signalid[i] == (quint8)0x08) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Magnetometer Y";
        signalDataTypeArray[i] = "i16*";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorMag);
      }
    } else if ((quint8)signalid[i] == (quint8)0x09) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Magnetometer Z";
        signalDataTypeArray[i] = "i16*";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorMag);
      }
    } else if ((quint8)signalid[i] == (quint8)0x0A) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Gyroscope X";
        signalDataTypeArray[i] = "i16*";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorGyro);
      }
    } else if ((quint8)signalid[i] == (quint8)0x0B) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Gyroscope Y";
        signalDataTypeArray[i] = "i16*";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorGyro);
      }
    } else if ((quint8)signalid[i] == (quint8)0x0C) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Gyroscope Z";
        signalDataTypeArray[i] = "i16*";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorGyro);
      }
    } else if ((quint8)signalid[i] == (quint8)0x0D) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "External ADC A7";
        signalDataTypeArray[i] = "u12";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExtA7);
      }
    } else if ((quint8)signalid[i] == (quint8)0x0E) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "External ADC A6";
        signalDataTypeArray[i] = "u12";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExtA6);
      }
    } else if ((quint8)signalid[i] == (quint8)0x0F) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "External ADC A15";
        signalDataTypeArray[i] = "u12";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExtA15);
      }
    } else if ((quint8)signalid[i] == (quint8)0x10) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Internal ADC A1";
        signalDataTypeArray[i] = "u12";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorIntA1);
      }
    } else if ((quint8)signalid[i] == (quint8)0x11) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Internal ADC A12";
        signalDataTypeArray[i] = "u12";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorIntA12);
      }
    } else if ((quint8)signalid[i] == (quint8)0x12) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Internal ADC A13";
        signalDataTypeArray[i] = "u12";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorIntA13);
      }
    } else if ((quint8)signalid[i] == (quint8)0x13) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Internal ADC A14";
        signalDataTypeArray[i] = "u12";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorIntA14);
      }
    } else if ((quint8)signalid[i] == (quint8)0x1A) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Temperature";
        signalDataTypeArray[i] = "u16r";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorPressure);
      }
    } else if ((quint8)signalid[i] == (quint8)0x1B) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "Pressure";
        signalDataTypeArray[i] = "u24r";
        packetSize = packetSize + 3;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorPressure);
      }
    } else if ((quint8)signalid[i] == (quint8)0x1C) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalNameArray[i] = "GSR Raw";
        signalDataTypeArray[i] = "u16";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorGSR);
      }
    } else if ((quint8)signalid[i] == (quint8)0x1D) {
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "u8";
        packetSize = packetSize + 1;
      }
    } else if ((quint8)signalid[i] == (quint8)0x1E) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i24r";
        packetSize = packetSize + 3;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg1);
      }
    } else if ((quint8)signalid[i] == (quint8)0x1F) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i24r";
        packetSize = packetSize + 3;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg1);
      }
    } else if ((quint8)signalid[i] == (quint8)0x20) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "u8";
        packetSize = packetSize + 1;
      }
    } else if ((quint8)signalid[i] == (quint8)0x21) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i24r";
        packetSize = packetSize + 3;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg2);
      }
    } else if ((quint8)signalid[i] == (quint8)0x22) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i24r";
        packetSize = packetSize + 3;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg2);
      }
    } else if ((quint8)signalid[i] == (quint8)0x23) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i16r";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg1_16Bit);
      }
    } else if ((quint8)signalid[i] == (quint8)0x24) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i16r";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg1_16Bit);
      }
    } else if ((quint8)signalid[i] == (quint8)0x25) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i16r";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg2_16Bit);
      }
    } else if ((quint8)signalid[i] == (quint8)0x26) {//EXG
      if (shimmerVersion == (qint32)Shimmer::SHIMMER3) {
        signalDataTypeArray[i] = "i16r";
        packetSize = packetSize + 2;
        enabledSensors = (enabledSensors | (qint32)Shimmer3::SensorExg1_16Bit);
      }
    } else {
      signalNameArray[i] = "";
      signalDataTypeArray[i] = "u12";
      packetSize = packetSize + 2;
    }
  }
  return signalDataTypeArray;
}


void
ShimmerSensor::UpdateSensorsFromChannelsShimmer3() {
  // set the sensors value
  // crude way of getting this value, but allows for more customised firmware
  // to still work with this application
  // e.g. if any axis of the accelerometer is being transmitted, then it will
  // recognise that the accelerometer is being sampled
  sensors = 0;
  numChannels = 0;
  QListIterator<qint32> channel(channels);
  while (channel.hasNext()) {//foreach (qint32 channel in channels)
    qint32 ichannel = channel.next();
    if ((ichannel == (qint32)Shimmer3::XLNAccel) ||
        (ichannel == (qint32)Shimmer3::YLNAccel) ||
        (ichannel == (qint32)Shimmer3::ZLNAccel))
    {
      sensors |= (qint32)Shimmer3::SensorAAccel;
    }
    else if (ichannel == (qint32)Shimmer3::VBatt)
    {
      sensors |= (qint32)Shimmer3::SensorVBatt;
    }
    else if ((ichannel == (qint32)Shimmer3::XGyro) ||
             (ichannel == (qint32)Shimmer3::YGyro) ||
             (ichannel == (qint32)Shimmer3::ZGyro))
    {
      sensors |= (qint32)Shimmer3::SensorGyro;
    }
    else if ((ichannel == (qint32)Shimmer3::XWRAccel) ||
             (ichannel == (qint32)Shimmer3::YWRAccel) ||
             (ichannel == (qint32)Shimmer3::ZWRAccel))
    {
      sensors |= (qint32)Shimmer3::SensorDAccel;
    }
    else if ((ichannel == (qint32)Shimmer3::XMag) ||
             (ichannel == (qint32)Shimmer3::YMag) ||
             (ichannel == (qint32)Shimmer3::ZMag))
    {
      sensors |= (qint32)Shimmer3::SensorMag;
    }
    else if ((ichannel == (qint32)Shimmer3::ExternalAdc7))
    {
      sensors |= (qint32)Shimmer3::SensorExtA7;
    }
    else if ((ichannel == (qint32)Shimmer3::ExternalAdc6))
    {
      sensors |= (qint32)Shimmer3::SensorExtA6;
    }
    else if ((ichannel == (qint32)Shimmer3::ExternalAdc15))
    {
      sensors |= (qint32)Shimmer3::SensorExtA15;
    }
    else if ((ichannel == (qint32)Shimmer3::InternalAdc1))
    {
      sensors |= (qint32)Shimmer3::SensorIntA1;
    }
    else if ((ichannel == (qint32)Shimmer3::InternalAdc12))
    {
      sensors |= (qint32)Shimmer3::SensorIntA12;
    }
    else if ((ichannel == (qint32)Shimmer3::InternalAdc13))
    {
      sensors |= (qint32)Shimmer3::SensorIntA13;
    }
    else if ((ichannel == (qint32)Shimmer3::InternalAdc14))
    {
      sensors |= (qint32)Shimmer3::SensorIntA14;
    }
    else if ((ichannel == (qint32)Shimmer3::Pressure))
    {
      sensors |= (qint32)Shimmer3::SensorPressure;
    }
    else if ((ichannel == (qint32)Shimmer3::Temperature))
    {
      sensors |= (qint32)Shimmer3::SensorPressure;
    }
    else if ((ichannel == (qint32)Shimmer3::GsrRaw))
    {
      sensors |= (qint32)Shimmer3::SensorGSR;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg1_CH1))
    {
      sensors |= (qint32)Shimmer3::SensorExg1;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg1_CH2))
    {
      sensors |= (qint32)Shimmer3::SensorExg1;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg2_CH1))
    {
      sensors |= (qint32)Shimmer3::SensorExg2;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg2_CH2))
    {
      sensors |= (qint32)Shimmer3::SensorExg2;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg1_CH1_16Bit))
    {
      sensors |= (qint32)Shimmer3::SensorExg1_16Bit;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg1_CH2_16Bit))
    {
      sensors |= (qint32)Shimmer3::SensorExg1_16Bit;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg2_CH1_16Bit))
    {
      sensors |= (qint32)Shimmer3::SensorExg2_16Bit;
    }
    else if ((ichannel == (qint32)Shimmer3::Exg2_CH2_16Bit))
    {
      sensors |= (qint32)Shimmer3::SensorExg2_16Bit;
    }
    numChannels++;
  }
}


void
ShimmerSensor::decodeCommand() {
  QList<quint8> buffer;
  qint32 i, j, nDati;
  bCompleteCommand = true;
  quint8* bufferbyte = NULL;

  while(bCompleteCommand) {
    try {
      if(isStreaming) {
        ShimmerDataPacket* packet = NULL;
        switch ((quint8)readBuffer[0]) {

        case(quint8)Shimmer3::INSTREAM_CMD_RESPONSE:
            if(readBuffer.count() < 3) {
              bCompleteCommand = false;
            } else if((quint8)readBuffer[1] == Shimmer3::STATUS_RESPONSE) {
              readBuffer = readBuffer.mid(3);
              bCompleteCommand = readBuffer.count() > 0;
              emit shimmerStatusReceived(this, (quint8)readBuffer[2]);
            } else if((quint8)readBuffer[1] == Shimmer3::DIR_RESPONSE) {
              qint32 iDirLen = (quint8)readBuffer[2];
              if(readBuffer.count()<iDirLen+3) {
                bCompleteCommand = false;
              } else {
                emit shimmerDirReceived(this, QString(readBuffer.mid(3)));
              }
            } else {
              throw QString("Bad Formed INSTREAM_CMD_RESPONSE !");
            }
            break;

        case (quint8)Shimmer3::DATA_PACKET:
          if(GetIsFilled()) {
            if(GetShimmerVersion() != (qint32)Shimmer::SHIMMER3) {
              throw QString("Shimmer Model NOT supported !");
            }

            qint32 extra = 0;
            qint32 enabledSensors = GetSensors();
            if((enabledSensors & (qint32)Shimmer3::SensorPressure) > 0)   extra = extra + 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg1) > 0)       extra = extra + 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg2) > 0)       extra = extra + 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg1_16Bit) > 0) extra = extra - 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg2_16Bit) > 0) extra = extra - 1;

            nDati = extra + 2 + (GetNumChannels() * 2);
            if(readBuffer.count() < nDati+1) {
              bCompleteCommand = false;
            } else {
              for(i=0; i<nDati; i++) {
                buffer.append((quint8)readBuffer[i+1]);//Discard the command
              }
              packet = new ShimmerDataPacket(buffer,
                                             GetNumAdcChannels(),
                                             GetNum1ByteDigiChannels(),
                                             GetNum2ByteDigiChannels(),
                                             GetShimmerVersion(),
                                             GetNumChannels(),
                                             this);

              if(keepPacket != NULL) {
                *keepPacket = *packet;
              } else {
                keepPacket = new ShimmerDataPacket(*packet);
              }
              readBuffer = readBuffer.mid(nDati+1);
              bCompleteCommand = readBuffer.count() > 0;
              buffer.clear();
              if(packet) delete packet;
              processPacket(keepPacket);
            }
          } else {
            sDebugMessage = QString();
            sInformation << dateTime.currentDateTime().toString()
                         << myRemoteAddress.toString()
                         << "DataPacket received while not configurated";
            if(pParent) {
              emit sendDebugMessage(sDebugMessage);
            } else {
               qDebug() << sDebugMessage;
            }
          }
          break;

        case (quint8)Shimmer3::ACK_COMMAND_PROCESSED:
          readBuffer = readBuffer.mid(1);
          bCompleteCommand = readBuffer.count() > 0;
          emit ackReceived(this);
          break;

        default:
          sDebugMessage = QString();
          sInformation << dateTime.currentDateTime().toString()
                       << myRemoteAddress.toString()
                       << "Unknown header received from: ";
          if(pParent) {
            emit sendDebugMessage(sDebugMessage);
          } else {
             qDebug() << sDebugMessage;
          }
          if(keepPacket) delete keepPacket;
          keepPacket = NULL;
          readBuffer = readBuffer.mid(1);
          bCompleteCommand = readBuffer.count() > 0;
          break;
        }

      } else {// Not streaming <-------------------------------------------------

        QString temp;
        switch ((quint8)readBuffer[0]) {

        case(quint8)Shimmer3::INSTREAM_CMD_RESPONSE:
            if(readBuffer.count() < 3) {
              bCompleteCommand = false;
            } else if((quint8)readBuffer[1] == Shimmer3::STATUS_RESPONSE) {
              quint8 status = (quint8)readBuffer[2];
              readBuffer = readBuffer.mid(3);
              bCompleteCommand = readBuffer.count() > 0;
              emit shimmerStatusReceived(this, status);
            } else if((quint8)readBuffer[1] == Shimmer3::DIR_RESPONSE) {
              qint32 iDirLen = (quint8)readBuffer[2];
              if(readBuffer.count()<iDirLen+3) {
                bCompleteCommand = false;
              } else {
                QString sDir = QString(readBuffer.mid(3));
                readBuffer = readBuffer.mid(iDirLen+3);
                bCompleteCommand = readBuffer.count() > 0;
                emit shimmerDirReceived(this, sDir);

              }
            } else {
              throw QString("Bad Formed INSTREAM_CMD_RESPONSE !");
            }
            break;

        case (quint8)Shimmer3::DATA_PACKET:

          if(GetIsFilled()) {
            if (GetShimmerVersion() != (qint32)Shimmer::SHIMMER3) {
              throw QString("Shimmer Model NOT supported !");
            }

            qint32 extra = 0;
            qint32 enabledSensors = GetSensors();
            if((enabledSensors & (qint32)Shimmer3::SensorPressure) > 0)   extra = extra + 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg1) > 0)       extra = extra + 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg2) > 0)       extra = extra + 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg1_16Bit) > 0) extra = extra - 1;
            if((enabledSensors & (qint32)Shimmer3::SensorExg2_16Bit) > 0) extra = extra - 1;

            nDati = extra + 2 + (GetNumChannels() * 2);
            if(readBuffer.count() < nDati+1) {
              bCompleteCommand = false;
            } else {
              readBuffer = readBuffer.mid(nDati+1);
              bCompleteCommand = readBuffer.count() > 0;
              buffer.clear();
            }
            sDebugMessage = QString();
            sInformation << dateTime.currentDateTime().toString()
                         << myRemoteAddress.toString()
                         << " Data Packet received while not streaming";
            if(pParent) {
              emit sendDebugMessage(sDebugMessage);
            } else {
               qDebug() << sDebugMessage;
            }
          } else {
            throw QString("Shimmer streaming while profile is not complete !");
          }

          break;

        case (quint8)Shimmer3::EXG_REGS_RESPONSE:
          if(readBuffer.count() < 11) {
            bCompleteCommand = false;
          } else {
            if (chipID == 1) {
              for(qint32 i=0; i<10; i++) {
                Exg1RegArray[i] = readBuffer[i+1];
              }
              readBuffer = readBuffer.mid(12);
              bCompleteCommand = readBuffer.count() > 0;
              emit exgRegs1Read(this);
            } else {
              for(qint32 i=0; i<10; i++) {
                Exg2RegArray[i] = (quint8)readBuffer[i+1];
              }
              readBuffer = readBuffer.mid(12);
              bCompleteCommand = readBuffer.count() > 0;
              emit exgRegs2Read(this);
            }
          }
          break;

        case (quint8)Shimmer3::INQUIRY_RESPONSE:
          if (GetShimmerVersion() != (qint32)Shimmer::SHIMMER3) {
            buffer.clear();
            throw QString("Shimmer Model NOT supported !");
          }
          if(readBuffer.count() < 8) {
            bCompleteCommand = false;
          } else if(readBuffer.count() < 9+(qint32)readBuffer[7]) {
            bCompleteCommand = false;
          } else {
            j = 1;
            // get Sampling rate, accel range, config setup byte0, num chans and buffer size
            for (i=0; i<8; i++) {
              buffer.append((quint8)readBuffer[j++]);// discard the first byte (the command !)
            }
            // read each channel type for the num channels
            for (i=0; i<(qint32)readBuffer[7]; i++) {
              buffer.append((quint8)readBuffer[j++]);
            }
            fillProfileShimmer3(buffer);
            readBuffer = readBuffer.mid(9+(qint32)readBuffer[7]);
            bCompleteCommand = readBuffer.count() > 0;
            buffer.clear();
            emit generalInquiryRead(this);
          }
          break;

        case (quint8)Shimmer3::SAMPLING_RATE_RESPONSE:
          if (GetShimmerVersion() != (qint32)Shimmer::SHIMMER3) {
            throw QString("Shimmer Model NOT supported !");
          }
          if(readBuffer.count() > 2) {
            qint32 value = 0;
            value = int((quint8)readBuffer[1]);
            value += ((int((quint8)readBuffer[2]) << 8) & 0xFF00);
            SetAdcSamplingRate(value);
            readBuffer = readBuffer.mid(3);
            bCompleteCommand = readBuffer.count() > 0;
            emit samplingRateRead(this);
          } else {
            bCompleteCommand = false;
          }
          break;

        case (quint8)Shimmer3::ACK_COMMAND_PROCESSED:
          readBuffer = readBuffer.mid(1);
          bCompleteCommand = readBuffer.count() > 0;
          emit ackReceived(this);
          break;

        case (quint8)Shimmer3::ALL_CALIBRATION_RESPONSE:
          if(GetShimmerVersion() != (qint32)Shimmer::SHIMMER3) {
            throw QString("ALL_CALIBRATION_RESPONSE: Sensor Version Unsupported !");
            break;
          }
          nDati = 1 + 4*21;
          if(readBuffer.count() < nDati) {
            bCompleteCommand = false;
          } else {
            //Retrieve Accel
            bufferbyte = new quint8[21];
            for (qint32 p = 0; p < 21; p++) {
              bufferbyte[p] = (quint8)readBuffer[p+1];
            }
            retrievecalibrationparametersfrompacket(bufferbyte, (quint8)Shimmer3::LSM303DLHC_ACCEL_RANGE_RESPONSE);

            //Retrieve Gyro
            for (qint32 p = 0; p < 21; p++) {
              bufferbyte[p] = (quint8)readBuffer[p+22];
            }
            retrievecalibrationparametersfrompacket(bufferbyte, (quint8)Shimmer3::MPU9150_GYRO_CALIBRATION_RESPONSE);

            //Retrieve Mag
            for (qint32 p = 0; p < 21; p++) {
              bufferbyte[p] = (quint8)readBuffer[p+43];
            }
            retrievecalibrationparametersfrompacket(bufferbyte, (quint8)Shimmer3::LSM303DLHC_MAG_CALIBRATION_RESPONSE);

            //Retrieve Digital Accel Cal Paramters if Shimmer 3
            for (qint32 p = 0; p < 21; p++) {
              bufferbyte[p] = (quint8)readBuffer[p+64];
            }
            retrievecalibrationparametersfrompacket(bufferbyte, (quint8)Shimmer3::LSM303DLHC_ACCEL_CALIBRATION_RESPONSE);

            readBuffer = readBuffer.mid(nDati+1);
            bCompleteCommand = readBuffer.count() > 0;
            emit allCalibrationsRead(this);
          }
          if(bufferbyte) delete [] bufferbyte;
          break;

        case (quint8)Shimmer3::FW_VERSION_RESPONSE:
          if(readBuffer.count() > 6) {
            SetFirmwareIdentifier((double)((readBuffer[2] & 0xFF) << 8) + (double)(readBuffer[1] & 0xFF));
            SetFirmwareVersion((double)((readBuffer[4] & 0xFF) << 8) + (double)(readBuffer[3] & 0xFF) + ((double)((readBuffer[5] & 0xFF)) / 10));
            SetFirmwareInternal((qint32)(readBuffer[6] & 0xFF));
            temp = QObject::tr("BtStream %1.%2").arg(GetFirmwareVersion(),3, 'g', 1).arg(GetFirmwareInternal());
            SetFirmVersionFullName(temp);
            readBuffer = readBuffer.mid(7);
            bCompleteCommand = readBuffer.count() > 0;
            emit firmwareRead(this);
          } else {
            bCompleteCommand = false;
          }
          break;

        case (quint8)Shimmer3::DEVICE_VERSION_RESPONSE:
          if(readBuffer.count() > 1) {
            SetShimmerVersion(readBuffer[1]);
            readBuffer = readBuffer.mid(2);
            bCompleteCommand = readBuffer.count() > 0;
            emit versionRead(this);
          } else {
            bCompleteCommand = false;
          }
          break;

        default:
          QString sString;
          sString.sprintf("Unprocessed BYTE: 0x%02x", (quint8)readBuffer.at(0));
          sDebugMessage = QString();
          sInformation << dateTime.currentDateTime().toString()
                       << myRemoteAddress.toString()
                        << sString;
          if(pParent) {
            emit sendDebugMessage(sDebugMessage);
          } else {
             qDebug() << sDebugMessage;
          }
          readBuffer.remove(0, 1);
          break;
        }
      }
    }
    catch(QString sString) {
      sDebugMessage = QString();
      sInformation << dateTime.currentDateTime().toString()
                   << myRemoteAddress.toString()
                   << " Exception rised. ShimmerSensor::decodeCommand() "
                   << sString;
      if(pParent) {
        emit sendDebugMessage(sDebugMessage);
      } else {
         qDebug() << sDebugMessage;
      }
      readBuffer.clear();
      bCompleteCommand = readBuffer.count() > 0;
    }
    catch(...) {
      sDebugMessage = QString();
      sInformation << dateTime.currentDateTime().toString()
                   << myRemoteAddress.toString()
                   << " Exception rised. Unknown Exception in ShimmerSensor::decodeCommand()";
      if(pParent) {
        emit sendDebugMessage(sDebugMessage);
      } else {
         qDebug() << sDebugMessage;
      }
      readBuffer.remove(0,1);
      bCompleteCommand = readBuffer.count() > 0;
    }
  }// while(...)
}


void
ShimmerSensor::jsonRead(const QJsonObject &json) {
  shimmerVersion      = json["shimmerVersion"].toInt();
  sensors             = json["sensors"].toInt();
  adcSamplingRate     = json["adcSamplingRate"].toInt();
  configSetupByte0    = json["configSetupByte0"].toInt();
  accelRange          = json["accelRange"].toInt();
  gyroRange           = json["gyroRange"].toInt();
  magGain             = json["magGain"].toInt();
  accelSamplingRate   = json["accelSamplingRate"].toInt();
  mpu9150SamplingRate = json["mpu9150SamplingRate"].toInt();
  magSamplingRate     = json["magSamplingRate"].toInt();
  pressureResolution  = json["pressureResolution"].toInt();
  gsrRange            = json["gsrRange"].toInt();
  internalExpPower    = json["internalExpPower"].toInt();
  enableLowPowerMag   = json["enableLowPowerMag"].toBool();
  enableLowPowerAccel = json["enableLowPowerAccel"].toBool();
  enableLowPowerGyro  = json["enableLowPowerGyro"].toBool();
  numChannels         = json["numChannels"].toInt();
  bufferSize          = json["bufferSize"].toInt();

  QJsonArray channelArray = json["channels"].toArray();
  channels.clear();
  for(int i = 0; i < channelArray.size(); i++) {
    channels.append(channelArray[i].toInt());
  }

  QJsonArray dataTypeArray = json["dataTypes"].toArray();
  signalDataTypeArray.clear();
  for(int i = 0; i < dataTypeArray.size(); i++) {
    signalDataTypeArray.append(dataTypeArray[i].toString());
  }

  QJsonArray orientationArray = json["initialOrientation"].toArray();
  initialOrientation = QQuaternion(orientationArray[0].toDouble(),
                                   orientationArray[1].toDouble(),
                                   orientationArray[2].toDouble(),
                                   orientationArray[3].toDouble());

  isFilled = true;
}


void
ShimmerSensor::jsonWrite(QJsonObject &json) const {
  json["shimmerVersion"]      = shimmerVersion;
  json["sensors"]             = sensors;
  json["adcSamplingRate"]     = adcSamplingRate;
  json["configSetupByte0"]    = configSetupByte0;
  json["accelRange"]          = accelRange;
  json["gyroRange"]           = gyroRange;
  json["magGain"]             = magGain;
  json["accelSamplingRate"]   = accelSamplingRate;
  json["mpu9150SamplingRate"] = mpu9150SamplingRate;
  json["magSamplingRate"]     = magSamplingRate;
  json["pressureResolution"]  = pressureResolution;
  json["gsrRange"]            = gsrRange;
  json["internalExpPower"]    = internalExpPower;
  json["enableLowPowerMag"]   = enableLowPowerMag;
  json["enableLowPowerAccel"] = enableLowPowerAccel;
  json["enableLowPowerGyro"]  = enableLowPowerGyro;
  json["numChannels"]         = numChannels;
  json["bufferSize"]          = bufferSize;

  QJsonArray channelArray;
  foreach (qint32 channel, channels) {
    channelArray.append(channel);
  }
  json["channels"] = channelArray;

  QJsonArray dataTypeArray;
  foreach (QString dataType, signalDataTypeArray) {
    dataTypeArray.append(dataType);
  }
  json["dataTypes"] = dataTypeArray;

  QJsonArray orientationArray;
  orientationArray.append(initialOrientation.scalar());
  orientationArray.append(initialOrientation.x());
  orientationArray.append(initialOrientation.y());
  orientationArray.append(initialOrientation.z());
  json["initialOrientation"] = orientationArray;
}
