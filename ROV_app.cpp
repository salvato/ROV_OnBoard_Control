#include "ROV_app.h"

#include <math.h>
#include <errno.h>
#include <unistd.h>// for sleep()
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <QDebug>
#include <QtNetwork>
#include <QtSerialPort/QSerialPortInfo>

#include "graddes3dorientation.h"

/*
 *
void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
  {
      QByteArray localMsg = msg.toLocal8Bit();
      switch (type) {
      case QtDebugMsg:
          fprintf(stderr, "Debug: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
          break;
      case QtInfoMsg:
          fprintf(stderr, "Info: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
          break;
      case QtWarningMsg:
          fprintf(stderr, "Warning: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
          break;
      case QtCriticalMsg:
          fprintf(stderr, "Critical: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
          break;
      case QtFatalMsg:
          fprintf(stderr, "Fatal: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
          abort();
      }
  }

  int main(int argc, char **argv)
  {
      qInstallMessageHandler(myMessageOutput);
      QApplication app(argc, argv);
      ...
      return app.exec();
  }

*/


/*
 *

// how to show the Raspberry PI's Temperature.
/opt/vc/bin/vcgencmd measure_temp

*/


/*

// rename example
#include <stdio.h>

int main ()
{
  int result;
  char oldname[] ="oldname.txt";
  char newname[] ="newname.txt";
  result= rename( oldname , newname );
  if ( result == 0 )
    puts ( "File successfully renamed" );
  else
    perror( "Error renaming file" );
  return 0;
}

*/

ROV_App::ROV_App(int argc, char *argv[])
  : QCoreApplication(argc, argv)

  // TCP-IP Server
  , pTcpServer(NULL)
  , pTcpServerConnection(NULL)
  , serverPort(43210)

  // Arduino Serial Port
  , baudRate(115200)
  , waitTimeout(1000)

  , bUseBluetooth(false)
  , bUseLowNoiseAccelerator(false)
  , bEnableGyroOnTheFlyCalibration(true)
  , shimmerWatchDogTime(30000)

#ifdef Q_PROCESSOR_ARM
//  , shimmerBtAdress(QBluetoothAddress("00:06:66:66:94:B9"))
  , shimmerBtAdress(QBluetoothAddress("00:06:66:66:93:FF"))
#else
  , shimmerBtAdress(QBluetoothAddress("00:06:66:66:93:FF"))
#endif

#ifdef Q_OS_LINUX
  , shimmerComPort("/dev/rfcomm0")
#else
  , shimmerComPort("COM9")
#endif

  , pShimmerSensor(NULL)

  // Costanti
  , ACK(char(6))
  , MOTOR_OFF(1)
  , MOTOR_ON(0)
  , MOTOR_FORWARD(0)
  , MOTOR_REVERSE(1)
  , AIR_VALVE_OFF(0)
  , AIR_VALVE_ON(1)

  // Status Variables
  , MotorDxStatus(MotorOff)
  , MotorSnStatus(MotorOff)
  , iCurrentUpDown(0)

  , iLastControllerX(0)
  , iLastControllerY(0)
  , iLastUpDown(0)

  , updateTime(100)
  , connectionWatchDogTime(10000)
{
  sInformation.setString(&sDebugMessage);

//  QString sUsbDevicesFile = "./usbDevices.txt";
//  system((QString("lsusb > ") + sUsbDevicesFile).toLatin1());
//  QFile usbDevicesFile(sUsbDevicesFile);
//  if(usbDevicesFile.exists()) {
//    if(usbDevicesFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
//      QTextStream usbDevices(&usbDevicesFile);
//      QString sLine;
//      while(!usbDevices.atEnd()) {
//        sLine = usbDevices.readLine();
//        if(sLine.contains("Bluetooth")) {
//          QString sBus, sDev;
//          QStringList sValues = sLine.split(QString(" "));
//          for(int i=0; i<sValues.count()-2; i++) {
//            if(sValues.at(i) == "Bus") {
//              sBus = sValues.at(i+1);
//              i++;
//            }
//            if(sValues.at(i) == "Device") {
//              sDev = sValues.at(i+1);
//              i++;
//            }
//          }
//          sUsbDeviceFile = QString("/dev/bus/usb/%1/%2").arg(sBus).arg(sDev.left(3));
//        }
//      }
//      if(!sUsbDevicesFile.isEmpty())
//        usbReset(sUsbDeviceFile);
//    }
//  }

  // Motore Destro
  minMotorDx = -10.0;
  maxMotorDx =  10.0;

  // Motore Sinistro
  minMotorSn = -10.0;
  maxMotorSn =  10.0;

  init();
  connect(&connectionWatchDogTimer, SIGNAL(timeout()),
          this, SLOT(onConnectionWatchDogTimeout()));
}


ROV_App::~ROV_App() {
  if(pTcpServer) pTcpServer->close();
  serialPort.close();
}


void
ROV_App::destroy() {
  updateTimer.stop();
  if(pShimmerSensor) {
    if(pShimmerSensor->currentStatus != unconnectedStatus)
      pShimmerSensor->disconnect();
  }
  exit(0);
}


void
ROV_App::stopStreaming() {
  if(pShimmerSensor) {
    if(pShimmerSensor->currentStatus != unconnectedStatus) {
      quint8 data;
      data = (quint8)Shimmer3::STOP_STREAMING_COMMAND;
      pShimmerSensor->currentStatus = stoppingStreamingStatus;
      pShimmerSensor->writeCommand(&data, 1);
      pShimmerSensor->stopWatchDogTimer();
    }
  }
}


void
ROV_App::switchOff() {
  if(pShimmerSensor) {
    stopStreaming();
    niWantToCloseAttempt = 0;
    connect(&iWantToCloseTimer, SIGNAL(timeout()),
            this, SLOT(iWantToCloseTimerTimeout()));
    iWantToCloseTimer.start(500);
  } else {
    qDebug() << dateTime.currentDateTime().toString()
             << " No streaming sensors: Closing ...";
    emit destroyMe();
  }
}


void
ROV_App::forwardDebugMessage(QString sDebugMessage) {
  qDebug() << sDebugMessage;
}


void
ROV_App::iWantToCloseTimerTimeout() {
  niWantToCloseAttempt++;
  if(niWantToCloseAttempt > 20 || !pShimmerSensor->isStreaming) {
    iWantToCloseTimer.stop();
    emit destroyMe();
  } else {
    iWantToCloseTimer.start(500);
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " Shimmers still streaming";
    qDebug() << sDebugMessage;
  }
}


int
ROV_App::init() {
  if(openTcpSession()) {
    ErrorHandler(QString("Impossible to open a TCP-IP Session !"));
    return -1;
  }

  bUseBluetooth = CheckBluetoothSupport();

//  if(!bUseBluetooth) {
//    sCommand = QString("rfcomm unbind 0");
//    int iResult = system(sCommand.toLatin1());
//    sCommand = QString("rfcomm bind 0 ") + shimmerBtAdress.toString();
//    iResult = system(sCommand.toLatin1());
//  }

  // Sensors to enable for each Shimmer
  activeSensors    = Shimmer3::SensorGyro |
                     Shimmer3::SensorMag;
  if(bUseLowNoiseAccelerator)
    activeSensors  |= Shimmer3::SensorAAccel;// Use the Low Noise Analog Accelerometer
  else
    activeSensors  |= Shimmer3::SensorDAccel;// Use the Wide Range Digital Accelerometer (default)

  // Sensors Ranges & sampling rate
  acceleratorRange = Shimmer3::RANGE_16_0G;
  gyroRange        = Shimmer3::RANGE_2000DPS;
  magGain          = Shimmer3::RANGE_8_1Ga;
  samplingRate     = Shimmer3::Sampling50Hz;

  pShimmerSensor = new ShimmerSensor(this,
                                     shimmerBtAdress,
                                     shimmerComPort,
                                     0);

  connect(this,           SIGNAL(destroyMe()),
          this, SLOT(destroy()));
  connect(&updateTimer,   SIGNAL(timeout()),
          this, SLOT(periodicUpdateWidgets()));
  connect(pShimmerSensor, SIGNAL(sendDebugMessage(QString)),
          this, SLOT(forwardDebugMessage(QString)));

  connect(pShimmerSensor, SIGNAL(shimmerConnected(ShimmerSensor*)),
          this, SLOT(onNewShimmerConnected(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(shimmerDisconnected(ShimmerSensor*)),
          this, SLOT(onShimmerDisconnected(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(shimmerFailedToConnect(ShimmerSensor*)),
          this, SLOT(onShimmerFailedToConnect(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(watchDogTimerTimeout(ShimmerSensor*)),
          this, SLOT(onShimmerWatchDogTimeout(ShimmerSensor*)));

  // Initialize Shimmer Messages
  connect(pShimmerSensor, SIGNAL(shimmerStatusReceived(ShimmerSensor*, quint8)),
          this, SLOT(onShimmerStatusReceived(ShimmerSensor*, quint8)));
  connect(pShimmerSensor, SIGNAL(firmwareRead(ShimmerSensor*)),
          this, SLOT(onFirmwareRead(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(versionRead(ShimmerSensor*)),
          this, SLOT(onShimmerVersionRead(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(samplingRateRead(ShimmerSensor*)),
          this, SLOT(onSamplingRateObtained(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(generalInquiryRead(ShimmerSensor*)),
          this, SLOT(onGeneralInquiryObtained(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(exgRegs1Read(ShimmerSensor*)),
          this, SLOT(onExgRegs1Obtained(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(exgRegs2Read(ShimmerSensor*)),
          this, SLOT(onExgRegs2Obtained(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(allCalibrationsRead(ShimmerSensor*)),
          this, SLOT(onCalibrationsObtained(ShimmerSensor*)));
  connect(pShimmerSensor, SIGNAL(ackReceived(ShimmerSensor*)),
          this, SLOT(onAckReceived(ShimmerSensor*)));

  if(connectToArduino()) {
    ErrorHandler(QString("no Arduino ready to use !"));
//    return -1;
  }

  if(bUseBluetooth)
    pShimmerSensor->BtSetup();
  else
    pShimmerSensor->ComSetup();

  return SetSpeed(0, 0);
}


void
ROV_App::onNewShimmerConnected(ShimmerSensor* currentShimmer) {
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " "
                << currentShimmer->myRemoteAddress.toString()
                << " is now Connected ";
  qDebug() << sDebugMessage;

  currentShimmer->currentStatus = connectedStatus;
  sleep(1);
  initShimmer(currentShimmer);
  updateTimer.start(updateTime);
}


void
ROV_App::initShimmer(ShimmerSensor* currentShimmer) {
  quint8 data;
  data = (quint8)Shimmer3::GET_STATUS_COMMAND;
  currentShimmer->writeCommand(&data, 1);
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " "
                << currentShimmer->myRemoteAddress.toString()
                << " Asking Shimmer Status ";
  qDebug() << sDebugMessage;
}


void
ROV_App::onShimmerFailedToConnect(ShimmerSensor* currentShimmer) {
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " "
                << currentShimmer->myRemoteAddress.toString()
                << " Failed to Connect ";
  qDebug() << sDebugMessage;
}


void
ROV_App::onShimmerDisconnected(ShimmerSensor* currentShimmer) {
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " "
                << currentShimmer->myRemoteAddress.toString()
                << " Disconnected.";
  qDebug() << sDebugMessage;
  //destroy();// <<========================================== To Be Changed !!!!!!!!!!!!!
}


bool
ROV_App::CheckBluetoothSupport() {
  // Check if Bluetooth is available on this device
  QList<QBluetoothHostInfo> localDevices = QBluetoothLocalDevice::allDevices();
  if(localDevices.isEmpty()) {
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << " No Bluetooth support found";
    qDebug() << sDebugMessage;
    return false;
  } else {
    // Use the first device (to be changed...)
    adapter = new QBluetoothLocalDevice(localDevices.first().address());
    if(!adapter->isValid()) {
      sDebugMessage = QString();
      sInformation  << dateTime.currentDateTime().toString()
                    << " No Valid Bluetooth interface found";
      qDebug() << sDebugMessage;
      return false;
    }
    adapter->setHostMode(QBluetoothLocalDevice::HostDiscoverable);
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << " Using Buetooth interface at address: "
                 << localDevices.first().address().toString();
    qDebug() << sDebugMessage;
  }
  return true;
}


int
ROV_App::connectToArduino() {
  QList<QSerialPortInfo> serialPorts = QSerialPortInfo::availablePorts();
  if(serialPorts.isEmpty()) {
    ErrorHandler(QString("no Arduino's connected !"));
    return -1;
  }
  bool found = false;
  QSerialPortInfo info;
  for(int i=0; i<serialPorts.size()&& !found; i++) {
    info = serialPorts.at(i);
    serialPort.setPortName(info.portName());
    serialPort.setBaudRate(115200);
    if(serialPort.open(QIODevice::ReadWrite)) {
      requestData = QByteArray(2, char(AreYouThere));
      sleep(3);
      if(writeRequest(requestData) == 0)
        found = true;
      else
        serialPort.close();
    }
  }
  if(!found) {
    return -1;
  }
  qDebug() << "Arduino found at: " << info.portName();
  return 0;
}


int
ROV_App::setSpeedAndSteering(int iX, int iY) {
  if(SetSpeed(iX, iY) != 0) return -1;
  return 0;
}


int
ROV_App::SetSpeed(int iX, int iY) {
  if(SetMotorDxSpeed(iX+iY) != 0) {
    return -1;
  }
  if(SetMotorSnSpeed(iY-iX) != 0) {
    return -1;
  }
  return 0;
}


void
ROV_App::ErrorHandler(QString sErrorString) {
  // MUST BE MODIFIED !!
  qDebug() << "Error ! " << sErrorString << endl;
}


int
ROV_App::writeRequest(QByteArray requestData) {
  serialPort.write(requestData.append(char(127)));
  if (serialPort.waitForBytesWritten(waitTimeout)) {
    if (serialPort.waitForReadyRead(waitTimeout)) {
      QByteArray responseData = serialPort.readAll();
      while(serialPort.waitForReadyRead(10))
        responseData += serialPort.readAll();
      if (responseData.at(0) != ACK) {
        QString response(responseData);
//      if(response != QString(ACK)) {
        ErrorHandler(tr("NACK on Command %1: expecting %2 read %3")
                     .arg(int(requestData.at(0)))
                     .arg(int(ACK))
                     .arg(int(response.at(0).toLatin1())));
        //return -1;
      }
    } else {
      ErrorHandler(tr(" Wait read response timeout %1")
                   .arg(QTime::currentTime().toString()));
      return -1;
    }
  } else {
    ErrorHandler(tr(" Wait write request timeout %1")
                 .arg(QTime::currentTime().toString()));
    return -1;
  }
  return 0;
}


int
ROV_App::SetAirValveIn(int iValue) {
  uInt8 dataOutput = iValue ? uInt8(AIR_VALVE_ON) : uInt8(AIR_VALVE_OFF);
  requestData = QByteArray(1, char(InflateValve));
  requestData.append(char(dataOutput));
  return writeRequest(requestData);
}


int
ROV_App::SetAirValveOut(int iValue) {
  uInt8 dataOutput = iValue ? uInt8(AIR_VALVE_ON) : uInt8(AIR_VALVE_OFF);
  requestData = QByteArray(1, char(DeflateValve));
  requestData.append(char(dataOutput));
  return writeRequest(requestData);
}


int
ROV_App::SwitchMotorDxOff() {
  if(MotorDxStatus == MotorForward) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(RightForward));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  else if(MotorDxStatus == MotorReverse) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(RightReverse));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  MotorDxStatus = MotorOff;
  return 0;
}


int
ROV_App::SwitchMotorDxForward() {
  if(MotorDxStatus == MotorForward)
    return 0;
  if(MotorDxStatus == MotorReverse) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(RightReverse));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  uInt8 dataOutput = uInt8(MOTOR_ON);
  requestData = QByteArray(1, char(RightForward));
  requestData.append(char(dataOutput));
  if(writeRequest(requestData))
    return -1;
  MotorDxStatus = MotorForward;
  return 0;
}


int
ROV_App::SwitchMotorDxReverse() {
  if(MotorDxStatus == MotorReverse)
    return 0;
  if(MotorDxStatus == MotorForward) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(RightForward));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  uInt8 dataOutput = uInt8(MOTOR_ON);
  requestData = QByteArray(1, char(RightReverse));
  requestData.append(char(dataOutput));
  if(writeRequest(requestData))
    return -1;
  MotorDxStatus = MotorReverse;
  return 0;
}


int
ROV_App::SwitchMotorSnOff() {
  if(MotorSnStatus == MotorForward) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(LeftForward));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  else if(MotorSnStatus == MotorReverse) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(LeftReverse));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  MotorSnStatus = MotorOff;
  return 0;
}


int
ROV_App::SwitchMotorSnForward() {
  if(MotorSnStatus == MotorForward)
    return 0;
  if(MotorSnStatus == MotorReverse) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(LeftReverse));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  uInt8 dataOutput = uInt8(MOTOR_ON);
  requestData = QByteArray(1, char(LeftForward));
  requestData.append(char(dataOutput));
  if(writeRequest(requestData))
    return -1;
  MotorSnStatus = MotorForward;
  return 0;
}


int
ROV_App::SwitchMotorSnReverse() {
  if(MotorSnStatus == MotorReverse)
    return 0;
  if(MotorSnStatus == MotorForward) {
    uInt8 dataOutput = uInt8(MOTOR_OFF);
    requestData = QByteArray(1, char(LeftForward));
    requestData.append(char(dataOutput));
    if(writeRequest(requestData))
      return -1;
  }
  uInt8 dataOutput = uInt8(MOTOR_ON);
  requestData = QByteArray(1, char(LeftReverse));
  requestData.append(char(dataOutput));
  if(writeRequest(requestData))
    return -1;
  MotorSnStatus = MotorReverse;
  return 0;
}


int
ROV_App::SetMotorDxSpeed(int newSpeed) {
  int iResult;
  if(newSpeed >= 0)
    iResult= SwitchMotorDxForward();
  else
    iResult = SwitchMotorDxReverse();
  if(iResult != 0)
    return -1;

  int dataOutput = int(double(newSpeed)*(maxMotorDx-minMotorDx)/20.0+0.5);
  if(dataOutput >  10) dataOutput = 10;
  if(dataOutput < -10) dataOutput = -10;
  //qDebug() << "Dx Speed = " << dataOutput;
  dataOutput = abs(dataOutput);
  requestData = QByteArray(1, char(RightSpeed));
  requestData.append(char(dataOutput));
  return writeRequest(requestData);
}


int
ROV_App::SetMotorSnSpeed(int newSpeed) {
  int iResult;
  if(newSpeed >= 0)
    iResult= SwitchMotorSnForward();
  else
    iResult = SwitchMotorSnReverse();
  if(iResult != 0) return -1;

  int dataOutput = int(double(newSpeed)*(maxMotorDx-minMotorDx)/20.0+0.5);
  if(dataOutput >  10) dataOutput = 10;
  if(dataOutput < -10) dataOutput = -10;
  //qDebug() << "Sn Speed = " << dataOutput;
  dataOutput = abs(dataOutput);
  requestData = QByteArray(1, char(LeftSpeed));
  requestData.append(char(dataOutput));
  return writeRequest(requestData);
}


int
ROV_App::SetUpDown(int newUpDown) {
  if(newUpDown < -10) newUpDown = -10;
  if(newUpDown >  10) newUpDown =  10;
  requestData = QByteArray(1, char(FrontThruster));
  requestData.append(char(newUpDown));
  iCurrentUpDown = newUpDown;
  //qDebug() << iCurrentUpDown;
  writeRequest(requestData);
  requestData = QByteArray(1, char(BackThruster));
  requestData.append(char(newUpDown));
  return writeRequest(requestData);
}


int
ROV_App::SetPitch(int newPitch) {
  if(newPitch < -10) newPitch = -10;
  if(newPitch >  10) newPitch =  10;
  requestData = QByteArray(1, char(FrontThruster));
  requestData.append(char(newPitch));
  iCurrentPitch = newPitch;
  //qDebug() << iCurrentPitch;
  writeRequest(requestData);
  requestData = QByteArray(1, char(BackThruster));
  requestData.append(char(-newPitch));
  return writeRequest(requestData);
}


int
ROV_App::openTcpSession() {
//  qDebug() << "Host Name: "
//           << QHostInfo::localHostName();
//  foreach (const QHostAddress &address, QNetworkInterface::allAddresses()) {
//    if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost)) {
//      qDebug() << address.toString();
//    }
//  }
  pTcpServer = new QTcpServer(this);
  if(!pTcpServer->listen(QHostAddress::Any, serverPort)) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << "TCP-IP Unable to start listen()";
    ErrorHandler(sDebugMessage);
    return -1;
  }
  connect(pTcpServer, SIGNAL(newConnection()),
          this, SLOT(newTcpConnection()));
  connect(pTcpServer, SIGNAL(acceptError(QAbstractSocket::SocketError)),
          this, SLOT(tcpError(QAbstractSocket::SocketError)));
  QString ipAddress;
  QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
  // use the first non-localhost IPv4 address
  for(qint32 i=0; i<ipAddressesList.size(); ++i) {
    if(ipAddressesList.at(i) != QHostAddress::LocalHost && ipAddressesList.at(i).toIPv4Address()) {
      ipAddress = ipAddressesList.at(i).toString();
      if(ipAddress.left(3) != QString("169")) break;
    }
  }
  // if we did not find one, use IPv4 localhost
  if(ipAddress.isEmpty()) ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
  sDebugMessage = QString();
  sInformation  << " Running TCP-IP server at address " + ipAddress + " port: "
                << pTcpServer->serverPort();
  qDebug() << sDebugMessage;
  return 0;
}


void
ROV_App::tcpError(QAbstractSocket::SocketError error) {
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString();
  if(error == QAbstractSocket::ConnectionRefusedError)
    sInformation << " The connection was refused by the peer (or timed out).";
  else if(error == QAbstractSocket::RemoteHostClosedError) {
    sInformation << " The remote host closed the connection.";
    qDebug() << sDebugMessage;
    return;
  } else if(error == QAbstractSocket::HostNotFoundError)
    sInformation << " The host address was not found.";
  else if(error == QAbstractSocket::SocketAccessError)
    sInformation << " The socket operation failed because the application lacked the required privileges.";
  else if(error == QAbstractSocket::SocketResourceError)
    sInformation << " The local system ran out of resources (e.g., too many sockets).";
  else if(error == QAbstractSocket::SocketTimeoutError)
    sInformation << " The socket operation timed out.";
  else if(error == QAbstractSocket::DatagramTooLargeError)
    sInformation << " The datagram was larger than the operating system's limit (which can be as low as 8192 bytes).";
  else if(error == QAbstractSocket::NetworkError)
    sInformation << " An error occurred with the network (e.g., the network cable was accidentally plugged out).";
  else if(error == QAbstractSocket::AddressInUseError)
    sInformation << " The address specified to QAbstractSocket::bind() is already in use and was set to be exclusive.";
  else if(error == QAbstractSocket::SocketAddressNotAvailableError)
    sInformation << " The address specified to QAbstractSocket::bind() does not belong to the host.";
  else if(error == QAbstractSocket::UnsupportedSocketOperationError)
    sInformation << " The requested socket operation is not supported by the local operating system (e.g., lack of IPv6 support).";
  else if(error == QAbstractSocket::ProxyAuthenticationRequiredError)
    sInformation << " The socket is using a proxy, and the proxy requires authentication.";
  else if(error == QAbstractSocket::SslHandshakeFailedError)
    sInformation << " The SSL/TLS handshake failed, so the connection was closed (only used in QSslSocket)";
  else if(error == QAbstractSocket::UnfinishedSocketOperationError)
    sInformation << " Used by QAbstractSocketEngine only, The last operation attempted has not finished yet (still in progress in the background).";
  else if(error == QAbstractSocket::ProxyConnectionRefusedError)
    sInformation << " Could not contact the proxy server because the connection to that server was denied";
  else if(error == QAbstractSocket::ProxyConnectionClosedError)
    sInformation << " The connection to the proxy server was closed unexpectedly (before the connection to the final peer was established)";
  else if(error == QAbstractSocket::ProxyConnectionTimeoutError)
    sInformation << " The connection to the proxy server timed out or the proxy server stopped responding in the authentication phase.";
  else if(error == QAbstractSocket::ProxyNotFoundError)
    sInformation << " The proxy address set with setProxy() (or the application proxy) was not found.";
  else if(error == QAbstractSocket::ProxyProtocolError)
    sInformation << " The connection negotiation with the proxy server failed, because the response from the proxy server could not be understood.";
  else if(error == QAbstractSocket::OperationError)
    sInformation << " An operation was attempted while the socket was in a state that did not permit it.";
  else if(error == QAbstractSocket::SslInternalError)
    sInformation << " The SSL library being used reported an internal error. This is probably the result of a bad installation or misconfiguration of the library.";
  else if(error == QAbstractSocket::SslInvalidUserDataError)
    sInformation << " Invalid data (certificate, key, cypher, etc.) was provided and its use resulted in an error in the SSL library.";
  else if(error == QAbstractSocket::TemporaryError)
    sInformation << " A temporary error occurred (e.g., operation would block and socket is non-blocking).";
  else if(error == QAbstractSocket::UnknownSocketError)
    sInformation << " An unidentified error occurred.";

  ErrorHandler(sDebugMessage);
}


void
ROV_App::newTcpConnection() {
  pTcpServerConnection = pTcpServer->nextPendingConnection();
  connect(pTcpServerConnection, SIGNAL(readyRead()),
          this, SLOT(readFromServer()));
  connect(pTcpServerConnection, SIGNAL(error(QAbstractSocket::SocketError)),
          this, SLOT(tcpError(QAbstractSocket::SocketError)));
  connect(pTcpServerConnection, SIGNAL(disconnected()),
          this, SLOT(tcpClientDisconnected()));

  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " Connected to: "
                << pTcpServerConnection->peerAddress().toString();
  qDebug() << sDebugMessage;
  SetSpeed(0, 0);
  SetAirValveOut(AIR_VALVE_OFF);
  SetAirValveIn(AIR_VALVE_OFF);
  connectionWatchDogTimer.start(connectionWatchDogTime);
}


void
ROV_App::tcpClientDisconnected() {
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " Disconnection from: "
                << pTcpServerConnection->peerAddress().toString();
  qDebug() << sDebugMessage;
  pTcpServerConnection = NULL;
  connectionWatchDogTimer.stop();
}


void
ROV_App::readFromServer() {
  message.append(pTcpServerConnection->readAll());
  while(message.length() > 1) {
    int iTarget = int(message.at(0));
    int iValue = int8(message.at(1));
    message.remove(0, 2);
    executeCommand(iTarget, iValue);
  }
}


void
ROV_App::executeCommand(int iTarget, int iValue) {
  if(iTarget == yAxisController) {
    iValue = - iValue;
    //qDebug() << "x=" << iLastControllerX << "y=" << iValue;
    if(setSpeedAndSteering(iLastControllerX, iValue) != 0) {
      ErrorHandler("Unable to set Motor speeds");
      return;
    }
    iLastControllerY = iValue;
  } else if(iTarget == xAxisController) {
    //qDebug() << "x=" << iValue << "y=" << iLastControllerY;
    if(setSpeedAndSteering(iValue, iLastControllerY) != 0) {
      ErrorHandler("Unable to set Motor speeds");
      return;
    }
    iLastControllerX = iValue;
  } else if(iTarget == DeflateButton) {
    SetAirValveOut(iValue);
  } else if(iTarget == InflateButton) {
    SetAirValveIn(iValue);
  } else if(iTarget == upDownAxis) {
    SetUpDown(iValue);
  } else if(iTarget == pitchAxis) {
    SetPitch(iValue);
  } else if(iTarget == 126) {
      connectionWatchDogTimer.start(connectionWatchDogTime);
      if(pTcpServerConnection) {
        if(pTcpServerConnection->isOpen()) {
          QString message;
            message = QString("alive#");
            pTcpServerConnection->write(message.toLatin1());
        }
      }
  }
}


void
ROV_App::onShimmerStatusReceived(ShimmerSensor* currentShimmer, quint8 shimmerStatus) {
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " "
                << currentShimmer->myRemoteAddress.toString()
                << " Shimmer status received: "
                << shimmerStatus;
  qDebug() << sDebugMessage;
  AskFirmwareVersion(currentShimmer);
}


void
ROV_App::AskFirmwareVersion(ShimmerSensor *currentShimmer) {
  quint8 data;
  data = (quint8)Shimmer3::GET_FW_VERSION_COMMAND;
  currentShimmer->writeCommand(&data, 1);
  currentShimmer->currentStatus = waitingFirmwVersStatus;
}


void
ROV_App::onFirmwareRead(ShimmerSensor *currentShimmer) {
  if(currentShimmer->GetFirmwareVersionFullName() == tr("BoilerPlate 0.1.0")) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " is Not a Shimmer3 model !";
    qDebug() << sDebugMessage;
    return;
  }
  if(currentShimmer->GetFirmwareVersion() == 1.2) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " Shimmer3 bad firmware version found !";
    qDebug() << sDebugMessage;
    return;
  }
  quint8 data = (quint8)Shimmer3::GET_DEVICE_VERSION_COMMAND;
  currentShimmer->writeCommand(&data, 1);

  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " "
                << currentShimmer->myRemoteAddress.toString()
                << " Asking Shimmer Version";
  qDebug() << sDebugMessage;

  currentShimmer->currentStatus = waitingShimmerVersStatus;
}


void
ROV_App::onShimmerVersionRead(ShimmerSensor *currentShimmer) {
  if(currentShimmer->currentStatus != waitingShimmerVersStatus) return;
  if(currentShimmer->GetShimmerVersion() != (int)Shimmer::SHIMMER3) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " No Shimmer3 device !";
    qDebug() << sDebugMessage;
    return;
  }
  setActiveSensors(currentShimmer);
}


void
ROV_App::setActiveSensors(ShimmerSensor *currentShimmer) {
  currentShimmer->currentStatus = waitingSettingSensorsStatus;
  quint8 command[4];
  command[0] = Shimmer3::SET_SENSORS_COMMAND;
  command[1] = (quint8)(activeSensors & 0xff);
  command[2] = (quint8)(activeSensors >> 8 & 0xff);
  command[3] = (quint8)(activeSensors >> 16 & 0xff);
  currentShimmer->writeCommand(command, 4);
}


void
ROV_App::setAcceleratorRange(ShimmerSensor *currentShimmer) {
  currentShimmer->currentStatus = waitingSettingAccRangeStatus;
  quint8 command[2];
  command[0] = (quint8)Shimmer3::SET_LSM303DLHC_ACCEL_RANGE_COMMAND;
  command[1] = (quint8)acceleratorRange;
  currentShimmer->writeCommand(command, 2);
}


void
ROV_App::setGyroRange(ShimmerSensor *currentShimmer) {
  currentShimmer->currentStatus = waitingSettingGyroRangeStatus;
  quint8 command[2];
  command[0] = (quint8)Shimmer3::SET_MPU9150_GYRO_RANGE_COMMAND;
  command[1] = (quint8)gyroRange;
  currentShimmer->writeCommand(command, 2);
}


void
ROV_App::setMagGain(ShimmerSensor *currentShimmer) {
  currentShimmer->currentStatus = waitingSettingMagGainStatus;
  quint8 command[2];
  command[0] = (quint8)Shimmer3::SET_LSM303DLHC_MAG_GAIN_COMMAND;
  command[1] = (quint8)magGain;
  currentShimmer->writeCommand(command, 2);
}


void
ROV_App::setAdcSamplingRate(ShimmerSensor *currentShimmer) {
  currentShimmer->currentStatus = waitingSettingAdcRateStatus;
  quint8 command[3];
  command[0] = (quint8)Shimmer3::SET_SAMPLING_RATE_COMMAND;
  command[1] = (quint8)(samplingRate & 0xff);
  command[2] = (quint8)(samplingRate >> 8 & 0xff);
  currentShimmer->writeCommand(command, 3);

  double rate = (double)32768 / (double)samplingRate;

  if (!currentShimmer->enableLowPowerAccel) {
    command[0] = (quint8)Shimmer3::SET_ACCEL_SAMPLING_RATE_COMMAND;
    if (rate <= 1) {
      command[1] =  (quint8)1;
    } else if (rate <= 10) {
      command[1] =  (quint8)2;
    } else if (rate <= 25) {
      command[1] =  (quint8)3;
    } else if (rate <= 50) {
      command[1] =  (quint8)4;
    } else if (rate <= 100) {
      command[1] =  (quint8)5;
    } else if (rate <= 200) {
      command[1] =  (quint8)6;
    } else {
      command[1] =  (quint8)7;
    }
  } else {
    command[1] =  (quint8)2;
  }
  currentShimmer->writeCommand(command, 2);

  if (!currentShimmer->enableLowPowerGyro) {
    command[0] = (quint8)Shimmer3::SET_MPU9150_SAMPLING_RATE_COMMAND;
    if (rate <= 51.28) {
      command[1] =  (quint8)0x9B;
    } else if (rate <= 102.56) {
      command[1] =  (quint8)0x4D;
    } else if (rate <= 129.03) {
      command[1] =  (quint8)0x3D;
    } else if (rate <= 173.91) {
      command[1] =  (quint8)0x2D;
    } else if (rate <= 205.13) {
      command[1] =  (quint8)0x26;
    } else if (rate <= 258.06) {
      command[1] =  (quint8)0x1E;
    } else if (rate <= 533.3) {
      command[1] =  (quint8)0xE;
    } else {
      command[1] =  (quint8)6;
    }
  } else {
    command[1] =  (quint8)0xFF;
  }
  currentShimmer->writeCommand(command, 2);
}


void
ROV_App::getSamplingRate(ShimmerSensor *currentShimmer) {
    currentShimmer->currentStatus = waitingSamplingRateStatus;
    quint8 data = (quint8)Shimmer3::GET_SAMPLING_RATE_COMMAND;
    currentShimmer->writeCommand(&data, 1);
}

void
ROV_App::onSamplingRateObtained(ShimmerSensor *currentShimmer) {
  if(currentShimmer->currentStatus == waitingSamplingRateStatus) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " SamplingRateObtained "
                  << currentShimmer->myRemoteAddress.toString()
                  << " Asking ExgRegs1Status";
    qDebug() << sDebugMessage;
    quint8 data = (quint8)Shimmer3::INQUIRY_COMMAND;
    currentShimmer->writeCommand(&data, 1);
    currentShimmer->currentStatus = waitingGeneralInquireStatus;
  }
}


void
ROV_App::onGeneralInquiryObtained(ShimmerSensor *currentShimmer) {
  if(currentShimmer->currentStatus == waitingGeneralInquireStatus) {
    if ((currentShimmer->GetFirmwareVersion() >= 0.3) ||
        (currentShimmer->GetFirmwareVersion() == 0.2 &&
         currentShimmer->GetFirmwareInternal()>=8))
    {
      quint8 data[4];
      currentShimmer->chipID = 1;
      data[0] = (quint8) Shimmer3::GET_EXG_REGS_COMMAND;
      data[1] = 0;
      data[2] = 0;
      data[3] = 10;
      currentShimmer->writeCommand(data, 4);
      currentShimmer->currentStatus = waitingExgRegs1Status;
      sDebugMessage = QString();
      sInformation  << dateTime.currentDateTime().toString()
                    << " GeneralInquiryObtained "
                    << currentShimmer->myRemoteAddress.toString()
                    << " Asking ExgRegs1Status";
      qDebug() << sDebugMessage;
    }
  }
}


void
ROV_App::onExgRegs1Obtained(ShimmerSensor *currentShimmer) {
  if(currentShimmer->currentStatus == waitingExgRegs1Status) {
    quint8 data[4];
    if ((currentShimmer->GetFirmwareVersion() >= 0.3) ||
        (currentShimmer->GetFirmwareVersion() == 0.2 && currentShimmer->GetFirmwareInternal()>=8))
    {
      currentShimmer->chipID = 2;
      data[0] = (quint8) Shimmer3::GET_EXG_REGS_COMMAND;
      data[1] = 1;
      data[2] = 0;
      data[3] = 10;
      currentShimmer->writeCommand(data, 4);
      currentShimmer->currentStatus = waitingExgRegs2Status;
      sDebugMessage = QString();
      sInformation  << dateTime.currentDateTime().toString()
                    << " exgRegs1Obtained "
                    << currentShimmer->myRemoteAddress.toString()
                    << " Asking ExgRegs2Status";
      qDebug() << sDebugMessage;
    }
  }
}


void
ROV_App::onExgRegs2Obtained(ShimmerSensor *currentShimmer) {
  quint8 data = (quint8)Shimmer3::GET_ALL_CALIBRATION_COMMAND;
  currentShimmer->writeCommand(&data, 1);
  currentShimmer->currentStatus = waitingCalibrationStatus;
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " exgRegs2Obtained "
                << currentShimmer->myRemoteAddress.toString()
                << " Asking All Calibrations";
  qDebug() << sDebugMessage;
}


void
ROV_App::onCalibrationsObtained(ShimmerSensor *currentShimmer) {
  sDebugMessage = QString();
  sInformation  << dateTime.currentDateTime().toString()
                << " "
                << currentShimmer->myRemoteAddress.toString()
                << " All Calibrations received:";
  qDebug() << sDebugMessage;
  setupAcquisition(currentShimmer);
}


void
ROV_App::setupAcquisition(ShimmerSensor *currentShimmer) {
  try {
    if(!(currentShimmer->GetSensors() & activeSensors)) {
      sDebugMessage = QString();
      sInformation  << dateTime.currentDateTime().toString()
                    << " Wrong configuration for "
                    << currentShimmer->myRemoteAddress.toString();
      qDebug() << sDebugMessage;
      throw QString("Wrong configuration !");
    }
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " Setup Acquisition";
    qDebug() << sDebugMessage;
    if (!currentShimmer->GetIsFilled()) {
      throw QString("Failed to read configuration information from shimmer. Please ensure correct shimmer is connected");
    }
    if (currentShimmer->GetShimmerVersion() != (int)Shimmer::SHIMMER3) {
      throw QString("No shimmer3 devices found !");
    }

    currentShimmer->mFirstTimeCalTime = true;
    currentShimmer->mLastReceivedCalibratedTimeStamp = -1;
    currentShimmer->mPacketReceptionRate = 100;
    currentShimmer->enable3DOrientation = true;

    double samplingRate = 32768.0 / currentShimmer->GetAdcSamplingRate();
    currentShimmer->mOrientation = new GradDes3DOrientation(0.4, (double)1.0/samplingRate, 1, 0, 0, 0);

    currentShimmer->setGyroOnTheFlyCalibration(bEnableGyroOnTheFlyCalibration, 100, 1.2);
    currentShimmer->currentStatus = canStartStreamingStatus;
    startStreaming(currentShimmer);
  } catch (QString sError) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << sError
                  << " in setupAcquisition()";
    qDebug() << sDebugMessage;
  }
}


void
ROV_App::startStreaming(ShimmerSensor *currentShimmer) {
  quint8 data;
  data = (quint8)Shimmer3::START_STREAMING_COMMAND;
  if(currentShimmer->currentStatus == canStartStreamingStatus) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " is now Streaming";
    qDebug() << sDebugMessage;
    currentShimmer->writeCommand(&data, 1);
    currentShimmer->currentStatus = startingStreamingStatus;
    currentShimmer->startWatchDogTimer(shimmerWatchDogTime);
  }
}


void
ROV_App::onAckReceived(ShimmerSensor *currentShimmer) {
  if(currentShimmer->currentStatus == waitingSettingSensorsStatus) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " setAcceleratorRange() !";
    qDebug() << sDebugMessage;
    setAcceleratorRange(currentShimmer);
    return;
  }
  if(currentShimmer->currentStatus == waitingSettingAccRangeStatus) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " setGyroRange() !";
    qDebug() << sDebugMessage;
    setGyroRange(currentShimmer);
    return;
  }
  if(currentShimmer->currentStatus == waitingSettingGyroRangeStatus) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " setMagGain() !";
    qDebug() << sDebugMessage;
    setMagGain(currentShimmer);
    return;
  }
  if(currentShimmer->currentStatus == waitingSettingMagGainStatus) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " setAdcSamplingRate() !";
    qDebug() << sDebugMessage;
    setAdcSamplingRate(currentShimmer);
    return;
  }
  if(currentShimmer->currentStatus == waitingSettingAdcRateStatus) {
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " getSamplingRate() !";
    qDebug() << sDebugMessage;
    getSamplingRate(currentShimmer);
    getSamplingRate(currentShimmer);
    return;
  }
  if(currentShimmer->currentStatus == stoppingStreamingStatus) {
    currentShimmer->isStreaming = false;
    sDebugMessage = QString();
    sInformation  << dateTime.currentDateTime().toString()
                  << " "
                  << currentShimmer->myRemoteAddress.toString()
                  << " Shimmer stopped streaming";
    qDebug() << sDebugMessage;
    return;
  }
  if(currentShimmer->currentStatus == startingStreamingStatus) {
    currentShimmer->currentStatus = streamingStatus;
    currentShimmer->isStreaming = true;
    return;
  }
}


void
ROV_App::periodicUpdateWidgets() {
  if(pTcpServerConnection) {
    if(pTcpServerConnection->isOpen()) {
      QString message;
        message = QString("box_pos %1 %2 %3 %4 %5 %6 %7 %8#").
          arg(0).
          arg(pShimmerSensor->shimmerBox.x).
          arg(pShimmerSensor->shimmerBox.y).
          arg(pShimmerSensor->shimmerBox.z).
          arg(pShimmerSensor->shimmerBox.pos[0]).
          arg(pShimmerSensor->shimmerBox.pos[1]).
          arg(pShimmerSensor->shimmerBox.pos[2]).
          arg(pShimmerSensor->shimmerBox.angle);
        pTcpServerConnection->write(message.toLatin1());
    }
  }
  updateTimer.start(updateTime);
}


void
ROV_App::onShimmerWatchDogTimeout(ShimmerSensor *currentShimmer) {
  sDebugMessage = QString();
  sInformation << dateTime.currentDateTime().toString()
               << " "
               << currentShimmer->myRemoteAddress.toString()
               << " Receiving Timeout";
  qDebug() << sDebugMessage;
  if(pShimmerSensor) {
    if(pShimmerSensor->currentStatus != unconnectedStatus) {
        sDebugMessage = QString();
        sInformation << dateTime.currentDateTime().toString()
                     << " "
                     << currentShimmer->myRemoteAddress.toString()
                     << " Stop Streaming";
        qDebug() << sDebugMessage;
        quint8 data;
        data = (quint8)Shimmer3::STOP_STREAMING_COMMAND;
        pShimmerSensor->currentStatus = stoppingStreamingStatus;
        pShimmerSensor->writeCommand(&data, 1);
        pShimmerSensor->stopWatchDogTimer();
    }
  }
}


void
ROV_App::onConnectionWatchDogTimeout() {
  SetSpeed(0, 0);
  SetAirValveOut(AIR_VALVE_OFF);
  SetAirValveIn(AIR_VALVE_ON);
  sDebugMessage = QString();
  sInformation << dateTime.currentDateTime().toString()
               << " Connection Timeout";
  qDebug() << sDebugMessage;
  if(pTcpServerConnection) {
      pTcpServerConnection->close();
  }
}


int
ROV_App::usbReset(QString sDevice) {
    int fd = open(sDevice.toLatin1(), O_WRONLY);
    if(fd < 0) {
        sDebugMessage = QString();
        sInformation << dateTime.currentDateTime().toString()
                     << " Error opening USB device file";
        qDebug() << sDebugMessage;
        return -1;
    }
    int rc = ioctl(fd, USBDEVFS_RESET, 0);
    if(rc < 0) {
      QString sError;
      if(errno == EBADF)  sError = " fd is not a valid file descriptor.";
      if(errno == EFAULT) sError = " argp references an inaccessible memory area.";
      if(errno == EINVAL) sError = " request or argp is not valid.";
      if(errno == ENOTTY) sError = " fd is not associated with a character special device.";
      if(errno == ENOTTY) sError+= " The specified request does not apply to the kind of object that the file descriptor fd references.";
      sDebugMessage = QString();
      sInformation << dateTime.currentDateTime().toString()
                   << " ioctl error in resetting USB device file "
                   << sDevice
                   << sError;
      qDebug() << sDebugMessage;
      return -1;
    }
    close(fd);
    sDebugMessage = QString();
    sInformation << dateTime.currentDateTime().toString()
                 << " Reset of USB device file "
                 << sDevice
                 << " done !";
    qDebug() << sDebugMessage;
    return 0;
}
