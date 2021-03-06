#ifndef ROV_APP_H
#define ROV_APP_H

#include <QCoreApplication>

#include <QDateTime>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtSerialPort/QSerialPort>
#include <QTimer>
#include <QCloseEvent>

#include "shimmerSensor.h"
#include "shimmer.h"
#include "shimmer3.h"
#include "shimmerdatapacket.h"

QT_FORWARD_DECLARE_CLASS(Shimmer3Box)
QT_FORWARD_DECLARE_CLASS(QBluetoothLocalDevice)


class
ROV_App : public QCoreApplication
{
  Q_OBJECT

public:
  ROV_App(int argc, char *argv[]);
  ~ROV_App();

  int  init();

public:
  static const int upDownAxis      =   0;
  static const int pitchAxis       =   1;
  static const int xAxisController =   2;
  static const int yAxisController =   3;

  static const int depthSensor     =  81;

  static const int DeflateButton   = 109;
  static const int InflateButton   = 111;
  static const int SetOrientation  = 125;
  static const int StillAlive      = 126;

public slots:
  void destroy();
  void stopStreaming();
  void tcpError(QAbstractSocket::SocketError error);
  void tcpClientDisconnected();
  void readFromServer();
  void periodicUpdateWidgets();
  void onShimmerWatchDogTimeout(ShimmerSensor *currentShimmer);
  void forwardDebugMessage(QString sDebugMessage);
  void onConnectionWatchDogTimeout();
  void switchOff();

private slots:
  void iWantToCloseTimerTimeout();
  void newTcpConnection();
  void onNewShimmerConnected(ShimmerSensor* currentShimmer);
  void onShimmerFailedToConnect(ShimmerSensor* currentShimmer);
  void onShimmerDisconnected(ShimmerSensor* currentShimmer);
  void onShimmerStatusReceived(ShimmerSensor* currentShimmer, quint8 shimmerStatus);
  void onAckReceived(ShimmerSensor *currentShimmer);
  void onFirmwareRead(ShimmerSensor *currentShimmer);
  void onShimmerVersionRead(ShimmerSensor *currentShimmer);
  void onSamplingRateObtained(ShimmerSensor *currentShimmer);
  void onGeneralInquiryObtained(ShimmerSensor *currentShimmer);
  void onExgRegs1Obtained(ShimmerSensor *currentShimmer);
  void onExgRegs2Obtained(ShimmerSensor *currentShimmer);
  void onCalibrationsObtained(ShimmerSensor *currentShimmer);
  void startStreaming(ShimmerSensor *currentShimmer);

signals:
  void iWantToClose();
  void destroyMe();

private:
  void closeEvent(QCloseEvent *event);
  void ErrorHandler(QString sErrorString);
  int  writeRequest(QByteArray requestData);
  int  SetSpeed(int iX, int iY);
  int  SetThrusterSpeed(int iFrontSpeed, int iRearSpeed);
  int  SetFrontThrusterSpeed(int newSpeed);
  int  SetRearThrusterSpeed(int newSpeed);
  int  SwitchMotorDxOff();
  int  SwitchMotorDxForward();
  int  SwitchMotorDxReverse();
  int  SwitchMotorSnOff();
  int  SwitchMotorSnForward();
  int  SwitchMotorSnReverse();
  int  SetMotorDxSpeed(int newSpeed);
  int  SetMotorSnSpeed(int newSpeed);
  int  SetAirValveIn(int iValue);
  int  SetAirValveOut(int iValue);
  int  GetRovDepth();
  void sendDepth(int depth);

  int  openTcpSession();
  int  connectToArduino();
  void executeCommand(int iTarget, int iValue);
  int  usbReset(QString sDevice);

  bool CheckBluetoothSupport();
  void initShimmer(ShimmerSensor* currentShimmer);
  void AskFirmwareVersion(ShimmerSensor *currentShimmer);
  void setActiveSensors(ShimmerSensor* currentShimmer);
  void setAdcSamplingRate(ShimmerSensor* currentShimmer);
  void setAcceleratorRange(ShimmerSensor *currentShimmer);
  void setGyroRange(ShimmerSensor *currentShimmer);
  void setMagGain(ShimmerSensor *currentShimmer);
  void getSamplingRate(ShimmerSensor *currentShimmer);
  void setupAcquisition(ShimmerSensor *currentShimmer);

private:
  int           xControl, yControl;
  QDateTime     dateTime;
  QTextStream   sInformation;
  QTcpServer*   pTcpServer;
  QTcpSocket*   pTcpServerConnection;
  int           serverPort;

  QSerialPort   serialPort;
  int           baudRate;
  int           waitTimeout;

  bool         bUseBluetooth;
  bool         bUseLowNoiseAccelerator;
  bool         bEnableGyroOnTheFlyCalibration;
  qint32       shimmerWatchDogTime;

  // For Sensor Configuration
  qint32       activeSensors;
  qint16       samplingRate;
  quint8       acceleratorRange;
  quint8       gyroRange;
  quint8       magGain;

  QBluetoothLocalDevice* adapter;
  QBluetoothAddress      shimmerBtAdress;
  QString                shimmerComPort;
  ShimmerSensor*         pShimmerSensor;

  qint32         error;
  QString       sString;
  QString       sCommand;
  QString       sDebugMessage;
  char          ACK;
  QByteArray    message;

  double       minMotorDx;
  double       maxMotorDx;
  double       minMotorSn;
  double       maxMotorSn;

  quint8         MOTOR_OFF;
  quint8         MOTOR_ON;
  quint8         MOTOR_FORWARD;
  quint8         MOTOR_REVERSE;

  quint8         AIR_VALVE_OFF;
  quint8         AIR_VALVE_ON;

  enum motorStatus {MotorOff, MotorForward, MotorReverse};

  enum commands {
    AreYouThere   = 70,
    InflateValve  = 71,
    DeflateValve  = 72,
    RightForward  = 73,
    RightReverse  = 74,
    LeftForward   = 75,
    LeftReverse   = 76,
    RightSpeed    = 77,
    LeftSpeed     = 78,
    FrontThruster = 79,
    BackThruster  = 80,
    GetDepth      = 81
  };
  commands command;

  // Status Variables
  motorStatus   MotorDxStatus;
  motorStatus   MotorSnStatus;
  int           iLastSpeedX;
  int           iLastSpeedY;
  int           iLastSpeedFront;
  int           iLastSpeedRear;

  QByteArray    requestData;

  QTimer        updateTimer;
  qint32        updateTime;
  QTimer        iWantToCloseTimer;
  qint32        niWantToCloseAttempt;
  QTimer        connectionWatchDogTimer;
  qint32        connectionWatchDogTime;

  QString       sUsbDeviceFile;
  volatile bool waitingDepth;
};

#endif // ROV_APP_H
