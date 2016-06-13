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

typedef signed char        int8;
typedef unsigned char      uInt8;
typedef signed short       int16;
typedef unsigned short     uInt16;
typedef signed long        int32;
typedef unsigned long      uInt32;
typedef float              float32;
typedef double             float64;
#if defined(__linux__) || defined(__APPLE__)
typedef long long int      int64;
#else
typedef __int64            int64;
#endif
#if defined(__linux__) || defined(__APPLE__)
typedef unsigned long long uInt64;
#else
typedef unsigned __int64   uInt64;
#endif
typedef uInt32               bool32;


class
ROV_App : public QCoreApplication
{
  Q_OBJECT

public:
  ROV_App(int argc, char *argv[]);
  ~ROV_App();

  int  init();

public:
  static const int UpDownAxis      = 1;
  static const int xAxisController = 2;
  static const int yAxisController = 3;
  static const int RollAxis        = 4;

  static const int DeflateButton = 109;
  static const int InflateButton = 111;

public slots:
  void destroy();
  void stopStreaming();
  void tcpError(QAbstractSocket::SocketError error);
  void tcpClientDisconnected();
  void readFromServer();
  void periodicUpdateWidgets();
  void onWatchDogTimerTimeout(ShimmerSensor *currentShimmer);
  void forwardDebugMessage(QString sDebugMessage);

private slots:
  void switchOff();
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
  int  SetUpDown(int newUpDown);
  int  SwitchMotorDxOff();
  int  SwitchMotorDxForward();
  int  SwitchMotorDxReverse();
  int  SwitchMotorSnOff();
  int  SwitchMotorSnForward();
  int  SwitchMotorSnReverse();
  int  SetMotorDxSpeed(int newSpeed);
  int  SetMotorSnSpeed(int newSpeed);
  int  setSpeedAndSteering(int iX, int iY);
  int  SetAirValveIn(int iValue);
  int  SetAirValveOut(int iValue);

  int  openTcpSession();
  int  connectToArduino();
  void executeCommand(int iTarget, int iValue);

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
  qint32       watchDogTime;

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

  int32         error;
  QString       sString;
  QString       sCommand;
  QString       sDebugMessage;
  char          ACK;
  QByteArray    message;

  float64       minMotorDx;
  float64       maxMotorDx;
  float64       minMotorSn;
  float64       maxMotorSn;

  uInt8         MOTOR_OFF;
  uInt8         MOTOR_ON;
  uInt8         MOTOR_FORWARD;
  uInt8         MOTOR_REVERSE;

  uInt8         AIR_VALVE_OFF;
  uInt8         AIR_VALVE_ON;

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
    UpDownServo   = 79
  };
  commands command;

  // Status Variables
  motorStatus   MotorDxStatus;
  motorStatus   MotorSnStatus;
  int           iCurrentUpDown;
  int           iLastControllerX;
  int           iLastControllerY;
  int           iLastUpDown;

  QByteArray    requestData;

  QTimer        updateTimer;
  qint32        updateTime;
  QTimer        iWantToCloseTimer;
  qint32        niWantToCloseAttempt;
};

#endif // ROV_APP_H
