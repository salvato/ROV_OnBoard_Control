#ifndef SHIMMER3_H
#define SHIMMER3_H

#include <QString>

class Shimmer3
{
public:
  Shimmer3();
  ~Shimmer3();
  static const QString ApplicationName;
  static int NumSensorBytes;

  static const QString SamplingRatesString[];
  enum SamplingRates
  {
    Sampling1000Hz = 32,   //1024Hz
    Sampling500Hz  = 64,   //512Hz
    Sampling250Hz  = 128,  //256
    Sampling200Hz  = 160,  //204.8Hz
    Sampling100Hz  = 320,  //102.4Hz
    Sampling50Hz   = 640,  //51.2Hz
    Sampling10Hz   = 3200, //10.24Hz
    Sampling1Hz    = 32768 //1Hz
  };


  static const QString AccelRangeString[];
  //#digital accel_range
  enum AccelRange
  {
    RANGE_2_0G  = 0,
    RANGE_4_0G  = 1,
    RANGE_8_0G  = 2,
    RANGE_16_0G = 3
  };


  static const QString GyroRangeString[];
  // Gyro range
  enum GyroRange
  {
    RANGE_250DPS  = 0,
    RANGE_500DPS  = 1,
    RANGE_1000DPS = 2,
    RANGE_2000DPS = 3
  };

  static const QString ExgGainString[];

  static const QString MagRangeString[];
  //LSM303DLHC Mag gain
  enum MagRange
  {
    RANGE_1_3Ga = 1,//+/-1.3 Gauss
    RANGE_1_9Ga = 2,//+/-1.9 Gauss
    RANGE_2_5Ga = 3,//+/-2.5 Gauss
    RANGE_4_0Ga = 4,//+/-4.0 Gauss
    RANGE_4_7Ga = 5,//+/-4.7 Gauss
    RANGE_5_6Ga = 6,//+/-5.6 Gauss
    RANGE_8_1Ga = 7//+/-8.1 Gauss
  };

  // Low Noise Analog Accelerometer
  double SensitivityMatrixLowNoiseAccel2gShimmer3[3][3];
  double AlignmentMatrixLowNoiseAccelShimmer3[3][3];
  double OffsetVectorAccelLowNoiseShimmer3[3];

  // Wide Range Digital Accellerometer
  double SensitivityMatrixWideRangeAccel2gShimmer3[3][3];
  double SensitivityMatrixWideRangeAccel4gShimmer3[3][3];
  double SensitivityMatrixWideRangeAccel8gShimmer3[3][3];
  double SensitivityMatrixWideRangeAccel16gShimmer3[3][3];
  double AlignmentMatrixWideRangeAccelShimmer3[3][3];
  double OffsetVectorAccelWideRangeShimmer3[3];

  // Gyroscope
  double AlignmentMatrixGyroShimmer3[3][3];
  double SensitivityMatrixGyroShimmer3[3][3];
  double OffsetVectorGyroShimmer3[3];

  // Magnetic Sensor
  double AlignmentMatrixMagShimmer3[3][3];
  double SensitivityMatrixMagShimmer3[3][3];
  double OffsetVectorMagShimmer3[3];


  enum PacketType //: quint8
  {
    DATA_PACKET                                 = 0x00,
    INQUIRY_COMMAND                             = 0x01,
    INQUIRY_RESPONSE                            = 0x02,
    GET_SAMPLING_RATE_COMMAND                   = 0x03,
    SAMPLING_RATE_RESPONSE                      = 0x04,
    SET_SAMPLING_RATE_COMMAND                   = 0x05,
    TOGGLE_LED_COMMAND                          = 0x06,
    START_STREAMING_COMMAND                     = 0x07,
    SET_SENSORS_COMMAND                         = 0x08,

    SET_LSM303DLHC_ACCEL_RANGE_COMMAND          = 0x09,// The Wide Range Accelerometer
    LSM303DLHC_ACCEL_RANGE_RESPONSE             = 0x0A,
    GET_LSM303DLHC_ACCEL_RANGE_COMMAND          = 0x0B,

//    SetGyroCalibrationCommand                   = 0x0C,// Non più definito ?
//    GyroCalibrationResponse                     = 0x0D,// Non più definito ?
//    GetGyroCalibrationCommand                   = 0x0E,// SET_CONFIG_SETUP_BYTES_COMMAND (?)
//    SetMagCalibrationCommand                    = 0x0F,// CONFIG_SETUP_BYTES_RESPONSE(?)
//    MagCalibrationResponse                      = 0x10,// GET_CONFIG_SETUP_BYTES_COMMAND(?)
//    GetMagCalibrationCommand                    = 0x11,// SET_A_ACCEL_CALIBRATION_COMMAND(?)
//    SetDAccelCalibrationCommand                 = 0x12,// A_ACCEL_CALIBRATION_RESPONSE (?)
//    DAccelCalibrationResponse                   = 0x13,// GET_A_ACCEL_CALIBRATION_COMMAND(?)
//    GetDAccelCalibrationCommand                 = 0x14,// SET_MPU9150_GYRO_CALIBRATION_COMMAND(?)

    SET_CONFIG_SETUP_BYTES_COMMAND              = 0x0E,
    CONFIG_SETUP_BYTES_RESPONSE                 = 0x0F,
    GET_CONFIG_SETUP_BYTES_COMMAND              = 0x10,

    SET_A_ACCEL_CALIBRATION_COMMAND             = 0x11,// The Low Noise Accelerometer
    A_ACCEL_CALIBRATION_RESPONSE                = 0x12,
    GET_A_ACCEL_CALIBRATION_COMMAND             = 0x13,

    SET_MPU9150_GYRO_CALIBRATION_COMMAND        = 0x14,
    MPU9150_GYRO_CALIBRATION_RESPONSE           = 0x15,
    GET_MPU9150_GYRO_CALIBRATION_COMMAND        = 0x16,

    SET_LSM303DLHC_MAG_CALIBRATION_COMMAND      = 0x17,
    LSM303DLHC_MAG_CALIBRATION_RESPONSE         = 0x18,
    GET_LSM303DLHC_MAG_CALIBRATION_COMMAND      = 0x19,

    SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND    = 0x1A,// The Wide Range Accelerometer
    LSM303DLHC_ACCEL_CALIBRATION_RESPONSE       = 0x1B,
    GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND    = 0x1C,

    STOP_STREAMING_COMMAND                      = 0x20,

    SET_GSR_RANGE_COMMAND                       = 0x21,
    GSR_RANGE_RESPONSE                          = 0x22,
    GET_GSR_RANGE_COMMAND                       = 0x23,
    DEPRECATED_GET_DEVICE_VERSION_COMMAND       = 0x24,

    DEVICE_VERSION_RESPONSE                     = 0x25,
    GET_ALL_CALIBRATION_COMMAND                 = 0x2C,
    ALL_CALIBRATION_RESPONSE                    = 0x2D,
    GET_FW_VERSION_COMMAND                      = 0x2E,
    FW_VERSION_RESPONSE                         = 0x2F,
    SET_BLINK_LED                               = 0x30,

    CHARGE_STATUS_LED_RESPONSE                  = 0x31,
    GET_CHARGE_STATUS_LED_COMMAND               = 0x32,
    BUFFER_SIZE_RESPONSE                        = 0x35,
    GET_BUFFER_SIZE_COMMAND                     = 0x36,

    SET_LSM303DLHC_MAG_GAIN_COMMAND             = 0x37,
    LSM303DLHC_MAG_GAIN_RESPONSE                = 0x38,
    GET_LSM303DLHC_MAG_GAIN_COMMAND             = 0x39,
    SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND    = 0x3A,
    LSM303DLHC_MAG_SAMPLING_RATE_RESPONSE       = 0x3B,
    GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND    = 0x3C,

    UNIQUE_SERIAL_RESPONSE                      = 0x3D,
    GET_UNIQUE_SERIAL_COMMAND                   = 0x3E,

    GET_DEVICE_VERSION_COMMAND                  = 0x3F,
    SET_ACCEL_SAMPLING_RATE_COMMAND             = 0x40,

    LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE     = 0x41,
    GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND  = 0x42,
    SET_LSM303DLHC_ACCEL_LPMODE_COMMAND         = 0x43,
    LSM303DLHC_ACCEL_LPMODE_RESPONSE            = 0x44,
    GET_LSM303DLHC_ACCEL_LPMODE_COMMAND         = 0x45,
    SET_LSM303DLHC_ACCEL_HRMODE_COMMAND         = 0x46,
    LSM303DLHC_ACCEL_HRMODE_RESPONSE            = 0x47,
    GET_LSM303DLHC_ACCEL_HRMODE_COMMAND         = 0x48,

    SET_MPU9150_GYRO_RANGE_COMMAND              = 0x49,
    MPU9150_GYRO_RANGE_RESPONSE                 = 0x4A,
    GET_MPU9150_GYRO_RANGE_COMMAND              = 0x4B,
    SET_MPU9150_SAMPLING_RATE_COMMAND           = 0x4C,
    MPU9150_SAMPLING_RATE_RESPONSE              = 0x4D,
    GET_MPU9150_SAMPLING_RATE_COMMAND           = 0x4E,
    SET_MPU9150_ACCEL_RANGE_COMMAND             = 0x4F,
    MPU9150_ACCEL_RANGE_RESPONSE                = 0x50,
    GET_MPU9150_ACCEL_RANGE_COMMAND             = 0x51,

    SET_BMP180_PRES_RESOLUTION_COMMAND          = 0x52,
    BMP180_PRES_RESOLUTION_RESPONSE             = 0x53,
    GET_BMP180_PRES_RESOLUTION_COMMAND          = 0x54,
    BMP180_CALIBRATION_COEFFICIENTS_RESPONSE    = 0x58,
    GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND = 0x59,

    RESET_TO_DEFAULT_CONFIGURATION_COMMAND      = 0x5A,
    RESET_CALIBRATION_VALUE_COMMAND             = 0x5B,

    MPU9150_MAG_SENS_ADJ_VALS_RESPONSE          = 0x5C,
    GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND       = 0x5D,

    SET_INTERNAL_EXP_POWER_ENABLE_COMMAND       = 0x5E,
    INTERNAL_EXP_POWER_ENABLE_RESPONSE          = 0x5F,
    GET_INTERNAL_EXP_POWER_ENABLE_COMMAND       = 0x60,
    SET_EXG_REGS_COMMAND                        = 0x61,
    EXG_REGS_RESPONSE                           = 0x62,
    GET_EXG_REGS_COMMAND                        = 0x63,

    SET_DAUGHTER_CARD_ID_COMMAND                = 0x64,
    DAUGHTER_CARD_ID_RESPONSE                   = 0x65,
    GET_DAUGHTER_CARD_ID_COMMAND                = 0x66,
    SET_DAUGHTER_CARD_MEM_COMMAND               = 0x67,
    DAUGHTER_CARD_MEM_RESPONSE                  = 0x68,
    GET_DAUGHTER_CARD_MEM_COMMAND               = 0x69,

    START_SDBT_COMMAND                          = 0x70,
    STATUS_RESPONSE                             = 0x71,
    GET_STATUS_COMMAND                          = 0x72,

    SET_TRIAL_CONFIG_COMMAND                    = 0x73,
    TRIAL_CONFIG_RESPONSE                       = 0x74,
    GET_TRIAL_CONFIG_COMMAND                    = 0x75,
    SET_CENTER_COMMAND                          = 0x76,
    CENTER_RESPONSE                             = 0x77,
    GET_CENTER_COMMAND                          = 0x78,
    SET_SHIMMERNAME_COMMAND                     = 0x79,
    SHIMMERNAME_RESPONSE                        = 0x7a,
    GET_SHIMMERNAME_COMMAND                     = 0x7b,
    SET_EXPID_COMMAND                           = 0x7c,
    EXPID_RESPONSE                              = 0x7d,
    GET_EXPID_COMMAND                           = 0x7e,
    SET_MYID_COMMAND                            = 0x7F,
    MYID_RESPONSE                               = 0x80,
    GET_MYID_COMMAND                            = 0x81,
    SET_NSHIMMER_COMMAND                        = 0x82,
    NSHIMMER_RESPONSE                           = 0x83,
    GET_NSHIMMER_COMMAND                        = 0x84,
    SET_CONFIGTIME_COMMAND                      = 0x85,
    CONFIGTIME_RESPONSE                         = 0x86,
    GET_CONFIGTIME_COMMAND                      = 0x87,
    DIR_RESPONSE                                = 0x88,
    GET_DIR_COMMAND                             = 0x89,
    INSTREAM_CMD_RESPONSE                       = 0x8A,
    ROUTINE_COMMUNICATION                       = 0xE0,

    ACK_COMMAND_PROCESSED                       = 0xFF
  };


  enum ChannelContents
  {
    XLNAccel          = 0x00,
    YLNAccel          = 0x01,
    ZLNAccel          = 0x02,
    VBatt             = 0x03,
    XWRAccel          = 0x04,
    YWRAccel          = 0x05,
    ZWRAccel          = 0x06,
    XMag              = 0x07,
    YMag              = 0x08,
    ZMag              = 0x09,
    XGyro             = 0x0A,
    YGyro             = 0x0B,
    ZGyro             = 0x0C,
    ExternalAdc7      = 0x0D,
    ExternalAdc6      = 0x0E,
    ExternalAdc15     = 0x0F,
    InternalAdc1      = 0x10,
    InternalAdc12     = 0x11,
    InternalAdc13     = 0x12,
    InternalAdc14     = 0x13,
    AlternativeXAccel = 0x14,
    AlternativeYAccel = 0x15,
    AlternativeZAccel = 0x16,
    AlternativeXMag   = 0x17,
    AlternativeYMag   = 0x18,
    AlternativeZMag   = 0x19,
    Temperature       = 0x1A,
    Pressure          = 0x1B,
    Exg1_Status       = 0x1D,
    Exg1_CH1          = 0x1E,
    Exg1_CH2          = 0x1F,
    Exg2_Status       = 0x20,
    Exg2_CH1          = 0x21,
    Exg2_CH2          = 0x22,
    Exg1_CH1_16Bit    = 0x23,
    Exg1_CH2_16Bit    = 0x24,
    Exg2_CH1_16Bit    = 0x25,
    Exg2_CH2_16Bit    = 0x26,
    GsrRaw            = 0x1C
  };


  static QString ChannelUnits[];
  static QString ChannelProperties[];
  static QString ChannelPropertiesECG[];
  static QString ChannelPropertiesEMG[];


  enum SensorBitmap
  {
    SensorAAccel     = 0x000080,
    SensorGyro       = 0x000040,
    SensorMag        = 0x000020,
    SensorExg1       = 0x000010,
    SensorExg2       = 0x000008,
    SensorGSR        = 0x000004,
    SensorExtA7      = 0x000002,
    SensorExtA6      = 0x000001,
    SensorVBatt      = 0x002000,
    SensorDAccel     = 0x001000,
    SensorExtA15     = 0x000800,
    SensorIntA1      = 0x000400,
    SensorIntA12     = 0x000200,
    SensorIntA13     = 0x000100,
    SensorIntA14     = 0x800000,
    SensorPressure   = 0x040000,
    SensorExg1_16Bit = 0x100000,
    SensorExg2_16Bit = 0x080000
  };


  enum MaxNumChannels
  {
    MaxNumChannels = 13
  };

  enum MaxPacketSizes
  {
    DataPacketSize = 29,
    ResponsePacketSize = 85,
    MaxCommandArgSize = 21
  };

  static bool IsAccelInRange(int iValue);
  static bool IsSamplingRateInRange(int iValue);

};//class Shimmer3



#endif // SHIMMER3_H
