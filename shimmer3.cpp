#include "shimmer3.h"




const QString Shimmer3::ApplicationName = "Shimmer3Connect";
int Shimmer3::NumSensorBytes = 2;

const QString Shimmer3::AccelRangeString[] =
{
  "±2g",
  "±4g",
  "±8g",
  "±16g"
};


const QString Shimmer3::GyroRangeString[] =
{
  "250dps",
  "500dps",
  "1000dps",
  "2000dps"
};

const QString Shimmer3::MagRangeString[] =
{
  "±1.3Ga",
  "±1.9Ga",
  "±2.5Ga",
  "±4.0Ga",
  "±4.7Ga",
  "±5.6Ga",
  "±8.1Ga"
};

const QString Shimmer3::ExgGainString[] =
{
  "1",
  "2",
  "3",
  "4",
  "6",
  "8",
  "12"
};


Shimmer3::Shimmer3() {
    //SensitivityMatrixLowNoiseAccel2gShimmer3   = { { 83, 0, 0 }, { 0, 83, 0 }, { 0, 0, 83 } };
    //AlignmentMatrixLowNoiseAccelShimmer3       = { { 0, -1, 0 }, { -1, 0, 0 }, { 0, 0, -1 } }; 	//Default Values for Accelerometer Calibration
    //OffsetVectorAccelLowNoiseShimmer3          = {  2047 ,  2047 ,  2047  };				//Default Values for Accelerometer Calibration

    //SensitivityMatrixWideRangeAccel2gShimmer3  = { { 1631, 0, 0 }, { 0, 1631, 0 }, { 0, 0, 1631 } };
    //SensitivityMatrixWideRangeAccel4gShimmer3  = { { 815, 0, 0 }, { 0, 815, 0 }, { 0, 0, 815 } };
    //SensitivityMatrixWideRangeAccel8gShimmer3  = { { 408, 0, 0 }, { 0, 408, 0 }, { 0, 0, 408 } };
    //SensitivityMatrixWideRangeAccel16gShimmer3 = { { 135, 0, 0 }, { 0, 135, 0 }, { 0, 0, 135 } };
    //AlignmentMatrixWideRangeAccelShimmer3      = { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 } }; 	//Default Values for Accelerometer Calibration
    //OffsetVectorAccelWideRangeShimmer3         = {  0 ,  0 ,  0  };				//Default Values for Accelerometer Calibration

    //double Shimmer3::SensitivityMatrixAccel2gShimmer3[3][3] = { { 76, 0, 0 }, { 0, 76, 0 }, { 0, 0, 76 } };
    //double Shimmer3::SensitivityMatrixAccel4gShimmer3[3][3] = { { 38, 0, 0 }, { 0, 38, 0 }, { 0, 0, 38 } };
    //double Shimmer3::SensitivityMatrixAccel6gShimmer3[3][3] = { { 25, 0, 0 }, { 0, 25, 0 }, { 0, 0, 25 } };

    //double Shimmer3::AlignmentMatrixGyroShimmer3[3][3]   = { { 0, -1, 0 }, { -1, 0, 0 }, { 0, 0, -1 } }; 				//Default Values for Gyroscope Calibration
    //double Shimmer3::SensitivityMatrixGyroShimmer3[3][3] = { { 131, 0, 0 }, { 0, 131, 0 }, { 0, 0, 131 } }; 		//Default Values for Gyroscope Calibration
    //double Shimmer3::OffsetVectorGyroShimmer3[3]         = {  0 ,  0 ,  0  };						//Default Values for Gyroscope Calibration

    //double Shimmer3::AlignmentMatrixMagShimmer3[3][3]   = { { 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } }; 				//Default Values for Magnetometer Calibration
    //double Shimmer3::SensitivityMatrixMagShimmer3[3][3] = { { 420, 0, 0 }, { 0, 450, 0 }, { 0, 0, 520 } }; 			//Default Values for Magnetometer Calibration
    //double Shimmer3::OffsetVectorMagShimmer3[3]         = {  180 ,  50 ,  50  };
}


Shimmer3::~Shimmer3() {

}


const QString Shimmer3::SamplingRatesString[] =
{
  "    1Hz",
  "  10.2Hz",
  "  51.2Hz",
  "102.4Hz",
  "204.8Hz",
  "   256Hz",
  "   512Hz",
  " 1024kHz"
};

QString Shimmer3::ChannelUnits[] =
{
  "m/(sec^2)",
  "m/(sec^2)",
  "m/(sec^2)",
  "mVolts",
  "m/(sec^2)",
  "m/(sec^2)",
  "m/(sec^2)",
  "local",
  "local",
  "local",
  "deg/sec",
  "deg/sec",
  "deg/sec",
  "mVolts",
  "mVolts",
  "mVolts",
  "mVolts",
  "mVolts",
  "mVolts",
  "mVolts",
  "",
  "",
  "",
  "",
  "",
  "",
  "Celsius",
  "Pa",
  "kOhms",
  "u8",
  "i24",
  "i24",
  "Nan",
  "mVolts",
  "mVolts",
  "mVolts",
  "mVolts",
  "mVolts",
  "mVolts"
};

QString Shimmer3::ChannelProperties[] =
{
  "Low Noise Accelerometer X",
  "Low Noise Accelerometer Y",
  "Low Noise Accelerometer Z",
  "VSenseBatt",
  "Wide Range Accelerometer X",
  "Wide Range Accelerometer Y",
  "Wide Range Accelerometer Z",
  "Magnetometer X",
  "Magnetometer Y",
  "Magnetometer Z",
  "Gyroscope X",
  "Gyroscope Y",
  "Gyroscope Z",
  "External ADC A7",
  "External ADC A6",
  "External ADC A15",
  "Internal ADC A1",
  "Internal ADC A12",
  "Internal ADC A13",
  "Internal ADC A14",
  "",
  "",
  "",
  "",
  "",
  "",
  "Temperature",
  "Pressure",
  "GSR",
  "EXG1 Sta",
  "EXG1 Ch1",
  "EXG1 Ch2",
  "EXG2 Sta",
  "EXG2 Ch1",
  "EXG2 Ch2",
  "EXG1 Ch1 16Bit",
  "EXG2 Ch2 16Bit",
  "EXG2 Ch1 16Bit",
  "EXG2 Ch2 16Bit"
};

QString Shimmer3::ChannelPropertiesECG[] =
{
  "Low Noise Accelerometer X",
  "Low Noise Accelerometer Y",
  "Low Noise Accelerometer Z",
  "VSenseBatt",
  "Wide Range Accelerometer X",
  "Wide Range Accelerometer Y",
  "Wide Range Accelerometer Z",
  "Magnetometer X",
  "Magnetometer Y",
  "Magnetometer Z",
  "Gyroscope X",
  "Gyroscope Y",
  "Gyroscope Z",
  "External ADC A7",
  "External ADC A6",
  "External ADC A15",
  "Internal ADC A1",
  "Internal ADC A12",
  "Internal ADC A13",
  "Internal ADC A14",
  "",
  "",
  "",
  "",
  "",
  "",
  "Temperature",
  "Pressure",
  "GSR",
  "EXG1 Sta",
  "ECG LL-RA",
  "ECG LA-RA",
  "EXG2Sta",
  "EXG2 Ch1",
  "EXG2 Vx-RL",
  "ECG LL-RA",
  "ECG LA-RA",
  "EXG2 Ch1",
  "EXG2 Vx-RL"
};

QString Shimmer3::ChannelPropertiesEMG[] =
{
  "Low Noise Accelerometer X",
  "Low Noise Accelerometer Y",
  "Low Noise Accelerometer Z",
  "VSenseBatt",
  "Wide Range Accelerometer X",
  "Wide Range Accelerometer Y",
  "Wide Range Accelerometer Z",
  "Magnetometer X",
  "Magnetometer Y",
  "Magnetometer Z",
  "Gyroscope X",
  "Gyroscope Y",
  "Gyroscope Z",
  "External ADC A7",
  "External ADC A6",
  "External ADC A15",
  "Internal ADC A1",
  "Internal ADC A12",
  "Internal ADC A13",
  "Internal ADC A14",
  "",
  "",
  "",
  "",
  "",
  "",
  "Temperature",
  "Pressure",
  "GSR",
  "EXG1 Sta",
  "EMG Ch1",
  "EMG Ch2",
  "EXG2 Sta",
  "EXG2 Ch1",
  "EXG2 Ch2",
  "EXG1 Ch1 16Bit",
  "EXG2 Ch2 16Bit",
  "EXG2 Ch1 16Bit",
  "EXG2 Ch2 16Bit"
};


bool
Shimmer3::IsAccelInRange(int iValue) {
  if(iValue == (int)RANGE_2_0G) return true;
  if(iValue == (int)RANGE_4_0G) return true;
  if(iValue == (int)RANGE_8_0G) return true;
  if(iValue == (int)RANGE_16_0G) return true;
  return false;
}


bool
Shimmer3::IsSamplingRateInRange(int iValue) {
  if(iValue == (int)Sampling1000Hz) return true;
  if(iValue == (int)Sampling500Hz) return true;
  if(iValue == (int)Sampling250Hz) return true;
  if(iValue == (int)Sampling200Hz) return true;
  if(iValue == (int)Sampling100Hz) return true;
  if(iValue == (int)Sampling50Hz) return true;
  if(iValue == (int)Sampling10Hz) return true;
  if(iValue == (int)Sampling1Hz) return true;
  return false;
}
