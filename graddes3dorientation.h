#ifndef GRADDES3DORIENTATION_H
#define GRADDES3DORIENTATION_H

//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 10/09/2015 G   Salvato   Adapted as a C++ Class
//
//=====================================================================================================


#include <QQuaternion>

class GradDes3DOrientation
{
public:
  GradDes3DOrientation();
  GradDes3DOrientation(double beta, double samplingPeriod, double q1, double q2, double q3, double q4);

  QQuaternion& MadgwickAHRSupdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz);
  QQuaternion& MadgwickAHRSupdateIMU(double ax, double ay, double az, double gx, double gy, double gz);

  double mBeta;
  double mSamplingPeriod;
  QQuaternion quat;

private:
  double recipNorm;
  double s1, s2, s3, s4;
  double qDot1, qDot2, qDot3, qDot4;

  double q1, q2, q3, q4;
  double hx, hy, _2bx, _2bz;
  double _4bx, _4bz, _2q1, _2q2, _2q3, _2q4;
  double q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4, _2q1q3, _2q3q4;
};

#endif // GRADDES3DORIENTATION_H
