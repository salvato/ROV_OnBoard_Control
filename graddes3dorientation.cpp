#include <QtCore/qmath.h>
#include "graddes3dorientation.h"


GradDes3DOrientation::GradDes3DOrientation(double beta, double samplingPeriod,
                                           double q1, double q2, double q3, double q4)
{
  mBeta = beta;
  quat = QQuaternion(q1, q2, q3, q4);
  mSamplingPeriod = samplingPeriod;
}


GradDes3DOrientation::GradDes3DOrientation()
  : mBeta(1)
  , mSamplingPeriod(1)
{
  quat = QQuaternion();
}


//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

QQuaternion&
GradDes3DOrientation::MadgwickAHRSupdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz) {
  double recipNorm;
  double s1, s2, s3, s4;
  double qDot1, qDot2, qDot3, qDot4;
  double hx, hy;
  double _2q1mx, _2q1my, _2q1mz, _2q2mx, _2bx, _2bz, _4bx, _4bz, _2q1, _2q2, _2q3, _2q4, _2q1q3, _2q3q4, q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4;

  q1 = quat.scalar();
  q2 = quat.x();
  q3 = quat.y();
  q4 = quat.z();

  // Use IMU algorithm if magnetometer measurement is invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    return MadgwickAHRSupdateIMU(ax, ay, az, gx, gy, gz);
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
  qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy);
  qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx);
  qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / qSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = 1.0 / qSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q1mx = 2.0 * q1 * mx;
    _2q1my = 2.0 * q1 * my;
    _2q1mz = 2.0 * q1 * mz;
    _2q2mx = 2.0 * q2 * mx;
    _2q1 = 2.0 * q1;
    _2q2 = 2.0 * q2;
    _2q3 = 2.0 * q3;
    _2q4 = 2.0 * q4;
    _2q1q3 = 2.0 * q1 * q3;
    _2q3q4 = 2.0 * q3 * q4;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q1q4 = q1 * q4;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q2q4 = q2 * q4;
    q3q3 = q3 * q3;
    q3q4 = q3 * q4;
    q4q4 = q4 * q4;

    // Reference direction of Earth's magnetic field
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    recipNorm = 1.0 / qSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    s4 *= recipNorm;

    // Apply feedback step
    qDot1 -= mBeta * s1;
    qDot2 -= mBeta * s2;
    qDot3 -= mBeta * s3;
    qDot4 -= mBeta * s4;
  }

  // Integrate rate of change of quaternion to yield quaternion
  quat += QQuaternion(qDot1*mSamplingPeriod,
                      qDot2*mSamplingPeriod,
                      qDot3*mSamplingPeriod,
                      qDot4*mSamplingPeriod);

  quat.normalize();
  return quat;
}


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

QQuaternion&
GradDes3DOrientation::MadgwickAHRSupdateIMU(double ax, double ay, double az, double gx, double gy, double gz) {
  double recipNorm;
  double s1, s2, s3, s4;
  double qDot1, qDot2, qDot3, qDot4;
  double _2q1, _2q2, _2q3, _2q4, _4q1, _4q2, _4q3 ,_8q2, _8q3, q1q1, q2q2, q3q3, q4q4;

  q1 = quat.scalar();
  q2 = quat.x();
  q3 = quat.y();
  q4 = quat.z();

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
  qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy);
  qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx);
  qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / qSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q4 = 2.0f * q4;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _4q3 = 4.0f * q3;
    _8q2 = 8.0f * q2;
    _8q3 = 8.0f * q3;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
    q4q4 = q4 * q4;

    // Gradient descent algorithm corrective step
    s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
    s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
    s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
    recipNorm = 1.0 / qSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    s4 *= recipNorm;

    // Apply feedback step
    qDot1 -= mBeta * s1;
    qDot2 -= mBeta * s2;
    qDot3 -= mBeta * s3;
    qDot4 -= mBeta * s4;
  }

  // Integrate rate of change of quaternion to yield quaternion
  quat += QQuaternion(qDot1*mSamplingPeriod,
                      qDot2*mSamplingPeriod,
                      qDot3*mSamplingPeriod,
                      qDot4*mSamplingPeriod);

  quat.normalize();
  return quat;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
