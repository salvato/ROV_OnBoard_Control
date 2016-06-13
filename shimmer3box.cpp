
#include "shimmer3box.h"

Shimmer3Box::Shimmer3Box()
{
  w = 0.034*0.5;// along X
  h = 0.051*0.5;// along Y
  d = 0.014*0.5;// along Z
  pos[0] = 0.0;
  pos[1] = 0.0;
  pos[2] = 0.0;
  angle  = 0.0;
  x      = 0.0;
  y      = 0.0;
  z      = 0.0;
}


Shimmer3Box::~Shimmer3Box() {
}


void
Shimmer3Box::setPos(double x, double y, double z) {
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
}


void
Shimmer3Box::setAxisAngle(double a, double x, double y, double z) {
  this->angle = a;
  this->x = x;
  this->y = y;
  this->z = z;
}
