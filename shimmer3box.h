#ifndef SHIMMER3BOX_H
#define SHIMMER3BOX_H


class
Shimmer3Box
{
public:
  explicit Shimmer3Box();
  ~Shimmer3Box();
  void setAxisAngle(double a, double x, double y, double z);
  void setPos(double x, double y, double z);
  double angle;
  double w, h, d;
  double x, y, z;// Center coordinates
  double pos[3];
};

#endif // SHIMMER3BOX_H
