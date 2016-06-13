#include <QDebug>
#include "matrixUtil.h"


void
CopyMatrix3x3(double* dest, double* source) {
  for(int i=0; i<9; i++)
    dest[i] = source[i];
}


void
CopyVector3(double* dest, double* source) {
  for(int i=0; i<3; i++)
    dest[i] = source[i];
}


bool
matrixinverse3x3(double data[3][3], double inverse[3][3]) {
  double a,b,c,d,e,f,g,h,i;
  a=data[0][0];
  b=data[0][1];
  c=data[0][2];
  d=data[1][0];
  e=data[1][1];
  f=data[1][2];
  g=data[2][0];
  h=data[2][1];
  i=data[2][2];
  //
  double deter=a*e*i+b*f*g+c*d*h-c*e*g-b*d*i-a*f*h;
  if(deter == 0.0) {
    qDebug() << "Unable to compute matrix inverse !";
    return false;
  }
  inverse[0][0]=(1/deter)*(e*i-f*h);
  inverse[0][1]=(1/deter)*(c*h-b*i);
  inverse[0][2]=(1/deter)*(b*f-c*e);
  inverse[1][0]=(1/deter)*(f*g-d*i);
  inverse[1][1]=(1/deter)*(a*i-c*g);
  inverse[1][2]=(1/deter)*(c*d-a*f);
  inverse[2][0]=(1/deter)*(d*h-e*g);
  inverse[2][1]=(1/deter)*(g*b-a*h);
  inverse[2][2]=(1/deter)*(a*e-b*d);
  return true;
}


void
matrixmultiplication(double* a, double* b, double* axb, int aRows, int aColumns, int bRows, int bColumns) {
  Q_UNUSED(bRows)
  for(int i=0; i<aRows*bColumns; i++) axb[i]= 0.0;
  for (int i=0; i<aRows; i++) { // aRow
    for (int j=0; j<bColumns; j++) { // bColumn
      for (int k = 0; k < aColumns; k++) { // aColumn
        axb[i*bColumns+j] += a[i*aColumns+k] * b[k*bColumns+j];
      }
    }
  }
}


void
matrixminus(double* a, double* b, double* result, int aRows, int aColumns, int bRows, int bColumns) {
  Q_UNUSED(bRows)
  Q_UNUSED(bColumns)
  for(int i=0; i<aRows; i++) { // aRow
    for(int k=0; k<aColumns; k++) { // aColumn
      result[i*aColumns+k]=a[i*aColumns+k]-b[i*aColumns+k];
    }
  }
}

