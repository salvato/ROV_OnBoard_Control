#ifndef MATRIXUTIL_H
#define MATRIXUTIL_H

#include <QVector>
#include <QVector3D>


void CopyMatrix3x3(double* dest, double* source);
void CopyVector3(double* dest, double* source);
bool matrixinverse3x3(double data[3][3], double inverse[3][3]);
void matrixmultiplication(double* a, double* b, double* axb, int aRows, int aColumns, int bRows, int bColumns);
void matrixminus(double* a, double* b, double* result, int aRows, int aColumns, int bRows, int bColumns);

#endif // MATRIXUTIL_H
