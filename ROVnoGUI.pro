QT += core
QT += network
QT += bluetooth
QT += serialport
QT += gui # needed for QQuaternion

CONFIG += c++11

TARGET = ROVnoGUI
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    ROV_app.cpp \
    shimmerSensor.cpp \
    shimmer.cpp \
    shimmer3.cpp \
    shimmer3box.cpp \
    shimmerdatapacket.cpp \
    graddes3dorientation.cpp \
    matrixUtil.cpp

HEADERS += \
    ROV_app.h \
    shimmerSensor.h \
    shimmer.h \
    shimmer3.h \
    shimmer3box.h \
    shimmerdatapacket.h \
    graddes3dorientation.h \
    matrixUtil.h
