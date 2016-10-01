QT += core
QT += gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14

TARGET = BGSDetector
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    BGSDetector.cpp

LIBS += -L$$PWD/../../../objectTracking/motionBased/src/build-objectTracking-Desktop_Qt_5_6_0_GCC_64bit-Debug/yzbxLib/ -lyzbxLib

message($$LIBS)

INCLUDEPATH += $$PWD/../../../objectTracking/motionBased/src/objectTracking/yzbxLib \
$$PWD/../../../objectTracking/motionBased/src/objectTracking/lib

message($$INCLUDEPATH)

DEPENDPATH += $$PWD/../../../objectTracking/motionBased/src/build-objectTracking-Desktop_Qt_5_6_0_GCC_64bit-Debug/yzbxLib

HEADERS += \
    BGSDetector.h

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary
