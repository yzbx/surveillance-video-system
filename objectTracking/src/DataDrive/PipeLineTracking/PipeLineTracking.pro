#-------------------------------------------------
#
# Project created by QtCreator 2016-09-04T13:55:26
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PipeLineTracking
TEMPLATE = app


SOURCES += main.cpp\
        MainWindow.cpp

HEADERS  += MainWindow.h

FORMS    += MainWindow.ui

YZBXLIB=/home/yzbx/build/build-objectTracking-Desktop_Qt_5_6_0_GCC_64bit-Debug
YZBX_INCLUDE=/home/yzbx/git/surveillance-video-system/objectTracking/src/objectTracking
LIBS += -L$$YZBXLIB/yzbxLib/ -lyzbxLib
INCLUDEPATH += $$YZBX_INCLUDE/yzbxLib
DEPENDPATH += $$YZBX_INCLUDE/yzbxLib
