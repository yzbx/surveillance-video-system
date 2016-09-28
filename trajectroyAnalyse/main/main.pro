#-------------------------------------------------
#
# Project created by QtCreator 2016-09-28T09:36:56
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = main
TEMPLATE = app


SOURCES += main.cpp\
        MainWindow.cpp

HEADERS  += MainWindow.h

FORMS    += MainWindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../objectTracking/motionBased/src/build-objectTracking-Desktop_Qt_5_6_0_GCC_64bit-Debug/yzbxLib/release/ -lyzbxLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../objectTracking/motionBased/src/build-objectTracking-Desktop_Qt_5_6_0_GCC_64bit-Debug/yzbxLib/debug/ -lyzbxLib
else:unix: LIBS += -L$$PWD/../../objectTracking/motionBased/src/build-objectTracking-Desktop_Qt_5_6_0_GCC_64bit-Debug/yzbxLib/ -lyzbxLib

INCLUDEPATH += $$PWD/../../objectTracking/motionBased/src/objectTracking/yzbxLib
DEPENDPATH += $$PWD/../../objectTracking/motionBased/src/objectTracking/yzbxLib

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary
