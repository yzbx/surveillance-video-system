#-------------------------------------------------
#
# Project created by QtCreator 2016-09-29T21:45:59
#
#-------------------------------------------------
CONFIG +=c++14
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ObjectCount
TEMPLATE = app


SOURCES += main.cpp\
        MainWindow.cpp

HEADERS  += MainWindow.h

FORMS    += MainWindow.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary yzbxlib DataDriveLib SVS-plugin

INCLUDEPATH += `pkg-config --cflags yzbxlib DataDriveLib SVS-plugin`

LIBS += \
    -lboost_system \
    -lboost_program_options \
    -lboost_filesystem \
