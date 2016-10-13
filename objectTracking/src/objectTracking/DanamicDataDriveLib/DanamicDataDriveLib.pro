#-------------------------------------------------
#
# Project created by QtCreator 2016-10-12T10:04:48
#
#-------------------------------------------------

QT       += core gui

TARGET = DanamicDataDriveLib
TEMPLATE = lib

DEFINES += DANAMICDATADRIVELIB_LIBRARY

SOURCES += DanamicDataDriveLib.cpp

HEADERS += DanamicDataDriveLib.h\
        danamicdatadrivelib_global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
