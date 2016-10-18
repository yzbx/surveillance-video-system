TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG += qt

SOURCES += main.cpp \
    DataDriveFunction002.cpp

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += SVS-plugin yzbxlib bgslibrary opencv boost

HEADERS += \
    DataDriveFunction002.h
