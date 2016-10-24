QT       += core gui

CONFIG +=c++14
TARGET = DataDriveLib
TEMPLATE = lib

QMAKE_CXXFLAGS +="-ftest-coverage -fprofile-arcs -msse -msse2 -msse3 -Wno-sign-compare -Wno-reorder -Wtype-limits"

DEFINES += DATADRIVELIB_LIBRARY

SOURCES += DataDriveFunction002.cpp \
    AssignmentVecSetMap.cpp \
    CamShiftTrackingDemo.cpp \
    DataDriveFunction001.cpp \
    DataDriveFunctions.cpp \
    DataDriveMain.cpp \
    DataDrivePipeLine.cpp \
    DataDriveTracker.cpp \
    KLTTrackingDemo.cpp \
    MeanShiftTrackingDemo.cpp \
    ObjectCount.cpp \
    PipeLineFactory.cpp \
    B2BTrackingDemo.cpp

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += SVS-plugin yzbxlib bgslibrary opencv boost

HEADERS += \
    DataDriveFunction002.h \
    AssignmentVecSetMap.h \
    CamShiftTrackingDemo.h \
    DataDriveFunction001.h \
    DataDriveFunctions.h \
    DataDriveMain.h \
    DataDrivePipeLine.h \
    DataDriveTracker.h \
    KLTTrackingDemo.h \
    MeanShiftTrackingDemo.h \
    ObjectCount.h \
    PipeLineFactory.h \
    DataDrive.h \
    B2BTrackingDemo.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
