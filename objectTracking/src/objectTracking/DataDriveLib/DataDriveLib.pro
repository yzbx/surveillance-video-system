QT       += core gui

TARGET = DanamicDataDriveLib
TEMPLATE = lib

DEFINES += DANAMICDATADRIVELIB_LIBRARY

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
    PipeLineFactory.cpp

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
    PipeLineFactory.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
