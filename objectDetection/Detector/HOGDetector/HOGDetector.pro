QT += core
QT += gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
QMAKE_CXXFLAGS +="-ftest-coverage -fprofile-arcs -msse -msse2 -msse3"
QMAKE_LFLAGS +="-lgcov --coverage"

TARGET = HOGDetector
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
#    dlibhogdetector.cpp \
    cvhogdetector.cpp

unix: CONFIG += link_pkgconfig
#unix: PKGCONFIG += dlib-1 opencv
unix: PKGCONFIG += opencv

#LIBS += -L/home/yzbx/linux/miniconda2/lib -lmkl_rt

HEADERS += \
#    dlibhogdetector.h \
    cvhogdetector.h
