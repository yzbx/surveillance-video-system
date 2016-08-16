QT += core
QT -= gui

CONFIG += c++11

TARGET = HOGDetector
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += dlib-1

LIBS += -L/home/yzbx/linux/miniconda2/lib -lmkl_rt
