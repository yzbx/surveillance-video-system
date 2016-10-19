#-------------------------------------------------
#
# Project created by QtCreator 2016-08-17T15:45:23
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG +=c++14
QMAKE_CXXFLAGS +="-ftest-coverage -fprofile-arcs -msse -msse2 -msse3 -Wall -Wextra -Wno-sign-compare"
QMAKE_LFLAGS +="-lgcov --coverage"

TARGET = benchmark
TEMPLATE = app


SOURCES += main.cpp\
        benchmark.cpp

HEADERS  += benchmark.h

FORMS    += benchmark.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary yzbxlib

INCLUDEPATH +=`pkg-config --cflags yzbxlib`
LIBS += \
    -lboost_system \
    -lboost_program_options \
    -lboost_filesystem \
