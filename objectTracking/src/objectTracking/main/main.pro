#-------------------------------------------------
#
# Project created by QtCreator 2016-08-09T20:53:03
#
#-------------------------------------------------

QT       += core gui svg

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG +=c++14
QMAKE_CXXFLAGS +="-ftest-coverage -fprofile-arcs -msse -msse2 -msse3 -Wall -Wextra -Wno-sign-compare"
QMAKE_LFLAGS +="-lgcov --coverage"

TARGET = main
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \

HEADERS  += mainwindow.h \

FORMS    += mainwindow.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary yzbxlib DataDriveLib SVS-plugin

INCLUDEPATH += `pkg-config --cflags yzbxlib DataDriveLib SVS-plugin`

LIBS += \
    -lboost_system \
    -lboost_program_options \
    -lboost_filesystem \

#unix: PKGCONFIG += dlib-1
#LIBS += -L/home/yzbx/linux/miniconda2/lib -lmkl_rt

DISTFILES += \
    ConvertOutputFormat.py
