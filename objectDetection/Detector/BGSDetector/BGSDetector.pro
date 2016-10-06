QT += core
QT += gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14

TARGET = BGSDetector
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    BGSDetector.cpp

HEADERS += \
    BGSDetector.h

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary
