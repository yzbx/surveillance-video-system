#-------------------------------------------------
#
# Project created by QtCreator 2016-09-28T09:36:56
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = main
TEMPLATE = app


SOURCES += main.cpp\
        MainWindow.cpp

HEADERS  += MainWindow.h

FORMS    += MainWindow.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary
