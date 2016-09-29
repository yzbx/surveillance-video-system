#-------------------------------------------------
#
# Project created by QtCreator 2016-09-29T21:45:59
#
#-------------------------------------------------
CONFIG +=c++14
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ObjectCount
TEMPLATE = app


SOURCES += main.cpp\
        MainWindow.cpp

HEADERS  += MainWindow.h

FORMS    += MainWindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/release/ -lyzbxLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/debug/ -lyzbxLib
else:unix: LIBS += -L$$OUT_PWD/../yzbxLib/ -lyzbxLib

INCLUDEPATH += $$PWD/../yzbxLib
DEPENDPATH += $$PWD/../yzbxLib

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary
