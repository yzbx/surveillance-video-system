#-------------------------------------------------
#
# Project created by QtCreator 2016-08-17T15:45:23
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG +=c++14
QMAKE_CXXFLAGS +="-ftest-coverage -fprofile-arcs -msse -msse2 -msse3"
QMAKE_LFLAGS +="-lgcov --coverage"

TARGET = benchmark
TEMPLATE = app


SOURCES += main.cpp\
        benchmark.cpp

HEADERS  += benchmark.h

FORMS    += benchmark.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/release/ -lyzbxLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/debug/ -lyzbxLib
else:unix: LIBS += -L$$OUT_PWD/../yzbxLib/ -lyzbxLib

INCLUDEPATH += $$PWD/../yzbxLib
DEPENDPATH += $$PWD/../yzbxLib

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/release/ -lyzbxLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/debug/ -lyzbxLib
else:unix: LIBS += -L$$OUT_PWD/../yzbxLib/ -lyzbxLib

INCLUDEPATH += $$PWD/../yzbxLib
DEPENDPATH += $$PWD/../yzbxLib
