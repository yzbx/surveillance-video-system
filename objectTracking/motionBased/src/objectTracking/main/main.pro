#-------------------------------------------------
#
# Project created by QtCreator 2016-08-09T20:53:03
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG +=c++14
QMAKE_CXXFLAGS +="-ftest-coverage -fprofile-arcs -msse -msse2 -msse3"
QMAKE_LFLAGS +="-lgcov --coverage"

TARGET = main
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \

HEADERS  += mainwindow.h \

FORMS    += mainwindow.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv bgslibrary dlib-1

LIBS += \
    -lboost_system \
    -lboost_program_options \
    -lboost_filesystem \

LIBS += -L/home/yzbx/linux/miniconda2/lib -lmkl_rt


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/release/ -lyzbxLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../yzbxLib/debug/ -lyzbxLib
else:unix: LIBS += -L$$OUT_PWD/../yzbxLib/ -lyzbxLib

INCLUDEPATH += $$PWD/../yzbxLib
DEPENDPATH += $$PWD/../yzbxLib
