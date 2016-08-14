#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QFileDialog>
#include "../lib/bgsfactory_yzbx.h"
#include "../extern/qt-json/json.h"
#include "../lib/trackingfactory_yzbx.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_inputPath_clicked();

    void on_pushButton_bgs_clicked();

    void on_pushButton_tracking_clicked();

    void on_pushButton_recordReplay_clicked();

private:
    Ui::MainWindow *ui;
    void loadIni(QString filepath);
    QString absoluteFilePath(QString currentPathOrFile, QString fileName);
    QStringList globalVideosList;
    QString globalVideoHome;
};

#endif // MAINWINDOW_H
