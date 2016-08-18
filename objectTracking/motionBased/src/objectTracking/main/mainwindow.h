#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QFileDialog>
#include "../yzbxLib/qyzbxlib.h"
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

    void on_comboBox_dataset_currentIndexChanged(const QString &arg1);

    void on_comboBox_replay_currentIndexChanged(const QString &arg1);

private:
    Ui::MainWindow *ui;
    void loadIni(QString filepath);
    QString absoluteFilePath(QString currentPathOrFile, QString fileName);
    QStringList globalVideosList;
    QString globalVideoHome;
    QStringList globalDatasetList;
    QString globalAnnotationHome;
    QStringList globalAnnotationList;
    bool globalInited=false;
    bool globalDatasetChanged=false;
};

#endif // MAINWINDOW_H
