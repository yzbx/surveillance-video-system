#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qyzbxlib.h"

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
    void on_pushButton_loadTrajectory_clicked();

    void on_comboBox_video_currentIndexChanged(int index);

    void on_comboBox_trajectory_currentIndexChanged(int index);

    void on_listWidget_currentRowChanged(int currentRow);

    void on_listWidget_doubleClicked(const QModelIndex &index);

private:
    Ui::MainWindow *ui;
    std::map<int,ObjectRecord> objectRecordMap;
    void loadconfig(QString configFile);
    QStringList globalVideoList;
    QStringList globalCSVList;
    QString globalVideoHome;
    QString globalCSVHome;
    QString globalImageDatabaseDir;

    bool videoToTrajectory=false;
    bool trajectoryToVideo=false;
};

#endif // MAINWINDOW_H
