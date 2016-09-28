#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qyzbxlib.h"
#include <unordered_map>

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

private:
    Ui::MainWindow *ui;
    std::unordered_map<int,ObjectRecord> objectRecordMap;
    void loadconfig(QString configFile);
    QStringList globalVideoList;
    QStringList globalCSVList;
    QString globalVideoHome;
    QString globalCSVHome;
    QString globalImageDatabaseDir;
};

#endif // MAINWINDOW_H
