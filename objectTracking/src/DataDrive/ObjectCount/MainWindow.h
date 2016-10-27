#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qyzbxlib.h>
#include <DataDrive.h>
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
    void on_pushButton_clicked();

    void on_comboBox_dataset_currentTextChanged(const QString &arg1);

private:
    Ui::MainWindow *ui;
    std::unique_ptr<DataDrivePipeLine> tracker;
    void loadIni(QString filepath);
    bool globalInited;
};

#endif // MAINWINDOW_H
