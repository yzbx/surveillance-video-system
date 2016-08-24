#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <QMainWindow>
#include <QtCore>
#include "../yzbxLib/qyzbxlib.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "../yzbxLib/HungarianBasedTracking.h"

namespace Ui {
class Benchmark;
}

class Benchmark : public QMainWindow
{
    Q_OBJECT

public:
    explicit Benchmark(QWidget *parent = 0);
    void process(QString configFile,QString videoFile);
    ~Benchmark();

    void loadConfig(QString configFilePath);
    void processOne(QString videoFile);
private slots:
    void on_pushButton_analyse_clicked();

    void on_pushButton_stop_clicked();

private:
    Ui::Benchmark *ui;
    boost::property_tree::ptree globalPt;
    QStringList globalVideosList;
    QString globalVideoHome;
    FrameInput frameInput;
    BlobDetector blobDetector;
    HungarianBasedTracking tracker;
    bool globalStop=false;
    QString globalVideoFile;
};

#endif // BENCHMARK_H
