#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <QMainWindow>
#include <QtCore>


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

private:
    Ui::Benchmark *ui;
};

#endif // BENCHMARK_H
