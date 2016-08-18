#include "benchmark.h"
#include "ui_benchmark.h"

Benchmark::Benchmark(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Benchmark)
{
    ui->setupUi(this);
}

Benchmark::~Benchmark()
{
    delete ui;
}
