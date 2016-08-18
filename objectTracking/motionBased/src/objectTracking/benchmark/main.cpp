#include "benchmark.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Benchmark w;
    w.show();

    return a.exec();
}
