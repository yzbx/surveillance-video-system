#include <QApplication>
#include <opencv2/core/types_c.h>
#include "cvhogdetector.h"
#include "dlibhogdetector.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
//    DlibHOGDetector dhog(argc,argv);
    CVHOGDetector cvhog(argc,argv);
    return a.exec();
}

