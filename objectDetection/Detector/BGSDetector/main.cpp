//#include <QCoreApplication>
#include <QApplication>
#include "BGSDetector.h"

int main(int argc, char *argv[])
{
//    QCoreApplication a(argc, argv);
//    QApplication a(argc, argv);
    QString videoFile="all";
    QString configFile="/home/yzbx/config/BGSDetector.ini";
    std::unique_ptr<DataDrivePipeLine> tracker=std::make_unique<BGSDetector>(BGSDetector(configFile));

    if(videoFile=="all"){
        tracker->runAll();
    }
    else{
        tracker->mainData->setCurrentVideo(videoFile);
        tracker->run();
    }
    return 0;
}
