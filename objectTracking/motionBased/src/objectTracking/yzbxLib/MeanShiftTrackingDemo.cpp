#include "MeanShiftTrackingDemo.h"

MeanShiftTrackingDemo::MeanShiftTrackingDemo(QString configFile):DataDrivePipeLine(configFile)
{

}

void MeanShiftTrackingDemo::run(){
    pipeline.push_back(std::make_shared<DataDrive::Input>(DataDrive::Input(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::Bgs>(DataDrive::Bgs(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::BlobFeature>(DataDrive::BlobFeature(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::MeanShiftTracker>(DataDrive::MeanShiftTracker(mainData)));

    bool gameover=false;
    while(!gameover){
        for(int i=0;i<pipeline.size();i++){
            bool flag=pipeline[i]->run();

            if(i==0&&flag==false) gameover=true;

            if(!flag) break;
        }
    }
}
