#include "DataDriveTracker.h"

DataDriveTracker::DataDriveTracker(QString configFile):DataDrivePipeLine(configFile)
{

}

void DataDriveTracker::run()
{
    pipeline.push_back(std::make_shared<DataDrive::Input>(DataDrive::Input(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::Bgs>(DataDrive::Bgs(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::BlobFeature>(DataDrive::BlobFeature(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::KLTAssignment>(DataDrive::KLTAssignment(mainData)));
//    pipeline.push_back(std::make_shared<DataDrive::OverLapAssignment>(DataDrive::OverLapAssignment(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::HungarianAssignment>(DataDrive::HungarianAssignment(mainData)));

    pipeline.push_back(std::make_shared<DataDrive::ShowAssignment>(DataDrive::ShowAssignment(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::SplitAndMerge>(DataDrive::SplitAndMerge(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::FilterDeleteObjectToDump>(DataDrive::FilterDeleteObjectToDump(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::FilterBadTrack>(DataDrive::FilterBadTrack(mainData)));

    bool gameover=false;
    while(!gameover){
        for(uint i=0;i<pipeline.size();i++){

            bool flag;
//            cout<<"i="<<i<<": "<<pipeline[i]->getClassName()<<endl;
//            MEASURE_TIME(flag=pipeline[i]->run());
            flag=pipeline[i]->run();
            if(i==0&&flag==false) gameover=true;

            if(!flag) break;
        }
    }
}

void DataDriveTracker::runAll(){
    for(uint i=0;i<mainData->globalVideoList.size();i++){
        QString currentVideo=mainData->globalVideoList[i];
        mainData->setCurrentVideo(currentVideo);
        pipeline.clear();
        run();
    }
}
