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
    pipeline.push_back(std::make_shared<DataDrive::OverLapAssignment>(DataDrive::OverLapAssignment(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::HungarianAssignment>(DataDrive::HungarianAssignment(mainData)));

    pipeline.push_back(std::make_shared<DataDrive::ShowAssignment>(DataDrive::ShowAssignment(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::SplitAndMerge>(DataDrive::SplitAndMerge(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::FilterDeleteObjectToDump>(DataDrive::FilterDeleteObjectToDump(mainData)));

    bool gameover=false;
    while(!gameover){
        for(int i=0;i<pipeline.size();i++){
            bool flag=pipeline[i]->run();

            if(i==0&&flag==false) gameover=true;

            if(!flag) break;
        }
    }
}
