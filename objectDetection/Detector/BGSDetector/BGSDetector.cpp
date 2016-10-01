#include "BGSDetector.h"

BGSDetector::BGSDetector(QString configFile):DataDrivePipeLine(configFile)
{

}

void BGSDetector::run()
{
    pipeline.push_back(std::make_shared<DataDrive::Input>(DataDrive::Input(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::Bgs>(DataDrive::Bgs(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::BlobFeature>(DataDrive::BlobFeature(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::SaveToVideo>(DataDrive::SaveToVideo(mainData)));

    bool gameover=false;
    while(!gameover){
        cout<<"frameNum= "<<mainData->frameNum<<endl;
        for(int i=0;i<pipeline.size();i++){

            bool flag=pipeline[i]->run();

            if(i==0&&flag==false) gameover=true;

            if(!flag) break;
        }
    }
}

void BGSDetector::runAll()
{
    for(int i=0;i<mainData->globalVideoList.size();i++){
        QString currentVideo=mainData->globalVideoList[i];
        mainData->setCurrentVideo(currentVideo);
        pipeline.clear();
        run();
    }
}
