#include "B2BTrackingDemo.h"

B2BTrackingDemo::B2BTrackingDemo(QString configFile):DataDrivePipeLine(configFile)
{

}

void B2BTrackingDemo::run()
{
    pipeline.push_back(std::make_shared<DataDrive::Input>(DataDrive::Input(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::Bgs>(DataDrive::Bgs(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::BlobFeature>(DataDrive::BlobFeature(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::BlobToBlobAssignment_KLT>(DataDrive::BlobToBlobAssignment_KLT(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::BlobToBlobAssignment_Hungarian>(DataDrive::BlobToBlobAssignment_Hungarian(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::BlobToBlobAssignment_SplitMerge>(DataDrive::BlobToBlobAssignment_SplitMerge(mainData)));
    pipeline.push_back(std::make_shared<DataDrive::TrackingDump>(DataDrive::TrackingDump(mainData)));

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

        int key=waitKey(30);
        if(key=='q'||key=='Q') break;
    }
}

void B2BTrackingDemo::runAll()
{

}
