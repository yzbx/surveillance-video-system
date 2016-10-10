#include "PipeLineFactory.h"

PipeLineFactory::PipeLineFactory(QString configFile):DataDrivePipeLine(configFile)
{

}

void PipeLineFactory::run()
{
    loadPipeLineConfig();
    pipeline.push_back(std::make_shared<DataDrive::FilterBadTrack>(DataDrive::FilterBadTrack(mainData)));

    bool gameover=false;
    while(!gameover){
        for(uint i=0;i<pipeline.size();i++){

            bool flag;
            cout<<"i="<<i<<": "<<pipeline[i]->getClassName()<<endl;
            MEASURE_TIME(flag=pipeline[i]->run());

            if(i==0&&flag==false) gameover=true;

            if(!flag) break;
        }
    }
}

void PipeLineFactory::runAll()
{

}

std::shared_ptr<DataDrive::Base> PipeLineFactory::stringToClass(string classname)
{
    if(classname.compare("Input")==0){
        return std::make_shared<DataDrive::Input>(DataDrive::Input(mainData));
    }
    else if(classname.compare("Bgs")==0){
        return std::make_shared<DataDrive::Bgs>(DataDrive::Bgs(mainData));
    }
    else if(classname.compare("BlobFeature")==0){
        return std::make_shared<DataDrive::BlobFeature>(DataDrive::BlobFeature(mainData));
    }
    else if(classname.compare("Tracker")==0){
        return std::make_shared<DataDrive::Tracker>(DataDrive::Tracker(mainData));
    }
    else if(classname.compare("KLTTracker")==0){
        return std::make_shared<DataDrive::KLTTracker>(DataDrive::KLTTracker(mainData));
    }
    else if(classname.compare("MeanShiftTracker")==0){
        return std::make_shared<DataDrive::MeanShiftTracker>(DataDrive::MeanShiftTracker(mainData));
    }
    else if(classname.compare("CamShiftTracker")==0){
        return std::make_shared<DataDrive::CamShiftTracker>(DataDrive::CamShiftTracker(mainData));
    }
    else if(classname.compare("KLTAssignment")==0){
        return std::make_shared<DataDrive::KLTAssignment>(DataDrive::KLTAssignment(mainData));
    }
    else if(classname.compare("OverLapAssignment")==0){
        return std::make_shared<DataDrive::OverLapAssignment>(DataDrive::OverLapAssignment(mainData));
    }
    else if(classname.compare("RestOverLapAssignment")==0){
        return std::make_shared<DataDrive::RestOverLapAssignment>(DataDrive::RestOverLapAssignment(mainData));
    }
    else if(classname.compare("HungarianAssignment")==0){
        return std::make_shared<DataDrive::HungarianAssignment>(DataDrive::HungarianAssignment(mainData));
    }
    else if(classname.compare("ShowAssignment")==0){
        return std::make_shared<DataDrive::ShowAssignment>(DataDrive::ShowAssignment(mainData));
    }
    else if(classname.compare("SplitAndMerge")==0){
        return std::make_shared<DataDrive::SplitAndMerge>(DataDrive::SplitAndMerge(mainData));
    }
    else if(classname.compare("FilterDeleteObjectToDump")==0){
        return std::make_shared<DataDrive::FilterDeleteObjectToDump>(DataDrive::FilterDeleteObjectToDump(mainData));
    }
    else if(classname.compare("FilterBadTrack")==0){
        return std::make_shared<DataDrive::FilterBadTrack>(DataDrive::FilterBadTrack(mainData));
    }
    else if(classname.compare("SaveToVideo")==0){
        return std::make_shared<DataDrive::SaveToVideo>(DataDrive::SaveToVideo(mainData));
    }
    else if(classname.compare("Countor")==0){
        return std::make_shared<DataDrive::Countor>(DataDrive::Countor(mainData));
    }
    else {
        assert(false);
    }

}

void PipeLineFactory::loadPipeLineConfig()
{
    QString config=mainData->pipeLineConfigFile;

    QString filedata;
    QFile file;
    file.setFileName(config);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<config;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        QStringList lists=filedata.split("\n",QString::SkipEmptyParts);

        for(int i=0;i<lists.size();i++){
            QString line=lists[i];
            if(!line.startsWith("#"))
                pipeline.push_back(stringToClass(lists[i].toStdString()));
        }
        file.close();
    }
}
