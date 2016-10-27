#include "DataDriveMain.h"

DataDriveMain::DataDriveMain(QString configFile):BlobToBlob("previous blob","current blob")
{
    boost::property_tree::ini_parser::read_ini(configFile.toStdString(),globalPt);
    QString Dataset=QString::fromStdString(globalPt.get<std::string>("General.Dataset"));
    QString VideoHome=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoHome"));
    QString VideoTxt=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoTxt"));
    pipeLineConfigFile=QString::fromStdString(globalPt.get<std::string>("DataDrive.PipeLineConfigFile"));


    VideoHome=yzbxlib::getAbsoluteFilePath(configFile,VideoHome);
    VideoTxt=yzbxlib::getAbsoluteFilePath(configFile,VideoTxt);
    pipeLineConfigFile=yzbxlib::getAbsoluteFilePath(configFile,pipeLineConfigFile);
    QString filedata;
    QFile file;
    file.setFileName(VideoTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<VideoTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        globalVideoList=filedata.split("\n",QString::SkipEmptyParts);
        globalVideoHome=VideoHome;
        file.close();
    }

    bgsFactory_yzbx fac;
    bgs.reset(fac.getBgsAlgorithm(bgsType));

    QString ParamConfigFile=QString::fromStdString(globalPt.get<std::string>("DataDrive.ParamConfigFile"));
    ParamConfigFile=yzbxlib::getAbsoluteFilePath(configFile,ParamConfigFile);
    param.loadConfig(ParamConfigFile.toStdString());
}

void DataDriveMain::init()
{
    this->bgs.release();
    bgsFactory_yzbx fac;
    bgs.reset(fac.getBgsAlgorithm(bgsType));
    blobFeatureDetector.params.minArea=param.MinBlobArea;

    this->blobIdxToPointSet[0].clear();
    this->blobIdxToPointSet[1].clear();
    this->blobToTrackSet.clear();
    this->deleteLaterObjectSet.clear();
    this->frameNum=0;
    this->fv.clear();
    this->fvlist.clear();
    this->imglist.clear();
    this->imgToSave.release();
    this->img_background.release();
    this->img_fg.release();
    this->img_input.release();
    this->KLTMatchMat.release();
    this->matchedBlobSet.clear();
    this->matchedTrackSet.clear();
    this->mNToOneMap.clear();
    this->mOneToNMap.clear();
    this->mOneToOneMap.clear();
    this->mOneToZeroSet.clear();
    this->mZeroToOneSet.clear();
    this->NextTrackID=0;
    this->pointIdxToBlobSet[0].clear();
    this->points[0].clear();
    this->pointIdxToBlobSet[1].clear();
    this->points[1].clear();
    this->prevBlobToTrack.clear();
    this->tracks.clear();
    this->trackToBlobSet.clear();
    this->trackToPrevBlob.clear();
    this->unmatchedBlobSet.clear();
    this->unmatchedTrackSet.clear();
    this->BlobToBlob.clear();
}
