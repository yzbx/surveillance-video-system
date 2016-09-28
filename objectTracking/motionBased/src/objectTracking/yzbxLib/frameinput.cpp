#include "frameinput.h"

FrameInput::FrameInput()
{
    pictureFormatList<<"*.jpg"<<"*.png"<<"*.jpeg";
}

void FrameInput::process(QString videoFile, cv::Mat &nextFrame)
{
    getNextFrame(videoFile,nextFrame);
}

void FrameInput::getNextFrame(QString videoFile, cv::Mat &nextFrame)
{
//    nextFrame.release();
    if(!nextFrame.empty()){
        nextFrame.release();
    }
    assert(nextFrame.empty());
    assert(!videoFile.isEmpty());
//    qDebug()<<"get Next Frame "<<this->frameNum<<"from "<<videoFile;

    if(videoFile==this->videoFilePath){
        QFileInfo info(videoFile);
        if(info.isDir()){
            if(frameNum>=pictureList.size()){
                qDebug()<<"end of picture list!!!";
                return;
            }
            QString imagestr=videoFile+"/"+pictureList.at(this->frameNum);
//            qDebug()<<"frameNum="<<this->frameNum<<" for image "<<imagestr;
            nextFrame=cv::imread(imagestr.toStdString());
        }
        else{
            videoCap>>nextFrame;
        }
        this->frameNum=this->frameNum+1;
    }
    else{
        this->videoFilePath=videoFile;
        this->frameNum=0;

        QFileInfo info(videoFile);
        if(info.isDir()){
            QDir videoDir(videoFile);
            //sort image by name, QDir::Reversed
            this->pictureList=videoDir.entryList(this->pictureFormatList,QDir::Files,QDir::Name);

            QString imagestr=videoFile+"/"+pictureList.at(this->frameNum);
//            qDebug()<<"frameNum="<<this->frameNum;
            nextFrame=cv::imread(imagestr.toStdString());
        }
        else{
            if(videoCap.isOpened()){
                videoCap.release();
            }
            if(!videoCap.open(videoFile.toStdString())){
                qDebug()<<"cannot open video file "<<videoFile;
                exit(-1);
            }
            videoCap>>nextFrame;
        }
        this->frameNum=this->frameNum+1;
    }

    if(nextFrame.empty()){
        qDebug()<<"get next frame failed !!!";
        qDebug()<<"maybe the end of videos";
//        exit(-1);
    }
}

void FrameInput::initBgs(IBGS *ibgs, int initFrameNum)
{
    qDebug()<<"init bgs...........................";
    if(this->frameNum!=0){
        qDebug()<<"init Bgs error, must init FrameInput first!!!";
        exit(-1);
    }

    cv::Mat inputFrame,img_foreground,img_background;
    for(int i=0;i<initFrameNum;i++){
        getNextFrame(this->videoFilePath,inputFrame);
        ibgs->process(inputFrame,img_foreground,img_background);
    }

    QFileInfo info(videoFilePath);
    if(info.isDir()){
        this->frameNum=0;
    }
    else{
        this->frameNum=0;
        videoCap.release();
        if(!videoCap.open(videoFilePath.toStdString())){
            qDebug()<<"cannot open video file "<<videoFilePath;
            exit(-1);
        }
    }
}

void FrameInput::init(QString videoFile)
{
    qDebug()<<"init frame input class: "<<videoFile;
    this->videoFilePath=videoFile;
    this->frameNum=0;

    QFileInfo info(videoFile);
    if(info.isDir()){
        QDir videoDir(videoFile);
        //sort image by name, QDir::Reversed
        this->pictureList=videoDir.entryList(this->pictureFormatList,QDir::Files,QDir::Name);
    }
    else{
        videoCap.release();
        if(!videoCap.open(videoFile.toStdString())){
            qDebug()<<"cannot open video file "<<videoFile;
            exit(-1);
        }
    }
}

void FrameInput::setStartFrameNum(QString videoFile, int frameNum)
{
    init(videoFile);
    QFileInfo info(videoFile);
    if(info.isDir()){
        this->frameNum=frameNum;
    }
    else{
        videoCap.set(CV_CAP_PROP_POS_FRAMES,frameNum);
        this->frameNum=frameNum;
    }
}
