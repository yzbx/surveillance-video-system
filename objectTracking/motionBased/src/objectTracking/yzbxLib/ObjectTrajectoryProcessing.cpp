#include "ObjectTrajectoryProcessing.h"

ObjectTrajectoryProcessing::ObjectTrajectoryProcessing(QString inputFile, QString outputFile)
{
    loadObjects(inputFile);
    filterObjects();
    dumpObjects(outputFile);
}

void ObjectTrajectoryProcessing::updateObjects(int id, int frameNum, Rect_t rect)
{
    bool findObj=false;
    for(auto ob=objects.begin();ob!=objects.end();ob++){
        object &obj=*ob;
        if(obj.id==id){
            obj.rects.push_back(std::make_pair(frameNum,rect));
            findObj=true;
            break;
        }
    }
    if(!findObj){
        object newObj;
        newObj.id=id;
        newObj.rects.push_back(std::make_pair(frameNum,rect));\
        objects.push_back(newObj);
    }
}

void ObjectTrajectoryProcessing::loadObjects(QString inputFile)
{
    QFile file(inputFile);
    if(!file.open(QIODevice::ReadOnly|QIODevice::Text)){
        assert(false);
    }

    QTextStream in(&file);
    while(in.atEnd()){
        QString line=in.readLine();
        QStringList numstrList=line.split(",",QString::SkipEmptyParts);
        assert(numstrList.size()==10);
        int frameNum=numstrList[0].toInt();
        int id=numstrList[1].toInt();

        track_t rect[4];
        for(int i=2;i<6;i++){
            rect[i-2]=numstrList[i].toFloat();
        }

        Rect_t r(rect[0],rect[1],rect[2],rect[3]);
        updateObjects(id,frameNum,r);
    }
}

void ObjectTrajectoryProcessing::filterObjects()
{
    //1 count for distance
    cv::Mat distMat;
    calculateDistanceMat(distMat);
    //2 find splite and merge

    //3 decide splite and merge

    //4 remove short trajectory
}

void ObjectTrajectoryProcessing::dumpObjects(QString outputFile)
{

}



void ObjectTrajectoryProcessing::calculateDistanceMat(cv::Mat &distMat){
    if(!distMat.empty()){
        distMat.release();
    }

    int m=objects.size();
    distMat=cv::Mat::zeros(cv::Size(m,m),CV_32FC1);
    int rows=0;
    for(auto it=objects.begin();it!=objects.end();it++,rows++){
        int cols=0;
        for(auto jt=objects.begin();jt!=objects.end();jt++,cols++){
            if(rows!=cols){
                distMat.at<float>(rows,cols)=calculateDistance(*it,*jt);
            }
        }
    }
}
