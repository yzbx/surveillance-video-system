#ifndef TRACKINGRESULTREPLAY_H
#define TRACKINGRESULTREPLAY_H
#include <QtCore>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "frameinput.h"
using namespace std;
class TrackingResultReplay
{
public:
    class object{
    public:
        object(int a,cv::Rect r,int id){
            frameNum=a;
            trace.push_back(r);
            ID=id;
        }
        void add(int a,cv::Rect r){
            frameNum=a;
            trace.push_back(r);
        }

        int frameNum;
        std::vector<cv::Rect> trace;
        int ID;
    };
public:
    TrackingResultReplay(QString datasetName="");
    void process(QString videoFilePath,QString recordFilePath);
    QStringList datasetNameList;
    QString datasetName;
    bool globalInit;
    void Init(QString datasetName);

    const string winname="replay";
    int MaxSkipFrame=20;
    void replay(cv::Mat &img_input, std::vector<object> &objects, int readToFrameNum);
public slots:
    void removeOldObjects(vector<object> &objects, int readToFrameNum);
};

#endif // TRACKINGRESULTREPLAY_H
