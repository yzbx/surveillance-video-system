#ifndef TRACKINGRESULTREPLAY_H
#define TRACKINGRESULTREPLAY_H
#include <QtCore>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "frameinput.h"
#include <boost/lexical_cast.hpp>

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
    TrackingResultReplay();
    void process(QString videoFilePath,QString recordFilePath);

    const string winname="replay";
    int MaxSkipFrame=20;
    void replay(cv::Mat &img_input, std::vector<object> &objects, int readToFrameNum);
public slots:
    void removeOldObjects(vector<object> &objects, int readToFrameNum, std::set<int> &idSet);
};

#endif // TRACKINGRESULTREPLAY_H
