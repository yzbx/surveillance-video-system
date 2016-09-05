#ifndef TRACKINGRESULTREPLAY_H
#define TRACKINGRESULTREPLAY_H
#include <QtCore>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "frameinput.h"
#include <boost/lexical_cast.hpp>
#include "yzbx_config.h"
#include "../lib/bgsfactory_yzbx.h"

using namespace std;
class TrackingResultReplay
{
protected:
    class object{
    public:
        object(int a,Rect_t r,int id){
            frameNum=a;
            trace.push_back(r);
            ID=id;
        }
        void add(int a,Rect_t r){
            frameNum=a;
            trace.push_back(r);
        }

        int frameNum;
        std::vector<Rect_t> trace;
        int ID;
    };
public:
    TrackingResultReplay();
    void process(QString videoFilePath, QString recordFilePath, QString bgsType="", bool saveVideo=true);

    const string winname="replay";
    int MaxSkipFrame=20;
    void replay(const cv::Mat &img_input, std::vector<object> &objects, int readToFrameNum, bool saveVideo=true);
public slots:
    void removeOldObjects(vector<object> &objects, int readToFrameNum, std::set<int> &idSet);

private:
    int globalChannel=1;
    IBGS *ibgs=NULL;
    cv::Mat global_img_fg;
    cv::VideoWriter videoWriter;
};

#endif // TRACKINGRESULTREPLAY_H
