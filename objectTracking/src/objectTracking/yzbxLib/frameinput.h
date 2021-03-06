#ifndef FRAMEINPUT_H
#define FRAMEINPUT_H
#include <QtCore>
#include <opencv2/opencv.hpp>
#include <package_bgs/IBGS.h>

class FrameInput
{
public:
    FrameInput();
    void process(QString videoFile,cv::Mat &nextFrame);

    QString videoFilePath;
    //from video
    cv::VideoCapture videoCap;

    //from pictures
    QStringList pictureList;
    QStringList pictureFormatList;
    int frameNum=-1;

    void getNextFrame(QString videoFile,cv::Mat &nextFrame);
    void initBgs(IBGS *ibgs,int initFrameNum=100);
    void init(QString videoFile);
    void setStartFrameNum(QString videoFile,int frameNum);
};

#endif // FRAMEINPUT_H
