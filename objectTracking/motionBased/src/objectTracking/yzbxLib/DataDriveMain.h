#ifndef DATADRIVEMAIN_H
#define DATADRIVEMAIN_H
#include "frameinput.h"
#include "BlobDetector.h"
#include "RectFloatTracker.h"
#include <package_bgs/IBGS.h>
#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class DataDriveMain
{
public:
    DataDriveMain(QString configFile);
    FrameInput frameInput;
    QString videoFilePath;
    cv::Mat img_input,img_fg,img_background;
    int frameNum=0;

    IBGS* bgs;
    QString bgsType="SJN_MultiCueBGS";

    BlobDetector blobFeatureDetector;
    vector<trackingObjectFeature> fv;
    list<vector<trackingObjectFeature>> fvlist;
    //list<pair<img_iput,img_fg>>
    list<pair<cv::Mat,cv::Mat>> imglist;
    uint MaxListLength;

    boost::property_tree::ptree globalPt;
    QString globalVideoHome;
    QStringList globalVideoList;

    void setCurrentVideo(QString filepath){
        videoFilePath=filepath;
    }
};

#endif // DATADRIVEMAIN_H
