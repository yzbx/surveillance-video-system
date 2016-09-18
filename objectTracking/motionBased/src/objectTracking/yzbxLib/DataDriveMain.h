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
    uint MaxListLength=2;

    boost::property_tree::ptree globalPt;
    QString globalVideoHome;
    QStringList globalVideoList;

    void setCurrentVideo(QString filepath){
        videoFilePath=filepath;
    }

    ///KLT Assignment
    //prevblob match current blob
    cv::Mat KLTMatchMat;
    std::map<Index_t,Index_t> prevBlobToTrack;
    std::map<Index_t,Index_t> trackToPrevBlob;
    std::map<Index_t,std::set<Index_t>> blobToTrackSet;
    std::map<Index_t,std::set<Index_t>> trackToBlobSet;
    std::set<Index_t> matchedBlobSet;
    std::set<Index_t> matchedTrackSet;

    ///Hungarian Assignment
    std::set<Index_t> unmatchedBlobSet;
    std::set<Index_t> unmatchedTrackSet;

    ///Split and Merge
    std::set<Index_t> mZeroToOneSet;	 // 0-1
    std::set<Index_t> mOneToZeroSet; // 1-0
    std::map<Index_t, Index_t> mOneToOneMap; //1-1
    std::map<Index_t, std::set<Index_t>> mOneToNMap; //1-N
    std::map<Index_t,std::set<Index_t>> mNToOneMap; //N-1

    ///Update, New, Delete
    std::set<Index_t> deleteLaterObjectSet;

    TrackingAlgorithmParamter param;
    std::vector<std::unique_ptr<singleObjectTracker>> tracks;
};

#endif // DATADRIVEMAIN_H
