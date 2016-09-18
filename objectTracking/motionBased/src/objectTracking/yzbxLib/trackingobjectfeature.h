#ifndef TRACKINGOBJECTFEATURE_H
#define TRACKINGOBJECTFEATURE_H
#include <opencv2/opencv.hpp>
#include "yzbx_config.h"
#include <QtCore>

class trackingObjectFeature
{
public:
    trackingObjectFeature();

    void copy(const trackingObjectFeature &of){
        pos=of.pos;
        rect=of.rect;
        size=of.size;
        Circularity=of.Circularity;
        Convexity=of.Convexity;
        Inertia=of.Inertia;
        radius=of.radius;
        onBoundary=of.onBoundary;
        if(of.LIFMat.empty()){
            assert(of.LIFPos.empty());
            LIFMat.release();
            LIFPos.clear();
            LIFColor.clear();
        }
        else{
            assert(!of.LIFPos.empty());
            assert(!of.LIFColor.empty());
            assert(of.LIFMat.rows==of.LIFColor.size());
            LIFMat=of.LIFMat.clone();
            LIFPos=of.LIFPos;
            LIFColor=of.LIFColor;
        }
        KLTPos=of.KLTPos;
        mask=of.mask.clone();
    }

    Point_t pos;
    Rect_t rect;

    //size is area, and range from [minArea,+Inf]
    track_t size;
    //range from [0,+Inf]
    track_t Convexity;
    track_t Circularity;
    track_t Inertia;
    track_t radius;
    bool onBoundary;

    QString dump();
    void fromString(QString str);

    ///LIFMat.rows==LIFPos.size()==LIFColor.size();
    cv::Mat LIFMat;
    std::vector<Point_t> LIFPos;
    std::vector<cv::Point3i> LIFColor;
    std::vector<cv::Point2i> KLTPos;
    cv::Mat mask;
};

#endif // TRACKINGOBJECTFEATURE_H
