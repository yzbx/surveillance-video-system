#ifndef TRACKINGOBJECTFEATURE_H
#define TRACKINGOBJECTFEATURE_H
#include <opencv2/opencv.hpp>
#include "yzbx_config.h"
#include <QtCore>

class trackingObjectFeature
{
public:
    trackingObjectFeature();
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
};

#endif // TRACKINGOBJECTFEATURE_H
