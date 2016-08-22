#ifndef TRACKINGOBJECTFEATURE_H
#define TRACKINGOBJECTFEATURE_H
#include <opencv2/opencv.hpp>

typedef float track_t;
typedef cv::Point_<track_t> Point_t;
#define Mat_t CV_32FC

class trackingObjectFeature
{
public:
    trackingObjectFeature();
    Point_t pos;
    track_t size;
    cv::Rect rect;
};

#endif // TRACKINGOBJECTFEATURE_H
