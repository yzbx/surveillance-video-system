#ifndef QYZBXTRACKINGFEATURES_H
#define QYZBXTRACKINGFEATURES_H
#include "yzbx_utility.h"
class ObjectFeature{
public:
    ObjectFeature()
    {

    }
    cv::Point2f pos;
    cv::Point2f pos_predict;
    float size;
};

class FrameFeature{
public:
    int frameNum;
    vector<ObjectFeature> features;
};

class QYzbxTrackingFeature
{
public:
    QYzbxTrackingFeature();
    void getObjects(const cv::Mat &img_input, const cv::Mat &fgMask, FrameFeature &frameFeatures);
    vector<Mat> getObjectsFromMask(const Mat &fgMask, bool show=false);
    cv::Point2f getMomentFromObject(const Mat &objMask);
};

#endif // QYZBXTRACKINGFEATURES_H
