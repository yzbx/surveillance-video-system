#ifndef QYZBXMASKFEATURES_H
#define QYZBXMASKFEATURES_H
#include "yzbx_utility.h"

class qyzbxMaskFeatures
{
public:
    qyzbxMaskFeatures();
    vector<Mat> getObjectsFromMask(const Mat &fgMask, bool show=false);
    cv::Point2d getMomentFromObject(const Mat &objMask);
};

#endif // QYZBXMASKFEATURES_H
