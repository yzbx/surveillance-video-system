#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H
#include <opencv2/opencv.hpp>
#include "trackingobjectfeature.h"
#include "yzbx_utility.h"
#include "ObjectLocalFeatureMatch.h"
//#include "qyzbxlib.h"

using namespace cv;
class BlobDetector
{
public:
    BlobDetector();
    ~BlobDetector();
    void process(InputArray _image, InputArray _binaryImage, std::vector<trackingObjectFeature> &features);
    void getBlobFeature(InputArray _image, InputArray _binaryImage, std::vector<trackingObjectFeature> &features);
    void showBlobFeature(const Mat &input,const Mat &mask,const trackingObjectFeature &of);
    SimpleBlobDetector::Params params;

private:
    double maxObjectSize=0;
    size_t cols=0,rows=0;
};

#endif // BLOBDETECTOR_H
