#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H
#include <opencv2/opencv.hpp>
#include "trackingobjectfeature.h"
#include "yzbx_utility.h"
using namespace cv;
class BlobDetector
{
public:
    BlobDetector();
    ~BlobDetector();
    void getBlobFeature(InputArray _image, InputArray _binaryImage, std::vector<trackingObjectFeature> &features);
    SimpleBlobDetector::Params params;

private:
    double maxObjectSize=0;
    size_t cols=0,rows=0;
};

#endif // BLOBDETECTOR_H
