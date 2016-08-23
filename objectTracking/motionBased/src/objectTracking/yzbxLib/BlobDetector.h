#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H
#include <opencv2/opencv.hpp>
#include "trackingobjectfeature.h"
using namespace cv;
class BlobDetector
{
public:
    BlobDetector();
    ~BlobDetector();
    void getBlobFeature(InputArray _image, InputArray _binaryImage, std::vector<trackingObjectFeature> &features);
    SimpleBlobDetector::Params params;
};

#endif // BLOBDETECTOR_H
