#ifndef BLOBBASEDTRACKER_H
#define BLOBBASEDTRACKER_H
#include "singleobjecttracker.h"
#include "../lib/tracking_yzbx.h"
#include "HungarianAlg.h"
#include "multiobjecttracking.h"
#include "BlobDetector.h"

class BlobBasedTracker: public MultiObjectTracking
{
public:
    BlobBasedTracker();
    ~BlobBasedTracker();
    void process(QString configFile,QString videoFile,TrackingStatus *status=NULL);
    void processOne(const cv::Mat& img_input,cv::Mat &img_foreground,cv::Mat &img_background,TrackingStatus *status);
    void run();

    BlobDetector blobDetector;
};

#endif // BLOBBASEDTRACKER_H
