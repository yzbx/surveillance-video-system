#ifndef TRACKINGBLOBSMATCHANNOTATION_H
#define TRACKINGBLOBSMATCHANNOTATION_H
#include <QtCore>
#include "HungarianAlg.h"

class TrackingBlobsMatchAnnotation
{
public:
    TrackingBlobsMatchAnnotation();
    void process(int frameNum, track_t dist_thres, std::vector<trackingObjectFeature> &fv, QString annotationTxt, std::vector<int> &ids);
private:
    track_t getRectDistance(Rect_t blob_r, Rect_t annotation_r);
};

#endif // TRACKINGBLOBSMATCHANNOTATION_H
