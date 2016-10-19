#ifndef DATADRIVEFUNCTION001_H
#define DATADRIVEFUNCTION001_H
#include "DataDriveMain.h"
#include "yzbx_config.h"
#include <memory>

namespace DataDrive {
void dumpRectInTracker(const std::vector<std::unique_ptr<singleObjectTracker>> &tracks);

void dumpRect(Rect_t rect);

void drawRectInTracker(const std::vector<std::unique_ptr<singleObjectTracker>> &tracks, const cv::Mat &img, string title="drawRectInTracker");

void dumpObjects(const std::vector<std::unique_ptr<singleObjectTracker>> &tracks, bool output=true);

void updateMatchedFeature(const std::shared_ptr<DataDriveMain> &data,ObjectFeature &of);

track_t calcDist(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2, int costType)
{
    if(costType==RectDist){
        Rect_t &r1=of1->rect;
        Rect_t &r2=of2->rect;
        Point_t p1=0.5f*(r1.tl()+r1.br());
        Point_t p2=0.5f*(r2.tl()+r2.br());
        track_t dist=fabs(r1.width-r2.width)+fabs(r1.height-r2.height)+cv::norm(p1-p2);

        return dist;
    }
    else if(costType==PosDist){
        Point_t p1=of1->pos;
        Point_t p2=of2->pos;
        return cv::norm(p1-p2);
    }
    else if(costType==KLTDist){
        assert(false);
        return -1.0f;
    }
    else{
        assert(false);
        return -1.0f;
    }
}
}


#endif // DATADRIVEFUNCTION001_H
