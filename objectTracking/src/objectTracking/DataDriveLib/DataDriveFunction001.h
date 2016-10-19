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

track_t calcDist(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2, int costType);
}


#endif // DATADRIVEFUNCTION001_H
