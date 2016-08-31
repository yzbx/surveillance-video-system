#ifndef RECTFLOATTRACKER_H
#define RECTFLOATTRACKER_H
#include "HungarianBasedTracking.h"

class RectFloatTracker : public HungarianBasedTracking
{
public:
    RectFloatTracker();
    void tracking(const cv::Mat &img_input,const cv::Mat &img_fg);
private:
    assignments_t getHungarainAssignment(vector<trackingObjectFeature> &fv);
    void runInSingleThread();
    void run();
    int globalChannel;
    void doAssignment(assignments_t assignment, vector<trackingObjectFeature> &fv);
    void outputAndRemove(uint index);
};

#endif // RECTFLOATTRACKER_H
