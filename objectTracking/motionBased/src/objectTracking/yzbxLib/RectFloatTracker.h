#ifndef RECTFLOATTRACKER_H
#define RECTFLOATTRACKER_H
#include "HungarianBasedTracking.h"
#include "ObjectLocalFeatureMatch.h"
#include <boost/lexical_cast.hpp>

class RectFloatTracker : public HungarianBasedTracking
{
public:
    RectFloatTracker();
    void process(const cv::Mat &img_input,const cv::Mat &img_fg,vector<trackingObjectFeature> &fv);
    void tracking(const cv::Mat &img_input,const cv::Mat &img_fg);
private:
    int globalChannel;
    std::list<std::pair<cv::Mat,cv::Mat>> imageList;
    std::list<std::vector<trackingObjectFeature>> featureVectorList;
    int maxListLength;

    assignments_t getHungarainAssignment(vector<trackingObjectFeature> &fv);
    void runInSingleThread();
    void run();
    void doAssignment(assignments_t assignment, vector<trackingObjectFeature> &fv, Mat assignmentMat);
    void outputAndRemove(uint index);
    bool isRectAInRectB(Rect_t A, Rect_t B);
    void showing(const cv::Mat &img_input, const cv::Mat &img_fg, std::vector<trackingObjectFeature> featureVector);
    void getLocalFeatureAssignment(cv::Mat &assignmentMat);
    track_t calcPathWeight(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2, ObjectLocalFeatureMatch &matcher);
    bool isPointInRect(cv::Point2f p, Rect_t rect);
    void showAssignment(assignments_t &assignments, std::vector<trackingObjectFeature> &fv);
    track_t calcPathWeight(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2);
    void getHungarainAssignment(assignments_t &assignment);
};

#endif // RECTFLOATTRACKER_H
