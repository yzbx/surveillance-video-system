#ifndef GRAPHBASEDTRACKER_H
#define GRAPHBASEDTRACKER_H
#include <QtCore>
#include "yzbx_config.h"
#include <opencv2/opencv.hpp>
#include <BlobDetector.h>
#include <singleobjecttracker.h>
#include "HungarianBasedTracking.h"
#include "BasicGraphClass.h"
#include "ObjectLocalFeatureMatch.h"


class GraphBasedTracker: public HungarianBasedTracking
{

public:  
    typedef std::shared_ptr<BasicGraphNode> pNode;
    typedef vector<std::shared_ptr<BasicGraphNode>> Path;
    typedef vector<Path> PathsForOneNode;
    typedef vector<PathsForOneNode> PathsForAllNodes;
    GraphBasedTracker(){
        NextTrackID=0;
        dt=0.2f;
        Accel_noise_mag=0.1f;

        //if (Cost[i + assignment[i] * N] > dist_thres)
        dist_thres = 100;

        maximum_allowed_skipped_frames = 100;
        max_trace_length=100;

        globalFirstDump=true;
        globalFirstOutput=true;
        frameNum=0;
        outputFileName="out.txt";
        FirstFG_frameNum=0;
    }

    void tracking(const cv::Mat &img_input, const cv::Mat &img_fg);

    track_t GetDistance(shared_ptr<trackingObjectFeature> of1, shared_ptr<trackingObjectFeature> of2);
    track_t calcPathWeight(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2, ObjectLocalFeatureMatch &matcher);
    bool splitOrNewTrackId(pNode nextHeadNode);
    void dumpSplitInformation(pNode headNode, pNode nextNode);
protected:
    BasicGraphClass featureGraph;
    track_t split_threshold=3.0;

    void GraphBasedTracking();
    void run();

    int maxlistLength=5;

    void updateGraphAssociation();
    vector<vector<std::shared_ptr<BasicGraphNode> > > findMaximumWeightedPaths();
    void updateTrajectory(vector<Path> &maxWeightPaths);
    std::list<std::pair<cv::Mat,cv::Mat>> imageList;
private:
    bool isPointInRect(cv::Point2f p, Rect_t rect);
};

#endif // GRAPHBASEDTRACKER_H
