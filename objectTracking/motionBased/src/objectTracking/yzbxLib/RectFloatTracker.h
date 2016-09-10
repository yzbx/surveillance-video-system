#ifndef RECTFLOATTRACKER_H
#define RECTFLOATTRACKER_H
#include "HungarianBasedTracking.h"
#include "ObjectLocalFeatureMatch.h"
#include "TrackingAlgorithmParamter.h"
#include <boost/lexical_cast.hpp>

class RectFloatTracker : public QThread
{
public:
    enum CostType{
        RectDist,
        LIFDist
    };
    typedef uint Index_t;
    typedef uint Id_t;
    RectFloatTracker();
    void process(const cv::Mat &img_input,const cv::Mat &img_fg,vector<trackingObjectFeature> &fv);
private:
    TrackingAlgorithmParamter param;
    std::vector<std::unique_ptr<singleObjectTracker>> tracks;

    int globalChannel;
    std::list<std::pair<cv::Mat,cv::Mat>> imageList;
    std::list<std::vector<trackingObjectFeature>> featureVectorList;
    int maxListLength;

    void runInSingleThread();
    void run();

    void showing(const cv::Mat &img_input, const cv::Mat &img_fg, std::vector<trackingObjectFeature> featureVector);

    //return the match feature number for objects-blobs
    void getLocalFeatureAssignment_step1(cv::Mat &matchMat);
    void showAssignment(assignments_t &assignments, std::vector<trackingObjectFeature> &fv);
    track_t calcMatchedFeatureNum(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2);
    void getHungarainAssignment(assignments_t &assignment, int costType=RectDist);
    track_t calcCost(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2, int costType);

    //get from UrbanTracker, temporary status
    std::map<Index_t, std::set<Index_t>> mObjectToBlobMap;
    std::map<Index_t, std::set<Index_t>> mBlobToObjectMap;
    std::set<Index_t> mMatchedBlobSet;
    std::set<Index_t> mMatchedObjectSet;

    std::set<Index_t> mZeroToOneSet;	 // 0-1
    std::set<Index_t> mOneToZeroSet; // 1-0
    std::map<Index_t, Index_t> mOneToOneMap; //1-1
    std::map<Index_t, std::set<Index_t>> mOneToNMap; //1-N
    std::map<Index_t,std::set<Index_t>> mNToOneMap; //N-1

    //split provocation
    const int MaxProvocationTimesForSplit=3;
    const int MaxProvocationTimesForMerge=10;
    const track_t MinSplitGap=100;
    const track_t MaxDistForMergingTrace=100.0;
    const int StableFeatureNumber=3;
    const uint InitMergeChance=2;

    ///Long history status, when detete objects, merge objects, split objects, we need update them!!!
    //NOTE must use objectId here, because trackIdx will be invalid when remove some object
    //std::map<objectId,std::pair<provocation times,displacement>>
    std::map<Id_t,std::pair<uint,Point_t>> splitProvocationMap;
    //std::map<objectIdA,std::map<objectIdB,mergedTimes>>, objectIdA < objectIdB
    //mergeChance=2,1,0; 0 for objects with cannot merge!
//    std::map<Id_t,std::map<Id_t,uint>> mergeChanceMap;
    std::map<Id_t,std::map<Id_t,uint>> mergeProvocationMap;
    std::set<Id_t> deleteLaterObjectSet;

    ///use mNewblobs, mUnmatchedObjects, mOneToOneMap, ... to do assignment
    void doAssignment_step3();
    void handleOneToOneObjects();

    void handleNewObjects();
    void handleMissedObjects();
    //NOTE split first, then merge!!!
    void handleOneToNObjects();
    //NOTE merge later, split first!!!
    void handleNToOneObjects();



    bool isTraceMergable(vector<Point_t> &traceA, vector<Point_t> &traceB);
    void getUnmatchedHungarainAssignment_step2(cv::Mat matchMat);
    track_t getRectGap(Rect_t ra, Rect_t rb);
    void showMatchedFeature();
    void showBlobFeature();
};

#endif // RECTFLOATTRACKER_H
