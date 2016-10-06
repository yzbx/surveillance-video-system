#ifndef RECTFLOATTRACKER_H
#define RECTFLOATTRACKER_H
#include "HungarianBasedTracking.h"
#include "ObjectLocalFeatureMatch.h"
#include "TrackingAlgorithmParamter.h"
#include <boost/lexical_cast.hpp>

class RectFloatTracker : public QThread
{
public:
    RectFloatTracker();
    void process(const cv::Mat &img_input,const cv::Mat &img_fg,vector<trackingObjectFeature> &fv);
    void setRecordFile(QString recordFile){
        globalRecordFile=recordFile;
    }
    void setFrameNum(int n){
        frameNum=n;
    }

private:
    int frameNum;
    QString globalRecordFile;
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
    void getLocalFeatureAssignment_step2(cv::Mat &matchMat);
    //add merge and split to object tracking system.
    void getRectOverlapAssignment_step1();
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
    const track_t MinSplitGap=5;
    const track_t MaxDistForMergingTrace=100.0;
    const int StableFeatureNumber=3;
    const uint InitMergeChance=2;
    const uint MaxFreshObjectLifeTime=5;
    const uint MinDumpLifeTime=10;
    uint NextTrackID;

    ///Long history status, when detete objects, merge objects, split objects, we need update them!!!
    // must use objectId here, because trackIdx will be invalid when remove some object
    //std::map<objectId,std::pair<provocation times,displacement>>
    std::map<Id_t,std::pair<uint,Point_t>> splitProvocationMap;
    //std::map<objectIdA,std::map<objectIdB,mergedTimes>>, objectIdA < objectIdB
    //mergeChance=2,1,0; 0 for objects with cannot merge!
//    std::map<Id_t,std::map<Id_t,uint>> mergeChanceMap;
    std::map<Id_t,std::map<Id_t,uint>> mergeProvocationMap;
    std::set<Id_t> deleteLaterObjectSet;

    ///use mNewblobs, mUnmatchedObjects, mOneToOneMap, ... to do assignment
    void doAssignment_step4();
    void handleOneToOneObjects();

    void handleNewObjects();
    void handleMissedObjects();
    // split first, then merge!!!
    void handleOneToNObjects();
    // merge later, split first!!!
    void handleNToOneObjects();

    bool isTraceMergable(vector<Point_t> &traceA, vector<Point_t> &traceB);
    void getUnmatchedHungarainAssignment_step3(cv::Mat matchMat);
    track_t getRectGap(Rect_t ra, Rect_t rb);
    void showMatchedFeature(const Mat matchMat);
    void showBlobFeature();
    void doRectOverlap_splitTrack(Index_t trackIdx, std::set<Index_t> &blobset);
    void doRectOverlap_splitBlob(Index_t blobIdx, std::set<Index_t> &trackset);
    void showNewBlobFeature();
    void showKalmanFeature();
    void showAssignment();
    void dumpDeleteObject(Index_t trackIdx);
};

#endif // RECTFLOATTRACKER_H
