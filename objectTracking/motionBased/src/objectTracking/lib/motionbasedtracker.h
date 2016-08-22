#ifndef MOTIONBASEDTRACKER_H
#define MOTIONBASEDTRACKER_H
#include "tracking_yzbx.h"
#include "../yzbxLib/qyzbxTrackingFeatures.h"
#include "../yzbxLib/trackingObjectAssociation.h"
#include "../yzbxLib/HungarianAlg.h"

class singleFrameTracker{
public:
    singleFrameTracker(ObjectFeature &of, int id);
    ~singleFrameTracker();
    void Update(ObjectFeature &of,bool dataCorrect, size_t max_trace_length);
    float CalcDistance(ObjectFeature &b);
    ObjectFeature *objectFeature;
    cv::KalmanFilter pos_kalman;
    int TrackID;
    vector<cv::Point2d> trace;
    int skipped_frames;
};

class MotionBasedTracker: public Tracking_yzbx
{
    Q_OBJECT
public:
    MotionBasedTracker();
    void process(QString configFile, QString videoFile,TrackingStatus* status=NULL);
    void processOne(const cv::Mat& img_input,cv::Mat &img_foreground,cv::Mat &img_background,TrackingStatus *status);
    void objectTracking(DirectedGraph &g);
    void singleFrameTracking(FrameFeature &ff);
    void initBgs(TrackingStatus *status);

    void run();
    void stop();

    bool globalStop=false;
    TrackingStatus *globalTrackingStatus;
//    vector<singleFrameTracker> tracks;
    std::vector<std::unique_ptr<singleFrameTracker>> tracks;
    int NextTrackID=0;
    float Distance_Threshold=5.0;
    int maximum_allowed_skipped_frames=20;
    int max_trace_length=100;
};

#endif // MOTIONBASEDTRACKER_H
