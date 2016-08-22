#ifndef MULTIOBJECTTRACKING_H
#define MULTIOBJECTTRACKING_H
#include "singleobjecttracker.h"
#include "../lib/tracking_yzbx.h"
#include "HungarianAlg.h"

class MultiObjectTracking : public Tracking_yzbx
{
public:
    MultiObjectTracking();
    void process(QString configFile,QString videoFile,TrackingStatus *status=NULL);
    void processOne(const cv::Mat& img_input,cv::Mat &img_foreground,cv::Mat &img_background,TrackingStatus *status);
    void run();
    void stop();
    ~MultiObjectTracking();

    bool globalStop=false;
    TrackingStatus *globalTrackingStatus;
    //    vector<singleFrameTracker> tracks;
    std::vector<std::unique_ptr<singleObjectTracker>> tracks;

    int NextTrackID=0;
    vector<trackingObjectFeature> getObjects(const cv::Mat &img_input, const cv::Mat &fgMask);

    void Tracking(vector<trackingObjectFeature> &fv);

    cv::Rect m_minObjectSize;
    track_t dist_thres = 60;
    size_t maximum_allowed_skipped_frames = 10;
    size_t max_trace_length = 10;
    //CTracker tracker(0.2f, 0.1f, 60.0f, 10, 50);
    // Шаг времени опроса фильтра
    track_t dt=0.2f;

    track_t Accel_noise_mag=0.1f;
};

#endif // MULTIOBJECTTRACKING_H
