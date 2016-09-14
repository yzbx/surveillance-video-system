#ifndef SINGLEOBJECTTRACKER_H
#define SINGLEOBJECTTRACKER_H
#include <opencv2/opencv.hpp>
#include <QtCore>
#include "trackingobjectfeature.h"
#include "Kalman.h"
#include "yzbx_config.h"
#include "yzbx_utility.h"

class singleObjectTracker
{
public:
    singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID,int frameNum=0);
    TKalmanFilter KF;
    int track_id;
    std::vector<Point_t> trace;
    std::vector<Rect_t> rects;
    std::vector<STATUS> vec_status;
    std::vector<int> frames;
    Point_t prediction;
    trackingObjectFeature *feature;
    STATUS status;
    Point_t bornPos;
    enum SplitMergeType splitMergeType;
private:
    int skipped_frames;
    int catch_frames;
    int lifetime;
    bool AvoidUpdate=false;
public:
    int get_skipped_frames(){
        return skipped_frames;
    }
    int get_catch_frames(){
        return catch_frames;
    }
    int get_lifetime(){
        return lifetime;
    }

    void set_skipped_frames(int a){
        skipped_frames=a;
    }
    void set_catch_frames(int a){
        catch_frames=a;
    }
    void set_lifetime(int a){
        lifetime=a;
    }

    track_t CalcDist(trackingObjectFeature &of);
    void Update(const trackingObjectFeature &of, bool dataCorrect, size_t max_trace_length);
    void NormalUpdate(const trackingObjectFeature &of, int frameNum);
    void MissUpdate(int frameNum);
    void PreUpdateForBiggestBlob(const trackingObjectFeature &of);
    void AvoidUpdateTwice();
    Rect_t GetPredictRect();
    QString dump(){
        QStringList dumpstrs;
        dumpstrs<<QString::number(track_id)
               <<QString::number(status)
               <<QString::number(prediction.x)
               <<QString::number(prediction.y)
               <<QString::number(skipped_frames)
              <<feature->dump();
        return dumpstrs.join("\t");
    }
    void predict(trackingObjectFeature &of);
};

#endif // SINGLEOBJECTTRACKER_H
