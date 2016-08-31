#ifndef SINGLEOBJECTTRACKER_H
#define SINGLEOBJECTTRACKER_H
#include <opencv2/opencv.hpp>
#include <QtCore>
#include "trackingobjectfeature.h"
#include "Kalman.h"
#include "yzbx_config.h"

class singleObjectTracker
{
public:
    singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID);
    TKalmanFilter KF;
    int track_id;
    std::vector<Point_t> trace;
    std::vector<Rect_t> rects;
    int skipped_frames;
    int catch_frames;
    Point_t prediction;
    trackingObjectFeature *feature;
    STATUS status;
    Point_t bornPos;
    int bornFrameNum;

    track_t CalcDist(trackingObjectFeature &of);
    void Update(const trackingObjectFeature &of, bool dataCorrect, size_t max_trace_length);
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
