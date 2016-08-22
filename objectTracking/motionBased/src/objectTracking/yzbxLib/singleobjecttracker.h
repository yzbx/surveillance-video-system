#ifndef SINGLEOBJECTTRACKER_H
#define SINGLEOBJECTTRACKER_H
#include <opencv2/opencv.hpp>
#include <QtCore>
#include "trackingobjectfeature.h"
#include "Kalman.h"

class singleObjectTracker
{
public:
    singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID);
    TKalmanFilter KF;
    int track_id;
    std::vector<Point_t> trace;
    int skipped_frames;
    Point_t prediction;
    trackingObjectFeature *feature;

    track_t CalcDist(trackingObjectFeature &of);
    void Update(const trackingObjectFeature &of, bool dataCorrect, size_t max_trace_length);


};

#endif // SINGLEOBJECTTRACKER_H
