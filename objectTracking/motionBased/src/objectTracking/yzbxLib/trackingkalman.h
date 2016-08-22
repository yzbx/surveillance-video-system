#ifndef TRACKINGKALMAN_H
#define TRACKINGKALMAN_H
#include <opencv2/opencv.hpp>

class TrackingKalman
{
public:
    TrackingKalman(cv::Point2d pt, float dt=0.2, float noiseMagnitude=0.5);
    ~TrackingKalman();
    cv::KalmanFilter* kalman;
    float deltatime;
    cv::Point2d LastResult;
    cv::Point2d GetPrediction();
    cv::Point2d Update(cv::Point2d p, bool DataCorrect);
};

#endif // TRACKINGKALMAN_H
