#include "trackingkalman.h"

TrackingKalman::TrackingKalman(cv::Point2d pt, float dt, float noiseMagnitude)
{
    //time increment (lower values makes target more "massive")
    deltatime = dt; //0.2

    // We don't know acceleration, so, assume it to process noise.
    // But we can guess, the range of acceleration values thich can be achieved by tracked object.
    // Process noise. (standard deviation of acceleration: m/s^2)
    // shows, woh much target can accelerate.
    //float Accel_noise_mag = 0.5;

    //4 state variables, 2 measurements
    kalman = new cv::KalmanFilter( 4, 2, 0 );
    // Transition cv::Matrix
    kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, deltatime, 0, 0, 1, 0, deltatime, 0, 0, 1, 0, 0, 0, 0, 1);

    // init...
    LastResult = pt;
    kalman->statePre.at<float>(0) = pt.x; // x
    kalman->statePre.at<float>(1) = pt.y; // y

    kalman->statePre.at<float>(2) = 0;
    kalman->statePre.at<float>(3) = 0;

    kalman->statePost.at<float>(0) = pt.x;
    kalman->statePost.at<float>(1) = pt.y;

    cv::setIdentity(kalman->measurementMatrix);

    kalman->processNoiseCov = (cv::Mat_<float>(4, 4) <<
                               pow(deltatime,4.0)/4.0	,0						,pow(deltatime,3.0)/2.0		,0,
                               0						,pow(deltatime,4.0)/4.0	,0							,pow(deltatime,3.0)/2.0,
                               pow(deltatime,3.0)/2.0	,0						,pow(deltatime,2.0)			,0,
                               0						,pow(deltatime,3.0)/2.0	,0							,pow(deltatime,2.0));


    kalman->processNoiseCov*=noiseMagnitude;

    setIdentity(kalman->measurementNoiseCov, cv::Scalar::all(0.1));

    setIdentity(kalman->errorCovPost, cv::Scalar::all(.1));
}

TrackingKalman::~TrackingKalman()
{
    delete kalman;
}

cv::Point2d TrackingKalman::GetPrediction()
{
    cv::Mat prediction = kalman->predict();
    LastResult = cv::Point2d(prediction.at<float>(0), prediction.at<float>(1));
    return LastResult;
}

cv::Point2d TrackingKalman::Update(cv::Point2d p, bool DataCorrect)
{
    cv::Mat measurement(2, 1, CV_64FC(1));
    if(!DataCorrect)
    {
        measurement.at<float>(0) = LastResult.x;  //update using prediction
        measurement.at<float>(1) = LastResult.y;
    }
    else
    {
        measurement.at<float>(0) = p.x;  //update using measurements
        measurement.at<float>(1) = p.y;
    }
    // Correction
    cv::Mat estiMated = kalman->correct(measurement);
    LastResult.x = estiMated.at<float>(0);   //update using measurements
    LastResult.y = estiMated.at<float>(1);
    return LastResult;
}
