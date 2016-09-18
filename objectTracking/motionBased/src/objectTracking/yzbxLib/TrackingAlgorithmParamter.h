#ifndef TRACKINGALGORITHMPARAMTER_H
#define TRACKINGALGORITHMPARAMTER_H
#include "yzbx_config.h"

class TrackingAlgorithmParamter
{
public:
    TrackingAlgorithmParamter();
    uint NextTrackID=0;
    track_t dt=0.2f;
    track_t Accel_noise_mag=0.1f;

    uint dist_thres = 100;

    uint maximum_allowed_skipped_frames = 100;
    uint max_trace_length=100;
    uint MatchNumThreshold=3;
    uint MaxFreshObjectLifeTime=5;
};

#endif // TRACKINGALGORITHMPARAMTER_H
