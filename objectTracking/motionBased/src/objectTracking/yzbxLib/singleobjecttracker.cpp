#include "singleobjecttracker.h"

singleObjectTracker::singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID):
    track_id(trackID),
    skipped_frames(0),
    prediction(of.pos),
    KF(of.pos, dt, Accel_noise_mag)
{
    feature=new trackingObjectFeature;
    feature->pos=of.pos;
    feature->rect=of.rect;
    feature->size=of.size;
}

track_t singleObjectTracker::CalcDist(trackingObjectFeature &of)
{
    Point_t dif=prediction-of.pos;
    track_t distP=sqrtf(dif.x * dif.x + dif.y * dif.y);
    cv::Rect lastRect=this->feature->rect;
    cv::Rect r=of.rect;

    std::array<track_t, 4> diff;
    diff[0] = prediction.x - lastRect.width / 2 - r.x;
    diff[1] = prediction.y - lastRect.height / 2 - r.y;
    diff[2] = static_cast<track_t>(lastRect.width - r.width);
    diff[3] = static_cast<track_t>(lastRect.height - r.height);

    track_t dist = 0;
    for (size_t i = 0; i < diff.size(); ++i)
    {
        dist += diff[i] * diff[i];
    }
    track_t distR=sqrtf(dist);

    track_t weight=distR+distP;
    return weight;
}

void singleObjectTracker::Update(const trackingObjectFeature &of, bool dataCorrect, size_t max_trace_length)
{

    KF.GetPrediction();
    prediction = KF.Update(of.pos, dataCorrect);

    if (dataCorrect)
    {
        feature->rect = of.rect;
    }

    if (trace.size() > max_trace_length)
    {
        trace.erase(trace.begin(), trace.end() - max_trace_length);
    }

    trace.push_back(prediction);
}
