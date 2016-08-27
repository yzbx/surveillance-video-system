#include "singleobjecttracker.h"

singleObjectTracker::singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID):
    track_id(trackID),
    skipped_frames(0),
    prediction(of.pos),
    KF(of.pos, dt, Accel_noise_mag)
{
    feature=new trackingObjectFeature;

    feature->Circularity=of.Circularity;
    feature->Convexity=of.Convexity;
    feature->Inertia=of.Inertia;
    feature->onBoundary=of.onBoundary;
    feature->pos = of.pos;
    feature->radius=of.radius;
    feature->rect = of.rect;
    feature->size=of.size;

    status=NEW_STATUS;

    skipped_frames=0;
    catch_frames=1;
    rects.push_back(of.rect);
}

track_t singleObjectTracker::CalcDist(trackingObjectFeature &of)
{
    Point_t dif=prediction-of.pos;
    track_t distP=sqrtf(dif.x * dif.x + dif.y * dif.y);
    Rect_t lastRect=this->feature->rect;
    Rect_t r=of.rect;

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

    track_t sizeRatio;
    {
        if(of.onBoundary||feature->onBoundary){
            sizeRatio=1;
        }
        else{
           float ratio=of.size/feature->size;
           ratio=(ratio+1/ratio)/2;
           float tolerateRatio=0.2;
           if(ratio<1-tolerateRatio||ratio>1+tolerateRatio){
               sizeRatio=1+(1-ratio)*(1-ratio)-tolerateRatio*tolerateRatio;
           }
           else{
               sizeRatio=1;
           }
        }
    }

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
        feature->Circularity=of.Circularity;
        feature->Convexity=of.Convexity;
        feature->Inertia=of.Inertia;
        feature->onBoundary=of.onBoundary;
        feature->radius=of.radius;
        feature->size=of.size;
        feature->pos=of.pos;
        status=NORMAL_STATUS;

        rects.push_back(of.rect);
        catch_frames++;
    }
    else{
        status=MISSING_STATUS;
        Rect_t lastRect=rects.back();
        Point_t rectCenter(lastRect.x+lastRect.width/2,lastRect.y+lastRect.height/2);
        Point_t diff=prediction-rectCenter;
        //BUG may out of image range!
        rects.push_back(Rect_t(lastRect.x+diff.x,lastRect.y+diff.y,lastRect.width,lastRect.height));
    }

    if (trace.size() > max_trace_length)
    {
        trace.erase(trace.begin(), trace.end() - max_trace_length);
    }

    trace.push_back(prediction);
}
