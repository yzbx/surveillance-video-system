#include "singleobjecttracker.h"

singleObjectTracker::singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID, int frameNum):
    track_id(trackID),
    skipped_frames(0),
    catch_frames(1),
    lifetime(1),
    status(NEW_STATUS),
    splitMergeType(NORMAL_SMTYPE),
    AvoidUpdate(false),
    firstSeePos(of.pos),
    lastSeePos(of.pos),
    prediction(of.pos),
    KF(of.pos, dt, Accel_noise_mag)
{
    feature=std::make_unique<trackingObjectFeature>(trackingObjectFeature());
    feature->copy(of);

    rects.push_back(of.rect);
    trace.push_back(of.pos);
    frames.push_back(frameNum);
    vec_status.push_back(status);
}

track_t singleObjectTracker::CalcDist(trackingObjectFeature &of)
{
    Point_t dif=prediction-of.pos;
    track_t distP=sqrtf(dif.x * dif.x + dif.y * dif.y);
    Rect_t lastRect=this->feature->rect;
    Rect_t r=of.rect;
    track_t distR=yzbxlib::getOverlapDist(lastRect,r);
//    std::array<track_t, 4> diff;
//    diff[0] = prediction.x - lastRect.width / 2 - r.x;
//    diff[1] = prediction.y - lastRect.height / 2 - r.y;
//    diff[2] = static_cast<track_t>(lastRect.width - r.width);
//    diff[3] = static_cast<track_t>(lastRect.height - r.height);

//    track_t dist = 0;
//    for (size_t i = 0; i < diff.size(); ++i)
//    {
//        dist += diff[i] * diff[i];
//    }
//    track_t distR=sqrtf(dist);

//    track_t sizeRatio;
//    {
//        if(of.onBoundary||feature->onBoundary){
//            sizeRatio=1;
//        }
//        else{
//           float ratio=of.size/feature->size;
//           ratio=(ratio+1/ratio)/2;
//           float tolerateRatio=0.2;
//           if(ratio<1-tolerateRatio||ratio>1+tolerateRatio){
//               sizeRatio=1+(1-ratio)*(1-ratio)-tolerateRatio*tolerateRatio;
//           }
//           else{
//               sizeRatio=1;
//           }
//        }
//    }

    track_t weight=distR*distP;
    return weight;
}

void singleObjectTracker::Update(const trackingObjectFeature &of, bool dataCorrect, size_t max_trace_length)
{
    assert(false);
    KF.GetPrediction();
    prediction = KF.Update(of.pos, dataCorrect);

    if (dataCorrect)
    {
//        feature=&of;
        status=NORMAL_STATUS;
//        of.copyTo(feature);
        feature->copy(of);

        rects.push_back(of.rect);
        catch_frames+=(1+skipped_frames);
        skipped_frames=0;
    }
    else{
        status=MISSING_STATUS;
        skipped_frames++;
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
    lifetime++;
    assert(lifetime==(catch_frames+skipped_frames));
}

void singleObjectTracker::NormalUpdate(const trackingObjectFeature &of, int frameNum)
{

    KF.GetPrediction();
    prediction = KF.Update(of.pos,true);
    if(AvoidUpdate){
        AvoidUpdate=false;
        return;
    }

    status=NORMAL_STATUS;
    vec_status.push_back(status);
    frames.push_back(frameNum);
    feature->copy(of);
    rects.push_back(of.rect);
    catch_frames+=(1+skipped_frames);
    skipped_frames=0;

    trace.push_back(prediction);
    lifetime++;
    lastSeePos=of.pos;
    assert(lifetime==1+frameNum-frames[0]);
    assert(lifetime==(catch_frames+skipped_frames));
}

void singleObjectTracker::MissUpdate(int frameNum)
{
    KF.GetPrediction();
    prediction = KF.Update(Point_t(0,0),false);
    if(AvoidUpdate){
        AvoidUpdate=false;
        return;
    }

    status=MISSING_STATUS;
    vec_status.push_back(status);
    frames.push_back(frameNum);
    skipped_frames++;
    Rect_t lastRect=rects.back();
    Point_t diff=prediction-feature->pos;
    //BUG may out of image range!
    rects.push_back(Rect_t(lastRect.x+diff.x,lastRect.y+diff.y,lastRect.width,lastRect.height));

    trace.push_back(prediction);
    lifetime++;
//    lastSeePos=of.pos;
    assert(lifetime==1+frameNum-frames[0]);
    assert(lifetime==(catch_frames+skipped_frames));
}

void singleObjectTracker::OneToNUpdateForBiggestBlob(const trackingObjectFeature &of, int frameNum)
{
    KF.GetPrediction();
    prediction = KF.Update(of.pos,true);
    if(AvoidUpdate){
        AvoidUpdate=false;
        return;
    }

    status=OneToN_STATUS;
    vec_status.push_back(status);
    frames.push_back(frameNum);
    feature->copy(of);
    rects.push_back(of.rect);
    catch_frames+=(1+skipped_frames);
    skipped_frames=0;

    trace.push_back(prediction);
    lifetime++;
    lastSeePos=of.pos;
    assert(lifetime==1+frameNum-frames[0]);
    assert(lifetime==(catch_frames+skipped_frames));
}

void singleObjectTracker::OneToNUpdateForNewBlob()
{
    status=OneToN_STATUS;
}

void singleObjectTracker::NToOneUpdate(trackingObjectFeature &of, int frameNum)
{
    KF.GetPrediction();
    prediction = KF.Update(of.pos,true);
    if(AvoidUpdate){
        AvoidUpdate=false;
        return;
    }

    status=NToOne_STATUS;
    vec_status.push_back(status);
    frames.push_back(frameNum);
    feature->copy(of);
    rects.push_back(of.rect);
    catch_frames+=(1+skipped_frames);
    skipped_frames=0;

    trace.push_back(prediction);
    lifetime++;
//    lastSeePos=of.pos;
    assert(lifetime==1+frameNum-frames[0]);
    assert(lifetime==(catch_frames+skipped_frames));
}

void singleObjectTracker::PreUpdateForBiggestBlob(const trackingObjectFeature &of)
{
    assert(false);
    feature->copy(of);
    status=PREUPDATE_STATUS;
}

void singleObjectTracker::AvoidUpdateTwice()
{
    assert(false);
    AvoidUpdate=true;
}

Rect_t singleObjectTracker::GetPredictRect()
{
    Rect_t lastRect=rects.back();
    Point_t diff=prediction-feature->pos;
    return Rect_t(lastRect.x+diff.x,lastRect.y+diff.y,lastRect.width,lastRect.height);
}

void singleObjectTracker::predict(trackingObjectFeature &of){
    of.Circularity=this->feature->Circularity;
    of.Convexity=this->feature->Convexity;
    of.Inertia=feature->Inertia;
    of.onBoundary=feature->onBoundary;
    of.pos=KF.GetPrediction();
    Rect_t r=feature->rect;
    Point_t dif=(2*of.pos-(r.br()+r.tl()));
    dif.x*=0.5f;
    dif.y*=0.5f;
    Point_t tl=r.tl()+dif;
    of.rect=Rect_t(tl.x,tl.y,r.width,r.height);
    of.radius=feature->radius;
    of.size=feature->size;
}
