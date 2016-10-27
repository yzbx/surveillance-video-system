#include "singleobjecttracker.h"

singleObjectTracker::singleObjectTracker(const trackingObjectFeature &of, track_t dt, track_t Accel_noise_mag, size_t trackID, int frameNum, int img_width, int img_height):
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
    img_width(img_width),
    img_height(img_height),
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
        // may out of image range!
        Rect_t newRect(lastRect.x+diff.x,lastRect.y+diff.y,lastRect.width,lastRect.height);
        if(newRect.x<0) newRect.x=0;
        else if(newRect.x>=img_width) newRect.x=img_width-1;

        if(newRect.y<0) newRect.y=0;
        else if(newRect.y>=img_height) newRect.y=img_height-1;
        rects.push_back(newRect);
    }

    if (trace.size() > max_trace_length)
    {
        trace.erase(trace.begin(), trace.end() - max_trace_length);
    }

    trace.push_back(prediction);
    lifetime++;
    assert(lifetime==(catch_frames+skipped_frames));
}

void singleObjectTracker::NormalUpdate(const trackingObjectFeature &of, int frameNum,const cv::Mat &img_normal)
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

    img_last_normal=img_normal.clone();
    mask_last_normal=of.mask.clone();
    rect_last_normal=of.rect;
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
    // may out of image range!
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
//    Rect_t old_rect=feature->rect;
//    Point_t p=yzbxlib::rectCenter(of.rect)-yzbxlib::rectCenter(old_rect);
//    Point2i displace((int)round(p.x),(int)round(p.y));

    //move mask to new position by displace and fit in size for of.rect
//    yzbxlib::moveMaskAndFitRect(of.mask,displace,of.rect);
//    Mat showMat;
//    cvtColor(of.mask,showMat,CV_GRAY2BGR);
//    rectangle(showMat,old_rect,Scalar(255,0,0),2,8);
//    rectangle(showMat,of.rect,Scalar(0,0,255),2,8);
//    yzbxlib::showImageInWindow("move mask",showMat);

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
void singleObjectTracker::B2B_NToOneUpdate(trackingObjectFeature &of, int frameNum)
{
    KF.GetPrediction();
    prediction = KF.Update(of.pos,true);

    status=NToOne_STATUS;
    vec_status.push_back(status);
    frames.push_back(frameNum);

    feature->copy(of);
    rects.push_back(of.rect);
    skipped_frames++;

    trace.push_back(prediction);
    lifetime++;

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
