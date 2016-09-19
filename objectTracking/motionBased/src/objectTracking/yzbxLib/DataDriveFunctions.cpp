#include "DataDriveFunctions.h"

namespace DataDrive {

bool Input::run()
{
    data->frameInput.getNextFrame(data->videoFilePath,data->img_input);
    data->frameNum++;
    if(data->img_input.empty()) return false;
    else return true;
}

bool Bgs::run()
{
    assert(!data->img_input.empty());
    if(!data->img_fg.empty()) data->img_fg.release();

    cv::Mat img_fg;
    data->bgs->process(data->img_input,img_fg,data->img_background);
    if(img_fg.empty()){
        return false;
    }
    else{
        if(img_fg.channels()==3){
            cv::cvtColor(img_fg,data->img_fg,CV_BGR2GRAY);
        }
        else{
            data->img_fg=img_fg.clone();
        }

        img_fg.release();
        return true;
    }
}

bool BlobFeature::run()
{
    assert(!data->img_fg.empty());
    assert(!data->img_input.empty());
    assert(data->fvlist.size()==data->imglist.size());
    vector<trackingObjectFeature> fv;
    data->blobFeatureDetector.getBlobFeature(data->img_input,data->img_fg,fv);
    data->fv=fv;
    data->fvlist.push_back(fv);
    data->imglist.push_back(std::make_pair(data->img_input,data->img_fg));

    if(data->fvlist.size()>data->MaxListLength){
        data->fvlist.pop_front();
        data->imglist.pop_front();
    }
    return true;
}

bool Tracker::run()
{
    return true;
}

bool KLTTracker::run()
{

    if(data->fvlist.size()<2){
        return false;
    }

    assert(data->img_input.channels()==3);
    auto it=std::prev(data->imglist.end(),2);
    cv::Mat gray,prev_gray;
    cv::cvtColor(data->img_input,gray,CV_BGR2GRAY);
    cv::cvtColor(it->first,prev_gray,CV_BGR2GRAY);

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    const int MAX_COUNT = 500;
    vector<Point2f> points[2];
    //    goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
    //    cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
    goodFeaturesToTrack(prev_gray, points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
    cornerSubPix(prev_gray, points[0], subPixWinSize, Size(-1,-1), termcrit);

    vector<uchar> status;
    vector<float> err;
    calcOpticalFlowPyrLK(prev_gray, gray, points[0], points[1], status, err, winSize,
            3, termcrit, 0, 0.001);

    assert(points[0].size()==points[1].size());
    size_t i, k;

    Mat image;
    data->img_input.copyTo(image);
    for( i = k = 0; i < points[1].size(); i++ )
    {
        circle( image, points[1][i], 3, Scalar(255,0,0), -1, 8);
        if( !status[i] )
            continue;

        points[0][k]=points[0][i];
        points[1][k++] = points[1][i];
        circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
        line(image,points[0][i],points[1][i],Scalar(0,0,255),3,8);
    }
    points[1].resize(k);

    cv::namedWindow("KLT",WINDOW_NORMAL);
    imshow("KLT",image);

    waitKey(30);
    return true;
}

bool MeanShiftTracker::run()
{
    static vector<trackingObjectFeature> fv;
    static cv::Mat choosedImage;
    static int key;
    if(data->fvlist.size()<2){
        return false;
    }
    assert(data->img_input.channels()==3);
    if(key=='n'||fv.empty()){
        fv=data->fvlist.back();
        choosedImage=data->img_input.clone();
        key='0';
    }
    Mat image=data->img_input.clone();

    Mat hsv,hue,hist,backproj,mask;

    int _vmin = 0, _vmax = 255,smin=0;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    int hsize = 16;
    int ch[] = {0, 0};
    cvtColor(image, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
            Scalar(180, 256, MAX(_vmin, _vmax)), mask);
    hue.create(hsv.size(), hsv.depth());
    mixChannels(&hsv, 1, &hue, 1, ch, 1);

    cv::Mat local_hsv,local_hue,local_mask,local_hist;
    cvtColor(choosedImage, local_hsv, COLOR_BGR2HSV);
    inRange(local_hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
            Scalar(180, 256, MAX(_vmin, _vmax)), local_mask);
    local_hue.create(local_hsv.size(), local_hsv.depth());
    mixChannels(&local_hsv, 1, &local_hue, 1, ch, 1);

    for(int i=0;i<fv.size();i++){
        Rect_t rect=fv[i].rect;
        Rect trackWindow=Rect((int)rect.x,(int)rect.y,(int)rect.width,(int)rect.height);
        if(key=='n'||fv.empty()){

            Mat roi(local_hue, trackWindow), maskroi(local_mask, trackWindow);
            calcHist(&roi, 1, 0, maskroi, local_hist, 1, &hsize, &phranges);
            normalize(local_hist, local_hist, 0, 255, CV_MINMAX);

            calcBackProject(&hue, 1, 0, local_hist, backproj, &phranges);
            backproj &= local_mask;

        }
        else{
            Mat roi(hue, trackWindow), maskroi(mask, trackWindow);
            calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
            normalize(hist, hist, 0, 255, CV_MINMAX);

            calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
            backproj &= mask;
        }

        meanShift(backproj, trackWindow,
                  TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
        rectangle(image, trackWindow, Scalar(0,0,255), 3, CV_AA );
    }

    cv::namedWindow("CamShift",WINDOW_NORMAL);
    imshow("CamShift",image);

    key=waitKey(30);
    return true;
}

bool CamShiftTracker::run()
{
    static vector<trackingObjectFeature> fv;
    static cv::Mat choosedImage;
    static int key;
    if(data->fvlist.size()<2){
        return false;
    }
    assert(data->img_input.channels()==3);
    if(key=='n'||fv.empty()){
        fv=data->fvlist.back();
        choosedImage=data->img_input.clone();
        key='0';
    }
    Mat image=data->img_input.clone();

    Mat hsv,hue,hist,backproj,mask;

    int _vmin = 0, _vmax = 255,smin=0;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    int hsize = 16;
    int ch[] = {0, 0};
    cvtColor(image, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
            Scalar(180, 256, MAX(_vmin, _vmax)), mask);
    hue.create(hsv.size(), hsv.depth());
    mixChannels(&hsv, 1, &hue, 1, ch, 1);

    cv::Mat local_hsv,local_hue,local_mask,local_hist;
    cvtColor(choosedImage, local_hsv, COLOR_BGR2HSV);
    inRange(local_hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
            Scalar(180, 256, MAX(_vmin, _vmax)), local_mask);
    local_hue.create(local_hsv.size(), local_hsv.depth());
    mixChannels(&local_hsv, 1, &local_hue, 1, ch, 1);

    for(int i=0;i<fv.size();i++){
        Rect_t rect=fv[i].rect;
        Rect trackWindow=Rect((int)rect.x,(int)rect.y,(int)rect.width,(int)rect.height);
        RotatedRect trackBox;
        if(key=='n'||fv.empty()){

            Mat roi(local_hue, trackWindow), maskroi(local_mask, trackWindow);
            calcHist(&roi, 1, 0, maskroi, local_hist, 1, &hsize, &phranges);
            normalize(local_hist, local_hist, 0, 255, CV_MINMAX);

            calcBackProject(&hue, 1, 0, local_hist, backproj, &phranges);
            backproj &= local_mask;

        }
        else{
            Mat roi(hue, trackWindow), maskroi(mask, trackWindow);
            calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
            normalize(hist, hist, 0, 255, CV_MINMAX);

            calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
            backproj &= mask;
        }

        trackBox = CamShift(backproj, trackWindow,
                            TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
        ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
    }

    cv::namedWindow("CamShift",WINDOW_NORMAL);
    imshow("CamShift",image);

    key=waitKey(30);
    return true;
}

bool KLTAssignment::run()
{
    if(data->fvlist.size()<2){
//        assert(false);
        return false;
    }

    if(!data->prevBlobToTrack.empty())data->prevBlobToTrack.clear();
    if(!data->trackToPrevBlob.empty())data->trackToPrevBlob.clear();
    if(!data->KLTMatchMat.empty())data->KLTMatchMat.release();
    if(!data->blobToTrackSet.empty())data->blobToTrackSet.clear();
    if(!data->matchedBlobSet.empty())data->matchedBlobSet.clear();
    if(!data->matchedTrackSet.empty())data->matchedTrackSet.clear();
    if(!data->trackToBlobSet.empty())data->trackToBlobSet.clear();


    vector<trackingObjectFeature> &fv=data->fvlist.back();
    vector<trackingObjectFeature> prev_fv;

    //associate prevBlob to track
    {
        std::map<Index_t,Index_t> prevBlobToTrack;
        std::map<Index_t,Index_t> trackToPrevBlob;
        for(int i=0;i<data->tracks.size();i++){
            if(data->tracks[i]->status!=MISSING_STATUS){
                prev_fv.push_back(*(data->tracks[i]->feature));
                prevBlobToTrack[prev_fv.size()-1]=i;
                trackToPrevBlob[i]=prev_fv.size()-1;
            }
        }
        data->prevBlobToTrack.swap(prevBlobToTrack);
        data->trackToPrevBlob.swap(trackToPrevBlob);
    }

    if(fv.empty()||prev_fv.empty())
    {
        data->prevBlobToTrack.clear();
        data->trackToPrevBlob.clear();
        if(!data->KLTMatchMat.empty()) data->KLTMatchMat.release();

        return true;
    }

    assert(!fv.empty()&&!prev_fv.empty());
    assert(data->img_input.channels()==3);


    auto it=std::prev(data->imglist.end(),2);
    cv::Mat gray,prev_gray;
    cv::cvtColor(data->img_input,gray,CV_BGR2GRAY);
    cv::cvtColor(it->first,prev_gray,CV_BGR2GRAY);

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    const int MAX_COUNT = 500;
    vector<Point2f> points[2];
    goodFeaturesToTrack(prev_gray, points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
    cornerSubPix(prev_gray, points[0], subPixWinSize, Size(-1,-1), termcrit);

    vector<uchar> status;
    vector<float> err;
    calcOpticalFlowPyrLK(prev_gray, gray, points[0], points[1], status, err, winSize,
            3, termcrit, 0, 0.001);

    assert(points[0].size()==points[1].size());
    size_t i, k;

    Mat image;
    data->img_input.copyTo(image);
    //NOTE for matchNum > 255
    cv::Mat matchMat(prev_fv.size(),fv.size(),CV_8UC1);
    matchMat=Scalar::all(0);

    for( i = k = 0; i < points[1].size(); i++ )
    {
        circle( image, points[1][i], 3, Scalar(255,0,0), -1, 8);
        if( !status[i] )
            continue;

        points[0][k]=points[0][i];
        points[1][k++] = points[1][i];
        circle(image,points[1][i], 3, Scalar(0,255,0), -1, 8);
        line(image,points[0][i],points[1][i],Scalar(0,0,255),3,8);

        Point2i p=points[1][i];
        Point2i prev_p=points[0][i];
        int matchMat_x=-1,matchMat_y=-1;
        for(int i=0;i<prev_fv.size();i++){
            cv::Mat mask=prev_fv[i].mask;

            assert(!mask.empty());
            if(mask.at<uchar>(prev_p.y,prev_p.x)==255){
                matchMat_y=i;
                break;
            }
        }
        for(int i=0;i<fv.size();i++){
            cv::Mat mask=fv[i].mask;

            assert(!mask.empty());
            if(mask.at<uchar>(p.y,p.x)==255){
                matchMat_x=i;
                break;
            }
        }

        if(matchMat_x==-1||matchMat_y==-1){
            //out of mask
        }
        else{
            matchMat.at<uchar>(matchMat_y,matchMat_x)+=((uchar)1);
        }
    }
    data->KLTMatchMat=matchMat;
    yzbxlib::showImageInWindow("matchMat",matchMat);
    //update blob track assignment
    {
        std::map<Index_t,std::set<Index_t>> &blobToTrackSet=data->blobToTrackSet;
        std::map<Index_t,std::set<Index_t>> &trackToBlobSet=data->trackToBlobSet;
        std::set<Index_t> &matchedBlobSet=data->matchedBlobSet;
        std::set<Index_t> &matchedTrackSet=data->matchedTrackSet;

        int m=matchMat.rows;
        int n=matchMat.cols;
        const uint MatchNumThreshold=data->param.MatchNumThreshold;
        for(int i=0;i<m;i++){
            for(int j=0;j<n;j++){
                if(matchMat.at<uchar>(i,j)>MatchNumThreshold){
                    int blobIdx=j;
                    int objIdx=data->prevBlobToTrack[i];
                    blobToTrackSet[blobIdx].insert(objIdx);
                    trackToBlobSet[objIdx].insert(blobIdx);
                    matchedBlobSet.insert(blobIdx);
                    matchedTrackSet.insert(objIdx);
                }
            }
        }
    }

    points[1].resize(k);

    cv::namedWindow("KLT",WINDOW_NORMAL);
    imshow("KLT",image);

    waitKey(30);

    gray.release();
    prev_gray.release();
    image.release();
    return true;
}

bool UpdateTrackStatus::run()
{
    return true;
}

bool SplitAndMerge::run()
{
    if(data->fvlist.size()<2){
        assert(false);
    }
    vector<trackingObjectFeature> &fv=data->fvlist.back();

    bool emptyBlobs=fv.empty();
    bool emptyObjects=data->tracks.empty();

    if(!data->mOneToZeroSet.empty())data->mOneToZeroSet.clear();
    if(!data->mZeroToOneSet.empty())data->mZeroToOneSet.clear();
    if(!data->mOneToNMap.empty())data->mOneToNMap.clear();
    if(!data->mOneToOneMap.empty())data->mOneToOneMap.clear();
    if(!data->mNToOneMap.empty())data->mNToOneMap.clear();

    if(emptyBlobs&&emptyObjects){
        //        qDebug()<<"empty Blobs and empty objects";
    }
    else if(emptyBlobs){
        //handle missed object
        for(int i=0;i<data->tracks.size();i++){
            data->mOneToZeroSet.insert(i);
        }
        handleMissedObjects();
    }
    else if(emptyObjects){
        //handle new object
        for(int i=0;i<fv.size();i++){
            data->mZeroToOneSet.insert(i);
        }
        handleNewObjects();
    }
    else{
        ///update mOneToOneMap mNToOneMap mOneToNMap
        auto &mObjectToBlobMap=data->trackToBlobSet;
        auto &mBlobToObjectMap=data->blobToTrackSet;
        for(auto ia=mObjectToBlobMap.begin();ia!=mObjectToBlobMap.end();ia++){
            //tracksIdx
            Index_t trackIdx=ia->first;
            if(ia->second.size()==1){
                Index_t fvIdx=*(ia->second.begin());
                if(mBlobToObjectMap[fvIdx].size()==1){
                    data->mOneToOneMap[trackIdx]=fvIdx;
                }
                else{
                    data->mNToOneMap[fvIdx]=mBlobToObjectMap[fvIdx];
                }
            }
            else{
                data->mOneToNMap[trackIdx]=ia->second;
            }
        }

        auto &mMatchedObjectSet=data->matchedTrackSet;
        ///handle missed object, not change tracks.size()
        for(int i=0;i<data->tracks.size();i++){
            if(mMatchedObjectSet.find(i)==mMatchedObjectSet.end()){
                data->mOneToZeroSet.insert(i);
            }
        }
        handleMissedObjects();

        auto &mMatchedBlobSet=data->matchedBlobSet;
        //handle new object, will change tracks.size()
        for(int i=0;i<fv.size();i++){
            if(mMatchedBlobSet.find(i)==mMatchedBlobSet.end()){
                data->mZeroToOneSet.insert(i);
            }
        }
        handleNewObjects();


        //handle one to one object
        handleOneToOneObjects();
        //handle split object
        handleOneToNObjects();
        //handle merged object
        handleNToOneObjects();

        auto &deleteLaterObjectSet=data->deleteLaterObjectSet;
        ///remove objects which need delete later!!!
        for(auto it=data->tracks.begin();it!=data->tracks.end();it++){
            Id_t id=(*it)->track_id;
            if(deleteLaterObjectSet.find(id)!=deleteLaterObjectSet.end()){
                //                dumpDeleteObject(it-tracks.begin());
                auto pre=prev(it,1);
                data->tracks.erase(it);
                it=pre;
            }
        }
        deleteLaterObjectSet.clear();

    }

//    waitKey(0);
    return true;
}

void SplitAndMerge::handleOneToOneObjects()
{
    vector<trackingObjectFeature> &fv=data->fvlist.back();
    for(auto it=data->mOneToOneMap.begin();it!=data->mOneToOneMap.end();it++){
        int trackIdx=it->first;
        int fvIdx=it->second;
        data->tracks[trackIdx]->NormalUpdate(fv[fvIdx],data->frameNum);
    }
}

void SplitAndMerge::handleNewObjects()
{
    vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto &param=data->param;
    for(auto newit=data->mZeroToOneSet.begin();newit!=data->mZeroToOneSet.end();newit++){
        data->tracks.push_back(std::make_unique<singleObjectTracker>(fv[*newit],
                               param.dt, param.Accel_noise_mag, param.NextTrackID++,data->frameNum));
    }
}

void SplitAndMerge::handleMissedObjects()
{
    auto &mOneToZeroSet=data->mOneToZeroSet;
    auto &param=data->param;
    auto &deleteLaterObjectSet=data->deleteLaterObjectSet;
    for(auto missit=mOneToZeroSet.begin();missit!=mOneToZeroSet.end();missit++){
        assert(*missit<data->tracks.size());
        data->tracks[*missit]->MissUpdate(data->frameNum);
        if(data->tracks[*missit]->get_lifetime()<param.MaxFreshObjectLifeTime||
                data->tracks[*missit]->get_skipped_frames()>param.maximum_allowed_skipped_frames){
            deleteLaterObjectSet.insert(data->tracks[*missit]->track_id);
        }
    }
}

void SplitAndMerge::handleOneToNObjects()
{
    vector<trackingObjectFeature> &fv=data->fvlist.back();

    ///1. the blob is away from other blobs
    ///2. the blob is go from faraway to nearby, size from small to big!
    ///3. the blob split for many times when moving!

    auto &mOneToNMap=data->mOneToNMap;
    auto &param=data->param;
    for(auto it=mOneToNMap.begin();it!=mOneToNMap.end();it++){
        Index_t trackIdx=it->first;
        std::set<Index_t> &blobset=(it->second);
        /// empty set will be remove, size()==1 is error status!
        assert(blobset.size()>1);

        //get max blob, assign it to origin objectIdx
        float maxArea=0;
        Index_t maxAreaBlobIdx;
        for(auto blob=blobset.begin();blob!=blobset.end();blob++){
            if(blob==blobset.begin()){
                maxArea=fv[*blob].size;
                maxAreaBlobIdx=*blob;
            }
            else{
                if(maxArea<fv[*blob].size){
                    maxArea=fv[*blob].size;
                    maxAreaBlobIdx=*blob;
                }
            }
        }
        //change blob feature
        data->tracks[trackIdx]->PreUpdateForBiggestBlob(fv[maxAreaBlobIdx]);

        for(auto blob=blobset.begin();blob!=blobset.end();blob++){
            if(*blob!=maxAreaBlobIdx){
                data->tracks.push_back(std::make_unique<singleObjectTracker>(fv[*blob],param.dt,
                                       param.Accel_noise_mag,param.NextTrackID++,data->frameNum));
                data->tracks.back()->AvoidUpdateTwice();
            }
        }
    }
}

void SplitAndMerge::handleNToOneObjects()
{
    vector<trackingObjectFeature> &fv=data->fvlist.back();

    auto &mNToOneMap=data->mNToOneMap;
    auto &param=data->param;
    for(auto it=mNToOneMap.begin();it!=mNToOneMap.end();it++){
        Index_t blobIdx=it->first;
        std::set<Index_t> &trackset=it->second;

        Rect_t rb=fv[blobIdx].rect;
        for(auto track=trackset.begin();track!=trackset.end();track++){
//            assert(data->tracks[*track]->status==PREUPDATE_STATUS||
//                    data->tracks[*track]->get_lifetime()>=param.MaxFreshObjectLifeTime);

            trackingObjectFeature of;
            of.copy(fv[blobIdx]);
            of.pos=data->tracks[*track]->KF.GetPrediction();
            Rect_t predict_rect=data->tracks[*track]->feature->rect;
            Rect_t subRect=yzbxlib::getSubRect(rb,predict_rect);
            of.rect=subRect;
            of.pos+=(of.rect.tl()-predict_rect.tl());

            if(track==trackset.begin()){
                fv[blobIdx]=of;
            }
            else{
                fv.push_back(of);
            }
            assert(data->tracks[*track]->feature->LIFMat.empty());
        }
    }
}

track_t HungarianAssignment::calcDist(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2, int costType)
{
    if(costType==RectDist){
        Rect_t &r1=of1->rect;
        Rect_t &r2=of2->rect;
        Point_t p1=0.5f*(r1.tl()+r1.br());
        Point_t p2=0.5f*(r2.tl()+r2.br());
        track_t dist=fabs(r1.width-r2.width)+fabs(r1.height-r2.height)+cv::norm(p1-p2);

        return dist;
    }
    else if(costType==PosDist){
        Point_t p1=of1->pos;
        Point_t p2=of2->pos;
        return cv::norm(p1-p2);
    }
    else if(costType==KLTDist){
        assert(false);
        return -1.0f;
    }
    else{
        assert(false);
        return -1.0f;
    }
}

bool HungarianAssignment::run()
{
    if(data->fvlist.size()<2){
        return false;
    }

    vector<trackingObjectFeature> &fv=data->fvlist.back();

    std::set<Index_t> &mMatchedObjectSet=data->matchedTrackSet;
    std::set<Index_t> &mMatchedBlobSet=data->matchedBlobSet;

    int m=data->tracks.size();
    int n=fv.size();
    int unMatchedObjectNum=data->tracks.size()-mMatchedObjectSet.size();
    int unMatchedBlobNum=fv.size()-mMatchedBlobSet.size();
    if(unMatchedBlobNum==0||unMatchedObjectNum==0){
        //            qDebug()<<"nothing for unmatched hungarain";
    }
    else{
        //vector<trackIdx>
        vector<Index_t> unMatchedObjects;
        //vector<featureVectorIdx>
        vector<Index_t> unMatchedBlobs;

        for(int i=0;i<m;i++){
            if(mMatchedObjectSet.find(i)==mMatchedObjectSet.end()){
                unMatchedObjects.push_back(i);
            }
        }
        for(int j=0;j<n;j++){
            if(mMatchedBlobSet.find(j)==mMatchedBlobSet.end()){
                unMatchedBlobs.push_back(j);
            }
        }

        size_t N = unMatchedObjects.size();
        size_t M = unMatchedBlobs.size();
        assert(N>0&&M>0);

        distMatrix_t Cost(N * M);
        for(int i=0;i<unMatchedObjects.size();i++){
            for(int j=0;j<unMatchedBlobs.size();j++){
                Cost[i+j*N]=calcDist(std::make_shared<trackingObjectFeature>(*(data->tracks[unMatchedObjects[i]]->feature)),
                        std::make_shared<trackingObjectFeature>(fv[unMatchedBlobs[j]]),RectDist);
            }
        }

        AssignmentProblemSolver APS;
        assignments_t unMatchedAssignment;
        APS.Solve(Cost, N, M, unMatchedAssignment, AssignmentProblemSolver::optimal);
        for (size_t i = 0; i < unMatchedAssignment.size(); i++)
        {
            if (unMatchedAssignment[i] != -1)
            {
                if (Cost[i + unMatchedAssignment[i] * N] > data->param.dist_thres)
                {
                    unMatchedAssignment[i] = -1;
                }
                else{
                    Index_t trackIdx=unMatchedObjects[i];
                    Index_t featureVectorIdx=unMatchedBlobs[unMatchedAssignment[i]];
                    data->trackToBlobSet[trackIdx].insert(featureVectorIdx);
                    data->blobToTrackSet[featureVectorIdx].insert(trackIdx);
                    data->matchedBlobSet.insert(featureVectorIdx);
                    data->matchedTrackSet.insert(trackIdx);
                }
            }
        }

    }

    return true;
}

bool ShowAssignment::run()
{
    if(data->fvlist.size()<2){
        assert(false);
    }

    vector<trackingObjectFeature> &fv=data->fvlist.back();
    Mat showImg=data->img_input.clone();
    //show track
    {
        for(int i=0;i<data->tracks.size();i++){
            Rect rect=data->tracks[i]->feature->rect;
            int id=data->tracks[i]->track_id;
            string title=boost::lexical_cast<string>(id);
            yzbxlib::annotation(showImg,rect,title);
            rectangle(showImg,rect,Scalar(255,0,0),3,8);
        }
    }

    //show blob
    {
        for(int i=0;i<fv.size();i++){
            Rect rect=fv[i].rect;
//            string title=boost::lexical_cast<string>(i);
//            yzbxlib::annotation(showImg,rect,title);
            rectangle(showImg,rect,Scalar(0,255,0),3,8);
        }
    }

    //show track To Blob
    {
        for(auto it=data->trackToBlobSet.begin();it!=data->trackToBlobSet.end();it++){
            Index_t trackIdx=it->first;
            set<Index_t> &blobset=it->second;
            Point p1=data->tracks[trackIdx]->feature->pos;
            for(auto it=blobset.begin();it!=blobset.end();it++){
                Index_t blobIdx=*it;
                Point p2=fv[blobIdx].pos;
                cv::line(showImg,p1,p2,Scalar(0,0,255),3,8);
            }
        }
    }

    namedWindow("assignment",WINDOW_NORMAL);
    imshow("assignment",showImg);
    return true;
}

}
